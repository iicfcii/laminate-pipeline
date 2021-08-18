import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as sg
import foldable_robotics.dxf as dxf
from foldable_robotics.layer import Layer
from foldable_robotics.laminate import Laminate
import foldable_robotics.manufacturing as mfg

import joint

SMALL_DIM = 0.001
CUT_THICKNESS = 0.1

def device(comps_poly,comps_circle,joints,layers_comp):
    # Construct device
    device = []
    for l in layers_comp.keys():
        layer = sg.Polygon()
        for comp in layers_comp[l]:
            for p in comps_poly[comp][l]:
                 # TODO: Add logic to determine outer poly with bounding box
                layer |= sg.Polygon(p)
            for circle in comps_circle[comp][l]:
                layer ^= sg.Point(list(circle[0])[0:2]).buffer(circle[1]) # circles are cutout

        # Merge touching bodies
        layer = Layer(layer)
        layer = mfg.cleanup(layer,SMALL_DIM)
        device.append(layer)

    device = Laminate(*device)

    # Cut for forming joints in laminate
    joints_cut = Laminate(*[Layer()]*len(device))
    for j in joints:
        for line in j['lines']:
            # TODO: Support different types of joint
            joint_block = joint.plain5(line, invert=True)
            start_layer = j['layer']-int(len(joint_block)/2)
            layers = []
            for i in range(len(device)):
                if i < start_layer:
                    layers.append(Layer())
                else:
                    if i-start_layer < len(joint_block):
                        layers.append(joint_block[i-start_layer])
                    else:
                        layers.append(Layer())
            laminate = Laminate(*layers)
            joints_cut |= laminate
    joints_cut = mfg.cleanup(joints_cut,SMALL_DIM)

    # Cut to separate all bodies
    bodies_cut = []
    for l in layers_comp:
        cut = sg.LineString()
        for comp in layers_comp[l]:
            for p in comps_poly[comp][l]:
                poly = sg.Polygon(p)
                cut |= poly.boundary
        cut = cut.buffer(CUT_THICKNESS/2,join_style=sg.JOIN_STYLE.mitre)
        bodies_cut.append(Layer(cut))
    bodies_cut = Laminate(*bodies_cut)

    # Mask to avoid cutting joints
    joints_mask = Laminate(*[Layer()]*len(device))
    for j in joints:
        for line in j['lines']:
            joint_block = joint.plain5(line, w=0.5, dl=-CUT_THICKNESS)
            start_layer = j['layer']-int(len(joint_block)/2)
            layers = []
            for i in range(len(device)):
                if i < start_layer:
                    layers.append(Layer())
                else:
                    if i-start_layer < len(joint_block):
                        layers.append(joint_block[i-start_layer])
                    else:
                        layers.append(Layer())
            laminate = Laminate(*layers)
            joints_mask |= laminate

    bodies_cut = bodies_cut-joints_mask
    device = device - joints_cut - bodies_cut

    # TODO: Clean unnecessary adhesive
    return device, joints_cut, bodies_cut

def cuts(device,jig_diameter=5,jig_hole_spacing=20):
    num_layers = len(device)
    is_adhesive = [i%2 == 1 for i in range(num_layers)] # assume alternative adhesive

    # Build jigholes and sheet
    # TODO: Default shapely buffer join_style to mitre
    device_bb = (mfg.unary_union(device)<<jig_hole_spacing/2).bounding_box()
    w,h = device_bb.get_dimensions()
    w = round(w/jig_hole_spacing)*jig_hole_spacing
    h = round(h/jig_hole_spacing)*jig_hole_spacing

    (x1,y1),(x2,y2) = device_bb.bounding_box_coords()
    xc,yc = ((x2+x1)/2,(y2+y1)/2)
    points = [] # jig holes
    points.append(sg.Point(xc-w/2,yc-h/2))
    points.append(sg.Point(xc-w/2,yc+h/2))
    points.append(sg.Point(xc+w/2,yc-h/2))
    points.append(sg.Point(xc+w/2,yc+h/2))
    jig_holes = Layer(*points)
    jig_holes <<= jig_diameter/2

    sheet = (jig_holes<<jig_diameter).bounding_box()
    sheet=sheet.to_laminate(num_layers)
    jig_holes=jig_holes.to_laminate(num_layers)

    # Identify removable scrap
    all_scrap = sheet-device
    removable_scrap_up = all_scrap-(mfg.not_removable_up(device,is_adhesive)<<CUT_THICKNESS)
    removable_scrap_down = all_scrap-(mfg.not_removable_down(device,is_adhesive)<<CUT_THICKNESS)

    # BUG: Bug details of not_removable_up/down algorithm are below.
    # If a part is adhered partly to the non-adhesive layers, it is still recognized as removable.

    # HACK: Manuanlly remove the part to generate correct cut files
    # patch = Layer(sg.Polygon([(-24,7.5),(-24,-7.5),(0,-7.5),(0,7.5)]))
    # removable_scrap_up[3] = removable_scrap_up[3]-patch<<0.1
    # removable_scrap_up.plot_layers()
    # plt.show(block=True)
    removable_scrap = removable_scrap_up|removable_scrap_down

    web = removable_scrap-jig_holes # Web that holds the device before release cut
    keepout =  mfg.keepout_laser(device) # Keepout region that laser should never cut
    release_cut_scrap = sheet-keepout
    layers_cut_scrap = mfg.cleanup(sheet-device-release_cut_scrap,0.001)
    support = mfg.support(device,mfg.keepout_laser,CUT_THICKNESS,0) # No clearance needed because cut already has thickness
    supported_device = web|device|support

    # removable_scrap_up.plot_layers()
    # device.plot_layers()
    # keepout.plot_layers()
    # support.plot_layers()
    # web.plot_layers()
    supported_device.plot_layers()
    release_cut_scrap.plot(new=True)
    plt.show(block=True)

    # Prepare cuts
    w, h = mfg.unary_union(supported_device).bounding_box().get_dimensions()

    layers_cut = supported_device[0]
    for i in range(1,num_layers):
        layers_cut |= supported_device[i].translate(0,(h+10)*i)

    release_cut = release_cut_scrap[0]

    # TODO: Convert thin polygons into lines
    # In each polygon, go over all the lines
    # If shorter or equal to cut thickness, remove
    # Else find the cloeset and parallel line,
    # remove the shorter one of the pair
    # translate the longer one so that it is in between the pair

    return layers_cut, release_cut
