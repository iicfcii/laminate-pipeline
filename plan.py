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

def not_web_material(laminate,up):
    num_layers = len(laminate)
    is_adhesive = [i%2 == 1 for i in range(num_layers)] # Assume alternative adhesive and first and last are not adhesive

    start = num_layers-1 if up else 0
    end = -1 if up else num_layers
    step = -1 if up else 1
    not_web_material = [laminate[start]]
    for i in range(start+step,end,step):
        not_web_material.append(laminate[i] | not_web_material[-1])
    not_web_material = Laminate(*not_web_material)
    not_web_material = not_web_material[::step]

    # Adhesive above/below other layers cannot be used as web
    for i in range(start,end,step):
        if is_adhesive[i]:
            not_web_material[i] |= not_web_material[i+step]

    # not_web_material.plot_layers()
    # plt.show(block=True)

    return not_web_material

def cuts(device,jig_diameter=5,jig_hole_spacing=20, clearance=1):
    assert clearance > CUT_THICKNESS
    num_layers = len(device)
    is_adhesive = [i%2 == 1 for i in range(num_layers)] # assume alternative adhesive

    # Build jigholes and sheet
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

    # Identify material for web
    all_scrap = sheet-device
    web_material_up = all_scrap-(not_web_material(device,True)<<clearance)
    web_material_down = all_scrap-(not_web_material(device,False)<<clearance)
    web_material = web_material_up|web_material_down

    web = web_material-jig_holes # Web that holds the device before release cut
    keepout =  mfg.keepout_laser(device) # Keepout region that laser should never cut
    release_cut_scrap = sheet-keepout
    support = mfg.support(device,mfg.keepout_laser,clearance,0)
    supported_device = web|device|support

    # TODO: Allow cleanup with shapely buffer join_style setting to mitre
    supported_device = mfg.cleanup(supported_device,CUT_THICKNESS)

    # support.plot_layers()
    supported_device.plot_layers()
    release_cut_scrap.plot(new=True)
    plt.show(block=True)

    # Prepare cuts
    w, h = mfg.unary_union(supported_device).bounding_box().get_dimensions()

    layers_cut = supported_device[0]
    for i in range(1,num_layers):
        layers_cut |= supported_device[i].translate(0,(h+10)*i)

    release_cut = release_cut_scrap[0]

    return layers_cut, release_cut
