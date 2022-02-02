import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
import shapely.errors as se
import foldable_robotics.dxf as dxf
from foldable_robotics.layer import Layer
from foldable_robotics.laminate import Laminate
import foldable_robotics.manufacturing as mfg
import os, sys

import joint

SMALL_DIM = 0.001
CUT_THICKNESS = 0.1
CIRCLE_RESOLUTION = 5

def device(comps_poly,comps_circle,joints,layers_comp,joint_dicts=joint.DICTS):
    # Construct device
    device = []
    for l in layers_comp.keys():
        layer = sg.Polygon()
        for comp in layers_comp[l]:
            for p in comps_poly[comp][l]:
                 # TODO: Add logic to determine outer poly with bounding box
                layer |= sg.Polygon(p)
            for circle in comps_circle[comp][l]:
                # HACK Flip circle around y, bug may be related to the extrusion direction(0,0,-1)
                center = (-list(circle[0])[0],list(circle[0])[1])
                layer ^= sg.Point(center).buffer(circle[1],resolution=CIRCLE_RESOLUTION) # circles are cutout

        # Merge touching bodies
        layer = Layer(layer)
        layer = mfg.cleanup(layer,SMALL_DIM)
        device.append(layer)

    device = Laminate(*device)

    def joint_fun(j):
        if j['type'] in joint_dicts:
            return joint_dicts[j['type']]
        else:
            max_layer = np.amax(list(layers_comp.keys()))
            if max_layer == 4:
                return joint_dicts['plain5']
            elif max_layer == 0:
                return joint_dicts['dashed1']
            else:
                # If type not matched, used the first one in dict
                return joint_dicts[list(joint_dicts.keys())[0]]

    # Cut for forming joints in laminate
    joints_cut = Laminate(*[Layer()]*len(device))
    for j in joints:
        for line in j['lines']:
            jf = joint_fun(j)
            joint_block = jf(line, invert=True)
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
            jf = joint_fun(j)
            joint_block = jf(line, w=0.5, dl=-CUT_THICKNESS)
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

    # Proejct layer one by one to form a laminate that can not be used as web material.
    # Web material needs to be removable from above or below.
    not_web_material = [laminate[start]]
    for i in range(start+step,end,step):
        not_web_material.append(laminate[i] | not_web_material[-1])
    not_web_material = Laminate(*not_web_material)
    not_web_material = not_web_material[::step]

    for i in range(start,end-step,step):
        # Adhesive above/below other material cannot be used as web
        if is_adhesive[i]:
            not_web_material[i] |= not_web_material[i+step]
        # Material above/below adhesive cannot be used as web
        if is_adhesive[i+step]:
            not_web_material[i] |= not_web_material[i+step]

    return not_web_material

def jig_holes(x,y,w,h,jig_diameter,num_layers):
    points = [] # jig holes
    points.append(sg.Point(x-w/2,y-h/2).buffer(jig_diameter/2,resolution=CIRCLE_RESOLUTION))
    points.append(sg.Point(x-w/2,y+h/2).buffer(jig_diameter/2,resolution=CIRCLE_RESOLUTION))
    points.append(sg.Point(x+w/2,y-h/2).buffer(jig_diameter/2,resolution=CIRCLE_RESOLUTION))
    points.append(sg.Point(x+w/2,y+h/2).buffer(jig_diameter/2,resolution=CIRCLE_RESOLUTION))
    points = Layer(*points)
    points = points.to_laminate(num_layers)

    return points

def labels(x,y,w,h,jig_diameter,num_layers,thickness=0.2,gap=2,hide_lines=False):
    assert gap < jig_diameter

    circles = []
    for i in range(num_layers):
        c = Layer(sg.Point((x-gap,y-h/2)).buffer(gap/2,resolution=1))
        circles.append(c)

    if not hide_lines:
        lines = []
        for i in range(num_layers):
            ls = Layer()
            for j in range(i+1):
                xl = x+j*(thickness+gap)
                l = sg.LineString([
                    (xl,y-h/2-jig_diameter/2+thickness/2),
                    (xl,y-h/2+jig_diameter/2-thickness/2)
                ])
                l = l.buffer(thickness/2,cap_style=sg.CAP_STYLE.square)
                ls |= Layer(l)
            lines.append(ls)

        labels = Laminate(*lines) | Laminate(*circles)
    else:
        labels = Laminate(*circles)

    return labels

# This translate function will print out exception and try to reolve it
# instead of not giving any info and losing features
def safe_translate_layer(layer,dx,dy):
    l = sg.Polygon()
    for geom in layer.geoms:
        g = sa.translate(geom,dx,dy)
        try:
            l |= g
        except se.TopologicalError as e:
            print('Please check the cuts. Tried to resolve exception by simplifying the geom a bit.')
            if not l.is_valid: l = l.simplify(0.0001)
            if not g.is_valid: g = g.simplify(0.0001)
            l |= g
    return Layer(l)

def cuts(device,jig_diameter=5,jig_hole_spacing=20, clearance=1):
    assert clearance > 0
    num_layers = len(device)
    is_adhesive = [i%2 == 1 for i in range(num_layers)] # assume alternative adhesive

    # Build jigholes and sheet
    device_bb = (mfg.unary_union(device)<<jig_hole_spacing/2).bounding_box()
    w,h = device_bb.get_dimensions()
    w = round(w/jig_hole_spacing)*jig_hole_spacing
    h = round(h/jig_hole_spacing)*jig_hole_spacing

    (x1,y1),(x2,y2) = device_bb.bounding_box_coords()
    xc,yc = ((x2+x1)/2,(y2+y1)/2)

    holes = jig_holes(xc,yc,w,h,jig_diameter,num_layers)
    lines = labels(xc,yc,w,h,jig_diameter,num_layers)

    sheet = (holes[0]<<jig_diameter).bounding_box()
    sheet=sheet.to_laminate(num_layers)

    # Identify material for web
    all_scrap = sheet-device
    web_material_up = all_scrap-(not_web_material(device,True)<<clearance)
    web_material_down = all_scrap-(not_web_material(device,False)<<clearance)
    web_material = web_material_up|web_material_down

    web = web_material-holes-lines # Web that holds the device before release cut
    keepout =  mfg.keepout_laser(device) # Keepout region that laser should never cut
    release_cut_label = labels(xc,yc,w,h,jig_diameter,num_layers,hide_lines=True)
    release_cut_scrap = sheet-keepout-release_cut_label
    support = mfg.support(device,mfg.keepout_laser,clearance,0)
    # IDEA: Support can be within the keepout region if it is part of the web maeterial
    supported_device = web|device|support

    device_released = supported_device-release_cut_scrap.dilate(CUT_THICKNESS/2)
    material_cut = device_released.dilate(CUT_THICKNESS) & release_cut_scrap

    single_layer_cut = []  # Cuts that only happens on a single layer
    for i in range(num_layers):
        material_cut_n = material_cut[i]
        for i in range(num_layers):
            if i == 2: continue
            material_cut_n -= material_cut[i]
        material_cut_n.geoms = [g for g in material_cut_n.geoms if g.area > 1e-3]
        material_cut_n = material_cut_n.dilate(CUT_THICKNESS/2)

        material_cut_n_mpg = sg.MultiPolygon(material_cut_n.geoms)
        release_cut_lines = []

        def separate(b):
            pts = b.coords
            for p1, p2 in zip(pts,pts[1:]+[pts[0]]):
                release_cut_lines.append(sg.LineString([p1,p2]))

        for g in release_cut_scrap[0].geoms:
            if g.boundary.type == 'MultiLineString':
                for ls in g.boundary.geoms:
                    separate(ls)
            else:
                separate(g.boundary)
        release_cut_lines = sg.MultiLineString(release_cut_lines)
        material_cut_n_lines = release_cut_lines & material_cut_n_mpg
        material_cut_n_lines = Layer(material_cut_n_lines)
        single_layer_cut.append(material_cut_n_lines)
    single_layer_cut = Laminate(*single_layer_cut)

    return supported_device, release_cut_scrap, single_layer_cut

def export(path, supported_device, release_cut_scrap, single_layer_cut,plot=False):
    num_layers = len(supported_device)

    # Prepare cuts
    w, h = mfg.unary_union(supported_device).bounding_box().get_dimensions()

    layers_cut = supported_device[0]
    for i in range(1,num_layers):
        step = 10
        d = int(np.ceil(h/step)*i+i)
        layers_cut |= safe_translate_layer(supported_device[i],0,d*step)

    release_cut = release_cut_scrap[0]

    if plot:
        layers_cut.plot(new=True)
        release_cut.plot(new=True)
        plt.show(block=True)

    folder_name = os.path.basename(os.path.normpath(path))
    layers_cut.export_dxf(os.path.join(path,'{}_layers'.format(folder_name)))
    release_cut.export_dxf(os.path.join(path,'{}_release'.format(folder_name)))
