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
import ezdxf

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
                # NOTE: Some polygon union may fail.
                # Dilate and erode fix the problem but not sure why
                layer |= sg.Polygon(p).buffer(SMALL_DIM).buffer(-SMALL_DIM)
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
            joint_block = joint.inv(jf,line)
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
            joint_block = jf(line)
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
    layers_cut = web|device|support

    device_released = layers_cut-release_cut_scrap.dilate(CUT_THICKNESS/2)
    material_cut = device_released.dilate(CUT_THICKNESS) & release_cut_scrap

    release_cut = [] # Release cuts in individual lines

    def separate(b):
        pts = b.coords
        for p1, p2 in zip(pts,pts[1:]+[pts[0]]):
            release_cut.append(sg.LineString([p1,p2]))

    for g in release_cut_scrap[0].geoms:
        if g.boundary.type == 'MultiLineString':
            for ls in g.boundary.geoms:
                separate(ls)
        else:
            separate(g.boundary)
    release_cut = Layer(sg.MultiLineString(release_cut))

    release_cut_layers = []  # Cuts that only happens on a single layer
    release_cut_layers_mpg = []
    for j in range(num_layers):
        material_cut_n = material_cut[j]
        for i in range(num_layers):
            if i == j: continue
            material_cut_n -= material_cut[i]

        material_cut_n.geoms = [g for g in material_cut_n.geoms if g.area > CUT_THICKNESS**2]
        material_cut_n = material_cut_n.erode(CUT_THICKNESS/10).dilate(CUT_THICKNESS/10) # clean very thin lines
        material_cut_n = material_cut_n.dilate(0.8) # Expand a bit to make sure all-the-way cuts won't affect single-layer cut

        material_cut_n_mpg = Layer(sg.MultiPolygon(material_cut_n.geoms))
        material_cut_n_lines = release_cut & material_cut_n_mpg

        release_cut_layers.append(material_cut_n_lines)
        release_cut_layers_mpg.append(material_cut_n_mpg)

    release_cut_layers = Laminate(*release_cut_layers)
    release_cut_layers_mpg = Laminate(*release_cut_layers_mpg)

    # Remove single layer cuts from the total cuts
    for rcl_mpg  in release_cut_layers_mpg: release_cut -= rcl_mpg

    # release_cut_layers_mpg[2].plot()
    # release_cut_layers[2].plot()
    # release_cut.plot()
    # plt.show(block=True)

    return layers_cut, release_cut, release_cut_layers

def export(path, layers_cut, release_cut, release_cut_layers, plot=False):
    num_layers = len(layers_cut)

    # Prepare cuts
    w, h = mfg.unary_union(layers_cut).bounding_box().get_dimensions()

    layers_cut_final = layers_cut[0]
    for i in range(1,num_layers):
        step = 10
        d = int(np.ceil(h/step)*i+i)
        layers_cut_final |= safe_translate_layer(layers_cut[i],0,d*step)

    if plot:
        layers_cut_final.plot(new=True)
        plt.figure()
        for l in release_cut_layers: l.plot()
        release_cut.plot()
        plt.show(block=True)

    folder_name = os.path.basename(os.path.normpath(path))
    layers_cut_final.export_dxf(os.path.join(path,'{}_layers'.format(folder_name)))

    # Different color for all-the-way cuts and signle-layer cuts
    doc = ezdxf.new('R2010')
    msp = doc.modelspace()
    c = 0
    for line in release_cut.get_paths():
        msp.add_lwpolyline(line,dxfattribs={'color': c})
    c += 1
    for l in release_cut_layers:
        for line in l.get_paths():
            msp.add_lwpolyline(line,dxfattribs={'color': c})
        c += 1
        if c > 6: print('Running out of colors for single-layer cut')
    doc.saveas(os.path.join(path,'{}_release.dxf'.format(folder_name)))
