import os, csv
import numpy as np
import foldable_robotics.dxf as dxf
import shapely.geometry as sg

def read(path):
    # Read layers and components realtionship
    layers_comp = {} # components within each layer
    comps_layer = {} # layers of each component
    zs = {} # thickness of each layer
    with open(os.path.join(path, 'layers.csv'), newline='') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i == 0: continue
            l = int(row[0])
            comp = row[1]
            z_start = float(row[2])

            if l not in layers_comp:
                layers_comp[l] = [comp]
            else:
                layers_comp[l].append(comp)

            if comp not in comps_layer:
                comps_layer[comp] = [l]
            else:
                comps_layer[comp].append(l)

            if l not in zs:
                zs[l] = z_start

    # Read joints data
    joints = []
    with open(os.path.join(path, 'rev_joints.csv'), newline='') as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i == 0: continue

            compA = row[0]
            compB = row[1]
            pt = [float(val) for val in row[2:5]]
            dir = [float(val) for val in row[5:8]]

            j = {}
            j['compA'] = compA
            j['compB'] = compB
            j['pt'] = pt
            j['dir'] = dir

            joints.append(j)

    # Read polys within each layer of each component
    comps_poly = {}
    comps_circle = {}
    for comp in comps_layer:
        comps_poly[comp] = {}
        comps_circle[comp] = {}
        for l in comps_layer[comp]:
            polys = dxf.read_lwpolylines(os.path.join(path, '{:d}_{}.dxf'.format(l,comp)))
            circles = dxf.read_circles(os.path.join(path, '{:d}_{}.dxf'.format(l,comp)))
            comps_poly[comp][l] = polys
            comps_circle[comp][l] = circles

    # Construct joints
    for j in joints:
        # Determine joint center layer
        for l in range(len(zs.keys())-1):
            if j['pt'][2] < zs[l+1] and j['pt'][2] > zs[l]:
                break

        # Determine line
        d = 0.01
        compA = sg.Polygon()
        for p in comps_poly[j['compA']][l]:
            compA |= sg.Polygon(p)
        compB = sg.Polygon()
        for p in comps_poly[j['compB']][l]:
            compB |= sg.Polygon(p)

        # TODO: Can use a line that extends over the bounding box
        length = 10000 # Long enough
        joint_ptA = np.array(j['pt'][0:2])-length*np.array(j['dir'][0:2])
        joint_ptB = np.array(j['pt'][0:2])+length*np.array(j['dir'][0:2])
        joint = sg.LineString([joint_ptA,joint_ptB])

        joint = compA.buffer(d) & compB.buffer(d) & joint # intersect the joint line with two bodies

        lines = [] # Collect joint lines
        if joint.geom_type == 'LineString':
            lines.append(list(joint.coords))
        else:
            assert joint.geom_type == 'MultiLineString'
            for g in joint.geoms:
                lines.append(list(g.coords))

        lines_shrunk = []
        for line in lines: # Shrink line to correct distance
            ptA = np.array(line[0])
            ptB = np.array(line[1])

            ptA = ptA+(ptB-ptA)*d/np.linalg.norm(ptB-ptA)
            ptB = ptB+(ptA-ptB)*d/np.linalg.norm(ptA-ptB)
            lines_shrunk.append([tuple(ptA), tuple(ptB)])

        j['lines'] = lines_shrunk
        j['layer'] = l
        # print(j)

    return comps_poly,comps_circle,joints,layers_comp
