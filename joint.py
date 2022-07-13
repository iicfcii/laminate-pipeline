import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
import foldable_robotics.dxf as dxf
from foldable_robotics.layer import Layer
from foldable_robotics.laminate import Laminate
import matplotlib.pyplot as plt

W_DEFAULT = 0.4
CUT_THICKNESS = 0.01

def bbox(line, w):
    dl = -CUT_THICKNESS # length offset to mask joint for cleaner cuts
    line = np.array(line).reshape((2,2))

    center = np.average(line,axis=0)
    dir = line[1,:]-line[0,:]
    l = np.linalg.norm(dir)
    dir = dir/l
    bbox = sg.LineString([center-dir*(l/2-w/2+dl/2),center+dir*(l/2-w/2+dl/2)])
    bbox = bbox.buffer(w/2,cap_style=sg.CAP_STYLE.square)

    return Layer(bbox)

# Material to be cut to form the joint
def inv(jf, line, w=W_DEFAULT):
    joint_block = jf(line,w=w)
    sheet = bbox(line,w)
    sheet = sheet.to_laminate(len(joint_block))

    return sheet-joint_block

# Plain 5 layer joint
def plain5(line, w=W_DEFAULT):
    flex_layer = bbox(line,w)
    other_layer = Layer()

    return Laminate(other_layer,other_layer,flex_layer,other_layer,other_layer)

def stamp5(line, w=W_DEFAULT):
    l3 = bbox(line,w)

    line = np.array(line).reshape((2,2))

    dir = line[1,:]-line[0,:]
    l = np.linalg.norm(dir)
    dir = dir/l
    ang = np.arctan2(dir[1],dir[0])+np.pi/2

    r = 0.2 # flexure joint width
    d = 0.2 # clearance between tooth
    lt = (l-d*2)/3 # tooth length
    wt = (w-r)/2 # tooth width

    dx = (wt/2+r/2)*np.cos(ang)
    dy = (wt/2+r/2)*np.sin(ang)

    def teeth(side):
        line_left = line+side*np.tile([dx,dy],(2,1))
        line_right = line-side*np.tile([dx,dy],(2,1))

        t1 = [
            line_left[0,:]+dir*wt/2,
            line_left[0,:]+dir*(wt/2+lt-wt)
        ]
        t2 = [
            line_right[0,:]+dir*(lt+d+wt/2),
            line_right[0,:]+dir*(lt+d+wt/2+lt-wt)
        ]
        t3 = [
            line_left[1,:]-dir*wt/2,
            line_left[1,:]-dir*(wt/2+lt-wt)
        ]
        ts = sg.Polygon()
        ts |= sg.LineString(t1)
        ts |= sg.LineString(t2)
        ts |= sg.LineString(t3)

        ts = ts.buffer(wt/2,cap_style=sg.CAP_STYLE.square)
        ts = Layer(ts)

        return ts

     # actual joint length is a little smaller due to cut thickness
    l1 = teeth(1)&l3
    l5 = teeth(-1)&l3

    j = Laminate(l1,l1,l3,l5,l5)
    # j.plot_layers()
    # plt.show(block=True)

    return j

# Plain 1 layer joint
# This basically makes sure joint at the layer is connected
def plain1(line, w=W_DEFAULT):
    flex_layer = bbox(line,w)
    return Laminate(flex_layer)

# Dashed 1 layer joint
# Creates dashed cuts along the joint for 1 layer design
# style: (cut lengths, not cut length)
# pad: pad on both ends to prevent tearing, actual length will be bigger to make cuts symmetrical abouot the center
def dashed1(line, w=W_DEFAULT, style=(1,1), pad=2):
    flex_layer = bbox(line,w)

    line = np.array(line).reshape((2,2))
    center = np.average(line,axis=0)
    dir = line[1,:]-line[0,:]
    l = np.linalg.norm(dir)
    dir = dir/l

    ls = style[0]+style[1]
    num_pair = int((l-pad*2)/ls)-0.5
    pad_actual = (l-num_pair*ls)/2

    t = CUT_THICKNESS
    cuts = sg.Polygon()
    p1 = center-dir*(l/2-pad_actual-t/2)

    for i in range(int(num_pair)):
        cuts |= sg.LineString([p1,p1+dir*(style[0]-t)])
        p1 = p1+dir*ls
    cuts |= sg.LineString([p1,p1+dir*(style[0]-t)])

    cuts = cuts.buffer(t/2,cap_style=sg.CAP_STYLE.square)
    cuts = Layer(cuts)

    return Laminate(flex_layer-cuts)

DICTS = {
    'plain5': plain5,
    'stamp5': stamp5,
    'plain1': plain1,
    'dashed1': dashed1
}
