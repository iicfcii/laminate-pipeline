import numpy as np
import shapely.geometry as sg
import foldable_robotics.dxf as dxf
from foldable_robotics.layer import Layer
from foldable_robotics.laminate import Laminate
import matplotlib.pyplot as plt
import plan

W_DEFAULT = 1

def bbox(line, w):
    dl = -plan.CUT_THICKNESS # length offset to mask joint for cleaner cuts
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

    t = plan.CUT_THICKNESS
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
    'plain1': plain1,
    'dashed1': dashed1
}
