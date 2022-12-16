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
    dl = -CUT_THICKNESS  # length offset to mask joint for cleaner cuts
    line = np.array(line).reshape((2, 2))

    center = np.average(line, axis=0)
    dir = line[1, :] - line[0, :]
    l = np.linalg.norm(dir)
    dir = dir / l
    bbox = sg.LineString([center - dir * (l / 2 - w / 2 + dl / 2),
                         center + dir * (l / 2 - w / 2 + dl / 2)])
    bbox = bbox.buffer(w / 2, cap_style=sg.CAP_STYLE.square)

    return Layer(bbox)


def plain5(line, w=W_DEFAULT):
    # Plain 5 layer joint

    flex_layer = bbox(line, w)
    other_layer = Layer()

    joint_laminate = Laminate(
        other_layer,
        other_layer,
        flex_layer,
        other_layer,
        other_layer)
    joint_laminate_inv = flex_layer.to_laminate(5) - joint_laminate
    return joint_laminate, joint_laminate_inv


def stamp5(line, w=W_DEFAULT):
    l3 = bbox(line, w)

    line = np.array(line).reshape((2, 2))

    dir = line[1, :] - line[0, :]
    l = np.linalg.norm(dir)
    dir = dir / l
    ang = np.arctan2(dir[1], dir[0]) + np.pi / 2

    r = 0.2  # flexure joint width
    d = 0.2  # clearance between tooth
    lt = (l - d * 2) / 3  # tooth length
    wt = (w - r) / 2  # tooth width

    dx = (wt / 2 + r / 2) * np.cos(ang)
    dy = (wt / 2 + r / 2) * np.sin(ang)

    def teeth(side):
        line_left = line + side * np.tile([dx, dy], (2, 1))
        line_right = line - side * np.tile([dx, dy], (2, 1))

        t1 = [
            line_left[0, :] + dir * wt / 2,
            line_left[0, :] + dir * (wt / 2 + lt - wt)
        ]
        t2 = [
            line_right[0, :] + dir * (lt + d + wt / 2),
            line_right[0, :] + dir * (lt + d + wt / 2 + lt - wt)
        ]
        t3 = [
            line_left[1, :] - dir * wt / 2,
            line_left[1, :] - dir * (wt / 2 + lt - wt)
        ]
        ts = sg.Polygon()
        ts |= sg.LineString(t1)
        ts |= sg.LineString(t2)
        ts |= sg.LineString(t3)

        ts = ts.buffer(wt / 2, cap_style=sg.CAP_STYLE.square)
        ts = Layer(ts)

        return ts

    # actual joint length is a little smaller due to cut thickness
    l1 = teeth(1) & l3
    l5 = teeth(-1) & l3

    joint_laminate = Laminate(l1, l1, l3, l5, l5)
    joint_laminate_inv = l3.to_laminate(5) - joint_laminate
    return joint_laminate, joint_laminate_inv


def bend5(line, w=6 + 0.5 * 2 + 0.4, ds=0.5, t=0.4, j=W_DEFAULT, pad=2.5):
    s = w - ds * 2 - t
    line = np.array(line).reshape((2, 2))
    dir = line[1, :] - line[0, :]
    l = np.linalg.norm(dir)
    dir = dir / l
    ang = np.arctan2(dir[1], dir[0])
    center = (line[1, :] + line[0, :]) / 2

    # Bounding polygon
    bpg = bbox(line, j)
    for c in [center, center + (l / 2 - pad) * dir,
              center - (l / 2 - pad) * dir]:
        for rot in [ang + np.pi / 2, ang - np.pi / 2]:
            p1 = c
            p2 = c + (s / 2 + ds) * np.array([np.cos(rot), np.sin(rot)])
            rib = Layer(sg.LineString([p1, p2]).buffer(
                t / 2, cap_style=sg.CAP_STYLE.square))
            bpg |= rib

    # Flex layer
    flex_layer = bbox(line, j)
    for c in [center, center + (l / 2 - pad) * dir,
              center - (l / 2 - pad) * dir]:
        for rot in [ang + np.pi / 2, ang - np.pi / 2]:
            p1 = c
            p2 = c + (s / 2 - t) * np.array([np.cos(rot), np.sin(rot)])
            hole = Layer(sg.LineString([p1, p2]).buffer(
                t / 2, cap_style=sg.CAP_STYLE.square))
            flex_layer |= hole

    # Hinge
    hinge = bbox(line, j)
    other_layer = flex_layer
    other_layer ^= hinge

    joint_laminate = Laminate(
        other_layer,
        other_layer,
        flex_layer,
        other_layer,
        other_layer)
    joint_laminate_inv = bpg.to_laminate(5) - joint_laminate

    return joint_laminate, joint_laminate_inv


def plain1(line, w=W_DEFAULT):
    # Plain 1 layer joint
    # This basically makes sure joint at the layer is connected

    flex_layer = bbox(line, w)

    joint_laminate = Laminate(flex_layer)
    joint_laminate_inv = flex_layer.to_laminate(1) - joint_laminate
    return joint_laminate, joint_laminate_inv


def dashed1(line, w=W_DEFAULT, style=(1, 1), pad=2):
    # Dashed 1 layer joint
    # Creates dashed cuts along the joint for 1 layer design
    # style: (cut lengths, not cut length)
    # pad: pad on both ends to prevent tearing, actual length will be bigger
    # to make cuts symmetrical abouot the center

    flex_layer = bbox(line, w)

    line = np.array(line).reshape((2, 2))
    center = np.average(line, axis=0)
    dir = line[1, :] - line[0, :]
    l = np.linalg.norm(dir)
    dir = dir / l

    ls = style[0] + style[1]
    num_pair = int((l - pad * 2) / ls) - 0.5
    pad_actual = (l - num_pair * ls) / 2

    t = CUT_THICKNESS
    cuts = sg.Polygon()
    p1 = center - dir * (l / 2 - pad_actual - t / 2)

    for i in range(int(num_pair)):
        cuts |= sg.LineString([p1, p1 + dir * (style[0] - t)])
        p1 = p1 + dir * ls
    cuts |= sg.LineString([p1, p1 + dir * (style[0] - t)])

    cuts = cuts.buffer(t / 2, cap_style=sg.CAP_STYLE.square)
    cuts = Layer(cuts)

    joint_laminate = Laminate(flex_layer - cuts)
    joint_laminate_inv = flex_layer.to_laminate(1) - joint_laminate
    return joint_laminate, joint_laminate_inv


DICTS = {
    'plain5': plain5,
    'stamp5': stamp5,
    'bend5': bend5,
    'plain1': plain1,
    'dashed1': dashed1
}
