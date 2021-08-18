import numpy as np
import shapely.geometry as sg
import foldable_robotics.dxf as dxf
from foldable_robotics.layer import Layer
from foldable_robotics.laminate import Laminate

# Simple 5 layer joint
# dl: length offset for cleaner cuts in some situations
def plain5(line, w=1.5, invert=False, dl=0):
    line = np.array(line).reshape((2,2))

    center = np.average(line,axis=0)
    dir = line[1,:]-line[0,:]
    l = np.linalg.norm(dir)
    dir = dir/l
    flex_layer = sg.LineString([center-dir*(l/2-w/2+dl/2),center+dir*(l/2-w/2+dl/2)])
    flex_layer = flex_layer.buffer(w/2,cap_style=sg.CAP_STYLE.square)

    flex_layer = Layer(flex_layer)
    other_layer = Layer()

    if not invert:
        return Laminate(other_layer,other_layer,flex_layer,other_layer,other_layer)
    else:
        return Laminate(flex_layer-other_layer,flex_layer-other_layer,flex_layer-flex_layer,flex_layer-other_layer,flex_layer-other_layer)
