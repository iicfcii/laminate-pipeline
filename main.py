import data
import plan
import matplotlib.pyplot as plt
import os, sys

if __name__ == '__main__':
    path = sys.argv[1]
    plot = sys.argv[2] == '-p' if len(sys.argv) > 2 else False
    twin = sys.argv[3] == '-t' if len(sys.argv) > 3 else False

    polys, circles, joints, layers = data.read(path)

    device, joints_cut, bodies_cut = plan.device(polys, circles, joints, layers)

    if twin: device = plan.twin(device)

    # Use clearance to remove thin web and separate web from device
    layers_cut, release_cut, release_cut_layers = plan.cuts(device)

    plan.export(path, layers_cut, release_cut, release_cut_layers, plot=plot)
