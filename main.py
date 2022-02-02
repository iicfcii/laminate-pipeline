import data
import plan
import matplotlib.pyplot as plt
import os, sys

if __name__ == '__main__':
    path = sys.argv[1]
    plot = sys.argv[2] == '-p' if len(sys.argv) > 2 else False

    polys, circles, joints, layers = data.read(path)

    device, joints_cut, bodies_cut = plan.device(polys, circles, joints, layers)

    # Use clearance to remove thin web and separate web from device
    supported_device, release_cut_scrap, single_layer_cut = plan.cuts(device)

    plan.export(path,supported_device,release_cut_scrap,single_layer_cut,plot=plot)
