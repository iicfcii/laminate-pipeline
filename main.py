import data
import plan
import matplotlib.pyplot as plt
import os, sys

if __name__ == '__main__':
    path = sys.argv[1]
    folder_name = os.path.basename(os.path.normpath(path))

    device, joints_cut, bodies_cut = plan.device(
        *data.read(path)
    )
     # Use clearance to remove thin web and separate web from device
    layers_cut, release_cut = plan.cuts(device)

    # layers_cut.plot(new=True)
    # release_cut.plot(new=True)
    # plt.show(block=True)

    layers_cut.export_dxf(os.path.join(path,'{}_layers'.format(folder_name)))
    release_cut.export_dxf(os.path.join(path,'{}_release'.format(folder_name)))

    print('{} cuts generated'.format(folder_name))
