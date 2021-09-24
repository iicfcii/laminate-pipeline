import data
import plan
import joint
import matplotlib.pyplot as plt

name = 'rib_test'

device, joints_cut, bodies_cut = plan.device(
    *data.read('./data/{}'.format(name)),
    joint_fun=joint.plain1
)
 # Use clearance to remove thin web and separate web from device
layers_cut, release_cut = plan.cuts(device)

layers_cut.plot(new=True)
release_cut.plot(new=True)
plt.show(block=True)

layers_cut.export_dxf('./data/{}/{}_layers'.format(name,name))
release_cut.export_dxf('./data/{}/{}_release'.format(name,name))
