import data
import plan
import matplotlib.pyplot as plt

name = 'leg_253'

comps_poly,comps_circle,joints,layers_comp = data.read('./data/{}'.format(name))
device, joints_cut, bodies_cut = plan.device(comps_poly,comps_circle,joints,layers_comp)
 # Use clearance to remove thin web and separate web from device
layers_cut, release_cut = plan.cuts(device)

layers_cut.plot(new=True)
release_cut.plot(new=True)
plt.show(block=True)

layers_cut.export_dxf('./data/{}/{}_layers'.format(name,name))
release_cut.export_dxf('./data/{}/{}_release'.format(name,name))
