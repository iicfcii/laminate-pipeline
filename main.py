import data
import plan

comps_poly,comps_circle,joints,layers_comp = data.read('./data/fourbar_comp')
device, joints_cut, bodies_cut = plan.device(comps_poly,comps_circle,joints,layers_comp)
layers_cut, release_cut = plan.cuts(device)
layers_cut.export_dxf('./data/fourbar_comp/leg_layers')
release_cut.export_dxf('./data/fourbar_comp/leg_release')
