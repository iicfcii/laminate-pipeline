#Author-Fuchen Chen
#Description-

import os, csv
import adsk.core, adsk.fusion, adsk.cam, traceback

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct
        root_comp = design.rootComponent
        sketches = root_comp.sketches
        planes = root_comp.constructionPlanes

        z_offset = 0 # Z offset for the first layer
        t_layers = [0.381,0.015,0.127,0.015,0.381]

        input, isCancelled = ui.inputBox(
            'Signed distance between\nthe bottom of your design\nto the xy plane in mm',
            'Z offset',
            str(z_offset)
        )
        if isCancelled: return
        z_offset = float(input)

        input, isCancelled = ui.inputBox(
            'Layer thicknesses in mm separated by ","\nFor a single layer design, just enter one number.',
            'Layer thicknesses',
            ','.join([str(t) for t in t_layers])
        )
        if isCancelled: return
        t_layers = [float(i) for i in input.split(',')]

        # Save folder
        path = ''
        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = 'Choose save folder'

        dlg_result = folder_dlg.showDialog()
        if dlg_result == adsk.core.DialogResults.DialogOK:
            path = folder_dlg.folder
        else:
            return # Quit script

        # Save bodies of each layer
        layers = {}
        z = z_offset # Start of the first layer
        for i, t in enumerate(t_layers):
            z += t /2 # center of the layer in z direction
            layers[i] = []

            for occ in root_comp.allOccurrences:
                plane_input = planes.createInput()
                plane_input.setByOffset(root_comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z/10))
                plane = planes.add(plane_input)
                sketch = sketches.add(plane)
                sketch.name = occ.component.name
                # ui.messageBox('{} {} {}'.format(*sketch.yDirection.asArray()))

                onLayer = False
                for body in occ.component.bRepBodies:
                    if z/10 < body.boundingBox.maxPoint.z and z/10 > body.boundingBox.minPoint.z: # system unit is cm
                        sketch.intersectWithSketchPlane([body.createForAssemblyContext(occ)])
                        onLayer = True

                if onLayer:
                    layers[i].append(occ.component.name)
                    sketch.saveAsDXF(os.path.join(path,'{:d}_{}.dxf'.format(i, occ.component.name)))

                sketch.deleteMe()
                plane.deleteMe()
                # return
            z += t/2 # Move to start of next layer

        with open(os.path.join(path,'layers.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['layer','component','z_start'])
            for l in layers.keys():
                for comp in layers[l]:
                    writer.writerow([l,comp,sum(t_layers[0:l])+z_offset]) # Save layer z start info

        all_joints = [*root_comp.allAsBuiltJoints,*root_comp.allJoints]
        rev_joints = []
        error_joints = []
        for joint in all_joints[:]:
            if joint.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType:
                try:
                    rev_joints.append([
                        joint.name,
                        joint.occurrenceOne.component.name,joint.occurrenceTwo.component.name,
                        *[val*10 for val in joint.geometry.origin.asArray()], # convert to mm
                        *joint.jointMotion.rotationAxisVector.asArray() # unit vector
                    ])
                except:
                    error_joints.append(joint.name)
                    continue

        if len(error_joints) > 0:
            ui.messageBox('Following joints have error: {}'.format(', '.join(error_joints)))

        with open(os.path.join(path,'rev_joints.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['name','componentA','componentB','x','y','z','dx','dy','dz'])
            for j in rev_joints:
                writer.writerow(j)

        ui.messageBox('Wrote {:d} layers of components and joints'.format(len(t_layers)))

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
