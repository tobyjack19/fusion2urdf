#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import re
import sys
from .utils import utils
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg
    
    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 
        components = design.allComponents

        # set the names        
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'

        # Ask user for target folder (base directory)
        base_dir = utils.file_dialog(ui)
        if base_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        # If a package with the same name already exists in the selected folder,
        # append a version suffix _vN where N is 1 higher than the highest
        # existing version number found. Existing names handled are:
        #   package_name
        #   package_name_v1, package_name_v2, ...
        try:
            existing_versions = []
            for name in os.listdir(base_dir):
                if name == package_name:
                    existing_versions.append(0)
                else:
                    m = re.match(re.escape(package_name) + r'_v(\d+)$', name)
                    if m:
                        try:
                            existing_versions.append(int(m.group(1)))
                        except ValueError:
                            pass
            if existing_versions:
                new_ver = max(existing_versions) + 1
                package_name = f"{package_name}_v{new_ver}"
        except Exception:
            # If anything goes wrong (permissions, etc.), fall back to original name
            pass

        # Final save directory is the selected folder + package_name
        save_dir = os.path.join(base_dir, package_name)
        try:
            os.mkdir(save_dir)
        except:
            pass    

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'
        
        # --------------------
        # set dictionaries
        
        # Generate joints_dict. All joints are related to root. 
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0   
        
        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0
        
        links_xyz_dict = {}
        
        # --------------------
        # Generate URDF
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir)
        Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
        Write.write_yaml(package_name, robot_name, save_dir, joints_dict)
        
        # copy over package files
        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # Generate STl files
        # copy_occs returns metadata about temporary components it created so we
        # can clean them up afterward and restore original names.
        copied_info = utils.copy_occs(root)
        utils.export_stl(design, save_dir, components)
        # delete temporary copied components and restore original names
        try:
            utils.delete_copied_components(root, copied_info)
        except Exception:
            # best-effort cleanup; ignore errors here to avoid blocking the user
            pass
        
        ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
