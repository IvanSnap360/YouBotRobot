import os
import cv2
import numpy as np
import string
from cube_orig_model_template import cube_orig_template
import sys 
import shutil
package_path = os.path.dirname(os.path.realpath(__file__))[0:-3]
saving_folder_path = package_path + "/aruco_cubes/"

def generate_cube_model_and_return_udf(id):
    #####
    new_cube_folder = saving_folder_path + f"/cube_{id}"
    try:
        os.mkdir(new_cube_folder)
    except:
        pass
    #####
    tag_image = np.zeros((800, 800, 1), dtype=np.uint8)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)
    tag_image = cv2.aruco.generateImageMarker(arucoDict, id, 800, tag_image, 1)
    cv2.imwrite(new_cube_folder+f"/marker_{id}.png", tag_image)
    shutil.copy(f"{package_path}/src/white.jpg",f"{new_cube_folder}/")
    #####
    dae_file = open(new_cube_folder + f"/cube{id}.dae", "w")
    data_to_write = cube_orig_template.substitute(marker_243_png=f"marker_{id}_png",texture_file=f"marker_{id}.png")
    dae_file.write(data_to_write)
    dae_file.close()
    #####

    cube_urdf_template = string.Template(
        """
    <?xml version="1.0"?>
    <robot name="cube">
        <link name="cube_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
                <geometry>
                    <mesh filename="$package_path/aruco_cubes/cube_$id/$cube_model_file_name.dae" scale="0.5 0.5 0.5"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
                <geometry>
                    <mesh filename="$package_path/aruco_cubes/cube_$id/$cube_model_file_name.dae" scale="0.5 0.5 0.5" />
                </geometry>
            </collision>
            <inertial>
                <mass value="0.040" />
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
                <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0" />
            </inertial>
        </link>
    </robot>"""
    )

    urdf_str = cube_urdf_template.substitute(package_path = package_path[:-1],cube_model_file_name=f"cube{id}",id=id)
    return urdf_str