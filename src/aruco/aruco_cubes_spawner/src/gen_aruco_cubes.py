#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel, GetModelState, DeleteModel, GetModelStateResponse
from geometry_msgs.msg import Pose
import rospy
import os
from gen_aruco_models import generate_cube_model_and_return_udf
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
package_path = os.path.dirname(os.path.realpath(__file__))[0:-3]

yaml_config = yaml.load(open(
    f"{package_path}/config/spawn_config.yaml", "r").read(), Loader=yaml.FullLoader)

cubes_num = len(yaml_config['cubes'])
# print(cubes_num)
# exit()
rospy.init_node("aruco_cubes_node")


rospy.wait_for_service('gazebo/spawn_urdf_model')

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
model_coordinates = rospy.ServiceProxy(
    '/gazebo/get_model_state', GetModelState)


ids = [i for i in range(0, cubes_num)]

i = 0
while (1):
    try:
        id = ids[i]
        model_state = GetModelStateResponse()

        model_state = model_coordinates(f"cube_{id}", "")

        if model_state.success == True:
            rospy.logwarn(f"Deleteting cube_{id}...")
            del_model_prox(f"cube_{id}")
            rospy.logwarn(f"Deleteting cube_{id}...DONE")
        else:
            break
        i += 1
    except:
        break


pose = Pose()

for id in ids:
    pose.position.x =    yaml_config['cubes'][id]['cube_pos'][0]
    pose.position.y =    yaml_config['cubes'][id]['cube_pos'][1]
    pose.position.z =    yaml_config['cubes'][id]['cube_pos'][2] + 0.025/2
    pose.orientation.x = yaml_config['cubes'][id]['cube_ori'][0]
    pose.orientation.y = yaml_config['cubes'][id]['cube_ori'][1]
    pose.orientation.z = yaml_config['cubes'][id]['cube_ori'][2]
    pose.orientation.w = 1
    rospy.loginfo(f"Spawning cube_{id}...")
    spawn_model_client(
        model_name=f'cube_{id}',
        model_xml=generate_cube_model_and_return_udf(id),
        robot_namespace='cubes',
        initial_pose=pose,
        reference_frame='world'
    )
    rospy.loginfo(f"Spawning cube_{id}...DONE")

