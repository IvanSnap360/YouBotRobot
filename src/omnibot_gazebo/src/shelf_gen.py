#!/usr/bin/env python3
import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState,GetModelStateResponse,SpawnModelRequest
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty


def main(shelf_count):
    rospy.init_node('shelf_gen_node')

    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    model_coordinates = rospy.ServiceProxy(
        '/gazebo/get_model_state', GetModelState)
    
    model_dir = os.path.dirname(os.path.realpath(__file__))[0:-3] + "models/model.sdf"
    model = open(model_dir,'r').read()
    pose = Pose()


    i = 0
    while(1):
        model_state = GetModelStateResponse()
        
        model_state = model_coordinates(f"bookshelf_{i+1}_1","")
        
        if model_state.success == True:
            rospy.logwarn(f"Deleteting bookshelf_{i+1}_1")
            rospy.logwarn(f"Deleteting bookshelf_{i+1}_2")
            del_model_prox(f"bookshelf_{i+1}_1")
            del_model_prox(f"bookshelf_{i+1}_2")
        else:
            break
        i += 1

    
    
    pose = Pose()
    for j in range(3):
        pose.position.x = -6.0 
        for i in range(shelf_count):
            pose.position.y = j
            pose.orientation.z = 0.0
            spawn_model_prox(f"bookshelf_{i+1}_1", model, "bookshelfs", pose, "world")
            pose.position.y = j + 0.1
            pose.orientation.z = 3.14
            spawn_model_prox(f"bookshelf_{i+1}_2", model, "bookshelfs", pose, "world")
            pose.position.x += 1.0

    print()
    # rospy.spin()


if __name__ == '__main__':
    main(16)
