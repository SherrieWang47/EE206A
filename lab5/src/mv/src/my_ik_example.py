#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper
import numpy as np
from numpy import linalg

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # rospy.init_node('gripper_test')

    #Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    init_pos = [0.581,-0.423,-0.382]
    final_pos = [0.603,-0.164,-0.367]

    # #Close the right gripper
    # print('Closing...')
    # right_gripper.close()
    # rospy.sleep(1.0)

    # #Open the right gripper
    # print('Opening...')
    # right_gripper.open()
    # rospy.sleep(1.0)
    # print('Done!')
    
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = init_pos[0]
        request.ik_request.pose_stamped.pose.position.y = init_pos[1]
        request.ik_request.pose_stamped.pose.position.z = init_pos[2]       
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()

            rospy.sleep(1.0)


            # #Open the right gripper
            print('Opening...')
            right_gripper.open()
            rospy.sleep(1.0)
            print('Done!')

            # #Close the right gripper
            print('Closing...')
            right_gripper.close()
            rospy.sleep(1.0)

            #Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = final_pos[0]
            request.ik_request.pose_stamped.pose.position.y = final_pos[1]
            request.ik_request.pose_stamped.pose.position.z = final_pos[2]       
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0

            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()

            rospy.sleep(1.0)


            # #Open the right gripper
            print('Opening...')
            right_gripper.open()
            rospy.sleep(1.0)
            print('Done!')
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

