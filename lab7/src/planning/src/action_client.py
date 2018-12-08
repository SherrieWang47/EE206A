#!/usr/bin/env python
import roslib; roslib.load_manifest('planning')

import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult, JointConstraint, Constraints

def main():
    #Initialize the node
    rospy.init_node('moveit_client')
    
    # Create the SimpleActionClient, passing the type of the action
    # (MoveGroupAction) to the constructor.
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)

    # Wait until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveGroupGoal()
    
    #----------------Construct the goal message (start)----------------
    joint_names = ['head_pan', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    
    #Set parameters for the planner    
    goal.request.group_name = 'both_arms'
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = 5.0
    
    #Define the workspace in which the planner will search for solutions
    goal.request.workspace_parameters.min_corner.x = -1
    goal.request.workspace_parameters.min_corner.y = -1
    goal.request.workspace_parameters.min_corner.z = -1
    goal.request.workspace_parameters.max_corner.x = 1
    goal.request.workspace_parameters.max_corner.y = 1
    goal.request.workspace_parameters.max_corner.z = 1
    
    goal.request.start_state.joint_state.header.frame_id = "base"
    
    #Set the start state for the trajectory
    goal.request.start_state.joint_state.name = joint_names
    #goal.request.start_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    head_pan = float(raw_input("Enter start joint1 position: "))
    left_s0 = float(raw_input("Enter start joint2 position: "))
    left_s1 = float(raw_input("Enter start joint3 position: "))
    left_e0 = float(raw_input("Enter start joint4 position: "))
    left_e1 = float(raw_input("Enter start joint5 position: "))
    left_w0 = float(raw_input("Enter start joint6 position: "))
    left_w1 = float(raw_input("Enter start joint7 position: "))
    left_w2 = float(raw_input("Enter start joint8 position: "))
    right_s0 = float(raw_input("Enter start joint9 position: "))
    right_s1 = float(raw_input("Enter start joint10 position: "))
    right_e0 = float(raw_input("Enter start joint11 position: "))
    right_e1 = float(raw_input("Enter start joint12 position: "))
    right_w0 = float(raw_input("Enter start joint13 position: "))
    right_w1 = float(raw_input("Enter start joint14 position: "))
    right_w2 = float(raw_input("Enter start joint15 position: "))

    goal.request.start_state.joint_state.position = [head_pan, left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2, right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2]
    goal.request.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    #Tell MoveIt whether to execute the trajectory after planning it
    goal.planning_options.plan_only = True
    
    #Set the goal position of the robot
    #Note that the goal is specified with a collection of individual
    #joint constraints, rather than a vector of joint angles
    arm_joint_names = joint_names[1:]
    ghead_pan = float(raw_input("Enter goal joint1 position: "))
    gleft_s0 = float(raw_input("Enter goal joint2 position: "))
    gleft_s1 = float(raw_input("Enter goal joint3 position: "))
    gleft_e0 = float(raw_input("Enter goal joint4 position: "))
    gleft_e1 = float(raw_input("Enter goal joint5 position: "))
    gleft_w0 = float(raw_input("Enter goal joint6 position: "))
    gleft_w1 = float(raw_input("Enter goal joint7 position: "))
    gleft_w2 = float(raw_input("Enter goal joint8 position: "))
    gright_s0 = float(raw_input("Enter goal joint9 position: "))
    gright_s1 = float(raw_input("Enter goal joint10 position: "))
    gright_e0 = float(raw_input("Enter goal joint11 position: "))
    gright_e1 = float(raw_input("Enter goal joint12 position: "))
    gright_w0 = float(raw_input("Enter goal joint13 position: "))
    gright_w1 = float(raw_input("Enter goal joint14 position: "))
    gright_w2 = float(raw_input("Enter goal joint15 position: "))

    target_joint_angles = [ghead_pan, gleft_s0, gleft_s1, gleft_e0, gleft_e1, gleft_w0, gleft_w1, gleft_w2, gright_s0, gright_s1, gright_e0, gright_e1, gright_w0, gright_w1, gright_w2]
    #target_joint_angles = [0.5, -0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tolerance = 0.0001
    consts = []
    for i in range(len(arm_joint_names)):
        const = JointConstraint()
        const.joint_name = arm_joint_names[i]
        const.position = target_joint_angles[i]
        const.tolerance_above = tolerance
        const.tolerance_below = tolerance
        const.weight = 1.0
        consts.append(const)
        
    goal.request.goal_constraints.append(Constraints(name='', joint_constraints=consts))
    #---------------Construct the goal message (end)-----------------

    # Send the goal to the action server.
    client.send_goal(goal)

    # Wait for the server to finish performing the action.
    client.wait_for_result()

    # Print out the result of executing the action
    print(client.get_result())
    

if __name__ == '__main__':
    main()

