#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
import numpy as np
import kin_func_skeleton as kfs
from sensor_msgs.msg import JointState
import math

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    #Print the contents of the message to the console
    #print(rospy.get_name() + ": I heard %s" % message.data)
    joint_angle = [message.position[4],message.position[5],message.position[2],message.position[3],message.position[6],message.position[7],message.position[8]]
    print("Left Arm Position:\n")
    print(joint_angle)

    q = np.ndarray((3,8))
    w = np.ndarray((3,7))

    q[0:3,0] = [0.0635, 0.2598, 0.1188]
    q[0:3,1] = [0.1106, 0.3116, 0.3885]
    q[0:3,2] = [0.1827, 0.3838, 0.3881]
    q[0:3,3] = [0.3682, 0.5684, 0.3181]
    q[0:3,4] = [0.4417, 0.6420, 0.3177]
    q[0:3,5] = [0.6332, 0.8337, 0.3067]
    q[0:3,6] = [0.7152, 0.9158, 0.3063]
    q[0:3,7] = [0.7957, 0.9965, 0.3058]

    w[0:3,0] = [-0.0059,  0.0113,  0.9999]
    w[0:3,1] = [-0.7077,  0.7065, -0.0122]
    w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,3] = [-0.7077,  0.7065, -0.0122]
    w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,5] = [-0.7077,  0.7065, -0.0122]
    w[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                          [-0.7040, 0.7102, -0.0053],
                          [0.7102, 0.7040, 0.0055]]).T

    # YOUR CODE HERE
    #base = np.array([0,0,0])
    
    #g_st(0)
    g_st0 = np.ndarray((4,4))
    g_st0[0:3,0:3] = R
    g_st0[3,0:3] = np.array([0,0,0])
    g_st0[0:3,3] = q[0:3,7]
    g_st0[3,3] = 1

    #twists
    xi = np.ndarray((6,7))
    for i in range(0,7):
        xi[0:3,i] = np.cross(-w[0:3,i],q[0:3,i])
        xi[3:6,i] = w[0:3,i]
    xi_hat = kfs.prod_exp(xi,joint_angle)

    g = np.matmul(xi_hat,g_st0)

    print("Transformation matrix:\n")
    print(g)




#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('forward_kinematics', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("/robot/joint_states", JointState, callback, queue_size=8)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


def lab5_test():
    # position = np.array((1,8))
    # position[2] = 0.0
    # position[3] = 0.0
    # position[4] = 0.0
    # position[5] = 0.0
    # position[6] = 0.0
    # position[7] = 0.0
    # position[8] = 0.0

    #Print the contents of the message to the console
    #print(rospy.get_name() + ": I heard %s" % message.data)
    joint_angle = [-1.263,0.492,-1.1299,2.523,2.2001,1.739,-3.059]
    print("Left Arm Position:\n")
    print(joint_angle)

    q = np.ndarray((3,8))
    w = np.ndarray((3,7))

    q[0:3,0] = [0.0635, 0.2598, 0.1188]
    q[0:3,1] = [0.1106, 0.3116, 0.3885]
    q[0:3,2] = [0.1827, 0.3838, 0.3881]
    q[0:3,3] = [0.3682, 0.5684, 0.3181]
    q[0:3,4] = [0.4417, 0.6420, 0.3177]
    q[0:3,5] = [0.6332, 0.8337, 0.3067]
    q[0:3,6] = [0.7152, 0.9158, 0.3063]
    q[0:3,7] = [0.7957, 0.9965, 0.3058]

    w[0:3,0] = [-0.0059,  0.0113,  0.9999]
    w[0:3,1] = [-0.7077,  0.7065, -0.0122]
    w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,3] = [-0.7077,  0.7065, -0.0122]
    w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,5] = [-0.7077,  0.7065, -0.0122]
    w[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                          [-0.7040, 0.7102, -0.0053],
                          [0.7102, 0.7040, 0.0055]]).T

    # YOUR CODE HERE
    #base = np.array([0,0,0])
    
    #g_st(0)
    g_st0 = np.ndarray((4,4))
    g_st0[0:3,0:3] = R
    g_st0[3,0:3] = np.array([0,0,0])
    g_st0[0:3,3] = q[0:3,7]
    g_st0[3,3] = 1

    #twists
    xi = np.ndarray((6,7))
    for i in range(0,7):
        xi[0:3,i] = np.cross(-w[0:3,i],q[0:3,i])
        xi[3:6,i] = w[0:3,i]
    xi_hat = kfs.prod_exp(xi,joint_angle)

    g = np.matmul(xi_hat,g_st0)
    print(g)
    test_result = g[0:3,3]

    print("Transformation matrix:\n")
    print(test_result)

#Python's syntax for a main() method
if __name__ == '__main__':
    # listener()
    lab5_test()
