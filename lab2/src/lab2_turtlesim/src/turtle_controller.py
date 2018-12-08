#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist



#Define the method which contains the main functionality of the node.
def talker():

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('talker', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic /chatter_talk
  turtle = sys.argv[1]
  pub = rospy.Publisher('/' + turtle + '/cmd_vel', Twist, queue_size=10)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    # Construct a string that we want to publish
    # (In Python, the "%" operator functions similarly
    #  to sprintf in C or MATLAB)
    vel_msg = Twist()
    k = raw_input("Please enter a command and press <Enter>: ")
    if k == 'w':
    	vel_msg.linear.x = 1
    	vel_msg.linear.y = 0
    	vel_msg.linear.z = 0
    elif k == 's':
    	vel_msg.linear.x = -1
    	vel_msg.linear.y = 0
    	vel_msg.linear.z = 0
    elif k == 'a':
    	vel_msg.angular.z = -1
    elif k == 'd':
    	vel_msg.angular.z = 1
    #pub_string = TimestampString()
    #pub_string.user_input = user_msg
    #pub_string.time = rospy.get_time()

    # Publish our string to the 'chatter_talk' topic
    pub.publish(vel_msg)
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    talker()
  except rospy.ROSInterruptException: pass

