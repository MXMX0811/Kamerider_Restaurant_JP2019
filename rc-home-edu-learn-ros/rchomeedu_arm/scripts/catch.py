#!/usr/bin/env python

"""
    arm.py - move robot arm according to predefined gestures

"""

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
import time
import math

class Loop:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

	# publish command message to joints/servos of arm
    	self.joint1 = rospy.Publisher('/waist_controller/command',Float64)
	self.joint2 = rospy.Publisher('/shoulder_controller/command',Float64)
    	self.joint3 = rospy.Publisher('/elbow_controller/command',Float64)
    	self.joint4 = rospy.Publisher('/wrist_controller/command',Float64)
	self.joint5 = rospy.Publisher('/hand_controller/command',Float64)
	self.init_pose = (0,math.pi/4,math.pi/2,-math.pi/4,-0.3)
	self.hold_pose = (0, -2.1, 2.1, math.pi/2, 0.2)
	self.release_pose = (0,math.pi/4,math.pi/2,-math.pi/4,-0.3)
	
	# Initial gesture of robot arm
	self.joint1.publish(self.init_pose[0])
	self.joint2.publish(self.init_pose[1])
	self.joint3.publish(self.init_pose[2])
	self.joint4.publish(self.init_pose[3])
	self.joint5.publish(self.init_pose[4])

	pub = rospy.Publisher('arm2navi', String, queue_size=1)

	while not rospy.is_shutdown():
		time.sleep(2)
		# gesture 1
		self.joint5.publish(self.hold_pose[4])
		rospy.sleep(4)
		self.joint2.publish(self.hold_pose[1])
		self.joint1.publish(self.hold_pose[0])
		self.joint3.publish(self.hold_pose[2])
		rospy.sleep(1)
		self.joint4.publish(self.hold_pose[3])

		rospy.wait_for_message('nav2arm', String)
		
		# gesture 2
		self.joint1.publish(self.init_pose[0])
		self.joint2.publish(self.init_pose[1])
		self.joint3.publish(self.init_pose[2])
		self.joint4.publish(self.init_pose[3])
		rospy.sleep(4)
		self.joint5.publish(self.init_pose[4])
		rospy.sleep(8)
		self.joint5.publish(0.4)
		rospy.sleep(5)
		self.joint2.publish(0)
		rospy.wait_for_message('navi_finish', String)
		print('countinue')

		# gesture 3
		self.joint1.publish(self.init_pose[0])
		self.joint2.publish(self.init_pose[1])
		self.joint3.publish(self.init_pose[2])
		self.joint4.publish(self.init_pose[3])
		rospy.sleep(4)
		self.joint5.publish(self.init_pose[4])
		rospy.sleep(4)


		
    def cleanup(self):
        rospy.loginfo("Shutting down robot arm....")

if __name__=="__main__":
    rospy.init_node('arm')
    try:
        Loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

