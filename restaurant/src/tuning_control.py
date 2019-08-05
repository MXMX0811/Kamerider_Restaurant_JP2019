#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg import String
import os
import cv2
import sys
import time
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class catch_object(object):

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.pub_twist_topic_name           = None
        self.sub_depth_image_topic_name     = None
        self.sub_is_reach_topic_name        = None
        self.sub_detect_result_topic_name   = None
        self.twist_msg = Twist()
        self.depth_img = np.array((480,640,2))
        self.avg_depth = 0.0

        # Variables
        self.is_reach = True
        self.is_get_image = True
        self.is_find_bottle = False
        self.is_turn_angular = False
        self.is_turn_linear = False
        self.is_pub_nav2arm = False

        self.get_params()
        print('[INFO] Start Turning Distance and Angular')

    def get_params(self):
        #self.sub_depth_image_topic_name     = rospy.get_param("sub_depth_image_topic_name", "/camera/depth/image_raw")
        self.sub_is_reach_topic_name        = rospy.get_param("sub_is_reach_topic_name", "/catch_start")
        self.sub_detect_result_topic_name   = rospy.get_param('sub_detect_result_topic_name', '/darknet_ros/bounding_boxes')
        self.pub_twist_topic_name           = rospy.get_param('pub_twist_topic_name', '/cmd_vel_mux/input/navi')
        self.pub_nav2arm_topic_name         = rospy.get_param('pub_nav2arm_topic_name', '/nav2arm')

        self.pub_nav2arm = rospy.Publisher(self.pub_nav2arm_topic_name, String, queue_size=1)
        self.pub_twist = rospy.Publisher(self.pub_twist_topic_name, Twist, queue_size=1)
        rospy.Subscriber(self.sub_is_reach_topic_name, String, self.isReachCallback)
        #rospy.Subscriber(self.sub_depth_image_topic_name, Image, self.depthimgCallback)
        rospy.Subscriber(self.sub_detect_result_topic_name, BoundingBoxes, self.resultCallback)

    def isReachCallback(self, msg):
        if msg.data == 'ready_to_catch':
            self.is_reach = True
	'''
    def depthimgCallback(self, msg):
        if self.is_reach:
            bridge = CvBridge()
            self.depth_img = bridge.imgmsg_to_cv2(msg, msg.encoding)
            cv2.imshow('img', self.depth_img)
            cv2.waitKey(30)
            self.is_get_image = True
        else:
            #print('The robot has not reached the destination')
            pass
	'''
    def resultCallback(self, msg):
        #if self.is_turn_angular and self.is_turn_linear and self.is_pub_nav2arm == False:
        if self.is_turn_linear and self.is_pub_nav2arm == False:
            nav2arm_msg = String()
            nav2arm_msg.data = 'catch'
            print('catch')
            self.pub_nav2arm.publish(nav2arm_msg)
            self.is_pub_nav2arm = True
            
        self.is_get_image = True

        if self.is_get_image:
            xmin = 0
            xmax = 0
            ymin = 0
            ymax = 0
            middle_x = 0
            mid_x = 0
            area = 0
            self.avg_depth = 0.0
            boundingbox = BoundingBox()
            boundingbox = msg.bounding_boxes
            #wprint(boundingbox)
            for bx in boundingbox:
                if bx.Class == 'bottle':
                    xmin = bx.xmin
                    xmax = bx.xmax
                    ymin = bx.ymin
                    ymax = bx.ymax
                    mid_x = (xmin+xmax)/2
                    area = abs(xmax-xmin)*abs(ymax-ymin)
                    
                    if area >= self.avg_depth:
                    	self.avg_depth = area
                    	middle_x = mid_x
                    self.is_find_bottle = True

            if self.is_find_bottle:
                if self.is_turn_angular==False or self.is_turn_linear==False:
                	
                    if middle_x-320>50:
                        self.twist_msg.angular.z = -0.3
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_angular = False
                        self.is_get_image = False
                    elif middle_x-320>10:
                        self.twist_msg.angular.z = -0.1
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_angular = False
                        self.is_get_image = False
                    elif middle_x-320<-50:
                        self.twist_msg.angular.z = 0.3
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_angular = False
                        self.is_get_image = False
                    elif middle_x-320<-10:
                        self.twist_msg.angular.z = 0.1
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_angular = False
                        self.is_get_image = False
                    else:
                        self.is_turn_angular = True
                        self.is_get_image = False
                        print('[INFO] The Angular is Correct')
                        
                    
                    print(self.avg_depth)
                    if self.avg_depth > 40000:
                        self.twist_msg.linear.x = -0.15
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_linear = False
                        self.is_get_image = False
                    elif self.avg_depth > 30000:
                        self.twist_msg.linear.x = -0.05
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_linear = False
                        self.is_get_image = False
                    elif self.avg_depth < 15000:
                        self.twist_msg.linear.x = 0.05
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_linear = False
                        self.is_get_image = False
                    elif self.avg_depth < 10000:
                        self.twist_msg.linear.x = 0.15
                        self.pub_twist.publish(self.twist_msg)
                        self.is_turn_linear = False
                        self.is_get_image = False
                    else:
                        self.is_turn_linear = True
                        self.is_get_image = False
                        print('[INFO] The Distance is Correct')
            
            else:
                print('[INFO] Cannot Find Target Object')
        else:
            print('[INFO] Cannot Get Valid Image')

    def cleanup(self):
        print('[INFO] I have finished the task')

if __name__ == '__main__':
    rospy.init_node("catch_object", anonymous=True)
    ctrl = catch_object()
    rospy.spin()
