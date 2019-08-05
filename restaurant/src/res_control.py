#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/06/25
    Author: Zijian Guo
    Abstract: Code for restaurant project
"""
import roslib
'''
roslib.load_manifest ('speech')
roslib.load_manifest () reads the package manifest and sets up the python library path based on the package dependencies. 
It's required for older rosbuild-based packages, but is no longer needed on catki
'''
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os
import sys
import cv2
import time
import wave
import datetime
import pyaudio
import numpy as np
#from kamerider_speech.msg import mission
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
from play_signal_sound import play_signal_sound
from turtlebot_msgs.srv import SetFollowState
from geometry_msgs.msg import Pose, Twist
from kamerider_image_msgs.msg import mission
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class restaurant(object):
    """
        class for restaurant
    """
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.question_start_signal = rospy.get_param("~question_start_signal", "/home/amore/robook_ws/src/kamerider_speech/sounds/question_start_signal.wav")

        self.pub_destination_topic_name = None
        self.pub_kws_topic_name         = None
        self.pub_twist_topic_name       = None
        self.pub_arm_command_topic_name = None
        self.pub_bar_pose_topic_name    = None

        self.sub_xfei_back_topic_name   = None
        self.sub_is_reach_topic_name    = None
        self.sub_amcl_pose_topic_name   = None
        self.sub_door_detect_topic_name = None
        self.sub_start_topic_name       = None
        self.sub_camera_topic_name       = None

        # Variables
        self.path_save = '/home/amore/robook_ws/src/restaurant/result/man_in_yellow.jpg'
        self.objects = ['tuna','nutella','jelly','pringles','tea','noodle','juice','pepsi','bean']
        self.object = ''
        self.is_stop            = False
        self.is_start           = False
        self.is_start_find_person = False
        self.is_find_person     = False
        self.detecting_bar      = False
        self.is_stop            = False
        self.get_amcl_pose = False
        self.reach_bar_num = 0
        self.person_pose = Pose()
        self.bar_pose = Pose()
        self.bar_location = ''
        self.mid_x = 320
        self.mid_y = 200
        self.area = 90000

        self.ticks = 0
        self.rate = 50

        # Initialize sound client
        self.sh = SoundClient(blocking=True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        self.get_params()
        #self.start_res()
        #play_signal_sound()
        print("[INFO] START")
        while True:
            if self.is_start_find_person:
                file_path = '/home/amore/robook_ws/data/haarcascades/haarcascade_frontalface_default.xml'
                # 人脸识别分类器
                faceCascade = cv2.CascadeClassifier(file_path)
                # 开启摄像头
                cap = cv2.VideoCapture(1)
                ok = True
                # set yellow thresh[0, 70, 70], [100, 255, 255]
                lower_yellow = np.array([0, 70, 70])
                upper_yellow = np.array([100, 255, 255])
                kernel_4 = np.ones((4,4),np.uint8)#4x4的卷积核
                while not self.is_find_person:
                    # 读取摄像头中的图像，ok为是否读取成功的判断参数
                    ok, img = cap.read()

                    # 读取ros话题中的图像
                    # img = self.camera_img

                    # 高斯滤波
                    img = cv2.GaussianBlur(img,(5,5),0)
                    # 转换成HSV
                    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    # get mask
                    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                    # detect yellow
                    # res = cv2.bitwise_and(img, img, mask=mask)
                    #下面四行是用卷积进行滤波
                    erosion = cv2.erode(mask,kernel_4,iterations = 1)
                    erosion = cv2.erode(erosion,kernel_4,iterations = 1)
                    dilation = cv2.dilate(erosion,kernel_4,iterations = 1)
                    dilation = cv2.dilate(dilation,kernel_4,iterations = 1)
                    #target是把原图中的非目标颜色区域去掉剩下的图像
                    target = cv2.bitwise_and(img, img, mask=dilation)
                    #将滤波后的图像变成二值图像放在binary中
                    ret, binary = cv2.threshold(dilation,127,255,cv2.THRESH_BINARY) 
                    #在binary中发现轮廓，轮廓按照面积从小到大排列
                    binary, contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    
                    if len(contours) == 0:
                        print('[INFO] Fail to find the costumer')
                        msg = Twist()
                        msg.angular.z = 0.35
                        self.pub_twist.publish(msg)

                    else:
                        x,y,w,h = cv2.boundingRect(contours[0])
                        # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,),2)
                        # 找出最大的框
                        target_x = 0
                        target_y = 0
                        target_w = 0
                        target_h = 0
                        for item in contours:
                            x,y,w,h = cv2.boundingRect(item)
                            if w*h>target_h*target_w:
                                target_h = h
                                target_w = w
                                target_x = x
                                target_y = y
                        print('[INFO] Find person in (%d, %d)'%(target_x, target_y))
                        print('[INFO] person area = ', target_h*target_w)
                        cv2.rectangle(img,(target_x, target_y),(target_x + target_w, target_y + target_h),(0,255,),2)
                        # 按框的位置来判断机器人运动
                        twist_msg = Twist()
                        target_x = target_x + target_w/2
                        target_y = target_y + target_h/2
                        if target_w*target_h <= 5000:
                            print('[INFO] Fail to find the costumer')
                            #msg = Twist()
                            #msg.angular.z = 0.35
                            #self.pub_twist.publish(msg)
                            
                            if self.bar_location == 'right':
                                msg = Twist()
                                msg.angular.z = -0.35
                                self.pub_twist.publish(msg)
                                #rospy.sleep(1)
                            else:
                                msg = Twist()
                                msg.angular.z = 0.35
                                self.pub_twist.publish(msg)
                                #rospy.sleep(1)
                            
                        elif abs(target_x-self.mid_x)<100:
                            if abs(target_w*target_h-self.area)<5000:
                                self.is_stop = True
                                self.is_find_person = True
                                self.sh.say('i have found the person')
                                self.start_res()
                                cv2.imwrite(self.path_save, img)
                            elif target_w*target_h-self.area>5000:
                                twist_msg.linear.x = -0.1
                                self.pub_twist.publish(twist_msg)
                                #rospy.sleep(1)
                            else:
                                twist_msg.linear.x = 0.1
                                self.pub_twist.publish(twist_msg)
                                #rospy.sleep(1)
                        elif target_x-self.mid_x>100:
                            twist_msg.angular.z = -0.2
                            self.pub_twist.publish(twist_msg)
                            #rospy.sleep(1)
                        else:
                            twist_msg.angular.z = 0.2
                            self.pub_twist.publish(twist_msg)
                            #rospy.sleep(1)
                    cv2.imshow('image', img)
                    #cv2.imshow('Mask', mask)
                    #cv2.imshow('Result', res)
                    cv2.waitKey(1)
                cv2.destroyAllWindows()


    def get_params(self):

        self.pub_destination_topic_name = rospy.get_param("pub_destination_topic_name", "/destination")
        self.pub_kws_topic_name         = rospy.get_param('pub_kws_topic_name',         '/kws_data')
        self.pub_twist_topic_name       = rospy.get_param('pub_twist_topic_name',       '/cmd_vel_mux/input/navi')
        self.pub_arm_command_topic_name_1 = rospy.get_param('pub_arm_command_topic_name_1', '/nav2arm')
        self.pub_arm_command_topic_name_2 = rospy.get_param('pub_arm_command_topic_name_2', '/navi_finish')
        self.pub_person_pose_topic_name = rospy.get_param('pub_person_pose_topic_name', '/person_pose')
        self.pub_bar_pose_topic_name    = rospy.get_param('pub_bar_pose_topic_name',    '/bar_pose')

        self.sub_is_reach_topic_name    = rospy.get_param("sub_is_reach_topic_name",    "/is_reach")
        self.sub_xfei_back_topic_name   = rospy.get_param("sub_xfei_back_topic_name",   "/baidu_to_control")
        self.sub_to_speech_topic_name   = rospy.get_param("sub_to_speech_topic_name",   '/kamerider_speech/input')
        self.sub_amcl_pose_topic_name   = rospy.get_param('sub_amcl_pose_topic_name',   '/amcl_pose')
        self.sub_door_detect_topic_name = rospy.get_param('sub_door_detect_topic_name', '/kamerider_image/door_detect')
        self.sub_start_topic_name       = rospy.get_param('sub_start_topic_name',       '/start')
        # self.sub_camera_topic_name      = rospy.get_param('sub_camera_topic_name',      '/camera1/rgb/image_raw')

        self.pub_destination = rospy.Publisher(self.pub_destination_topic_name, String, queue_size=10)
        self.pub_arm_command_1 = rospy.Publisher(self.pub_arm_command_topic_name_1, String, queue_size=10)
        self.pub_arm_command_2 = rospy.Publisher(self.pub_arm_command_topic_name_2, String, queue_size=10)
        self.pub_twist       = rospy.Publisher(self.pub_twist_topic_name,       Twist,  queue_size=1)
        self.pub_person_pose = rospy.Publisher(self.pub_person_pose_topic_name, Pose,   queue_size=1)
        self.pub_bar_pose    = rospy.Publisher(self.pub_bar_pose_topic_name,    Pose,   queue_size=1)

        rospy.Subscriber(self.sub_xfei_back_topic_name, String,                     self.xfeiCallback)
        rospy.Subscriber(self.sub_is_reach_topic_name, String,                      self.isReachCallback)
        rospy.Subscriber(self.sub_start_topic_name, String,                         self.startCallback)
        rospy.Subscriber(self.sub_door_detect_topic_name, String,                   self.doorCallback)
        rospy.Subscriber(self.sub_amcl_pose_topic_name, PoseWithCovarianceStamped,  self.amclCallback)
        # rospy.Subscriber(self.sub_camera_topic_name, Image,                         self.cameraCallback)

    def start_res(self):
        # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
        self.sh.say("Hello my name is Jack", self.voice)
        self.sh.say('If you need my help, please say "Hi Jack", then you can tell me your order after the prompt tone.', self.voice)
        
    def startCallback(self, msg):
        rospy.sleep(3)
        if msg.data == 'start':
            twist_msg = Twist()
            #twist_msg.angular.z = 2.5
            #self.pub_twist.publish(twist_msg)

            twist_msg.angular.z = 0.35
            goal_angualr = 0.5*3.14
            angular_duration = int(goal_angualr / twist_msg.angular.z)
            self.ticks = angular_duration * self.rate

            r = rospy.Rate(self.rate)
            #i = 0
            for i in range(self.ticks):
                 self.pub_twist.publish(twist_msg)
                 i=i+1
                 r.sleep()
            #rospy.sleep(1)
            os.system('gnome-terminal -x bash -c "rosrun kamerider_image_detection door_detect"')
            self.detecting_bar = True
    
    def cameraCallback(self, msg):
        # 将接收到的图像转换到opencv
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.camera_img = cv_image

    def amclCallback(self, msg):
        if self.is_stop:
            self.person_pose.position.x = msg.pose.pose.position.x
            self.person_pose.position.y = msg.pose.pose.position.y
            self.person_pose.position.z = msg.pose.pose.position.z
            self.person_pose.orientation.x = msg.pose.pose.orientation.x
            self.person_pose.orientation.y = msg.pose.pose.orientation.y
            self.person_pose.orientation.z = msg.pose.pose.orientation.z
            self.person_pose.orientation.w = msg.pose.pose.orientation.w
            print('[INFO] Get person pose in amcl')
            self.is_stop = False
        if self.get_amcl_pose:
            self.bar_pose.position.x = msg.pose.pose.position.x
            self.bar_pose.position.y = msg.pose.pose.position.y
            self.bar_pose.position.z = msg.pose.pose.position.z
            self.bar_pose.orientation.x = msg.pose.pose.orientation.x
            self.bar_pose.orientation.y = msg.pose.pose.orientation.y
            self.bar_pose.orientation.z = msg.pose.pose.orientation.z
            self.bar_pose.orientation.w = msg.pose.pose.orientation.w
            print('[INFO] Get bar pose in amcl')
            self.get_amcl_pose = False

    def doorCallback(self, msg):
        if self.detecting_bar:
            if msg.data == 'door_is_open':
                self.sh.say('the bar is on the right')
                self.bar_location = 'right'
                self.detecting_bar = False
            else:
                self.sh.say('the bar is on the left')
                self.bar_location = 'left'
                self.detecting_bar = False
            os.system('gnome-terminal -x bash -c "rosnode kill /door_detect"')
            twist_msg = Twist()
            #twist_msg.angular.z = -0.3
            #self.pub_twist.publish(twist_msg)

            twist_msg.angular.z = -0.35
            goal_angualr = - 0.5*3.14
            angular_duration = int(goal_angualr / twist_msg.angular.z)
            self.ticks = angular_duration * self.rate

            r = rospy.Rate(self.rate)
            #i = 0
            for i in range(self.ticks):
                 self.pub_twist.publish(twist_msg)
                 i=i+1
                 r.sleep()
            
            rospy.sleep(1)
            twist_msg.angular.z = 0
            twist_msg.linear.x = 0.15
            
            goal_linear = 0.5
            linear_duration = int(goal_linear / twist_msg.linear.x)
            self.ticks = linear_duration * self.rate
            
            for i in range(self.ticks):
                 self.pub_twist.publish(twist_msg)
                 i=i+1
                 r.sleep()            
            
            rospy.sleep(1)
            self.is_start_find_person = True
            self.get_amcl_pose = True
                

    def isReachCallback(self, msg):
        if msg.data == "in_position_bar":
            self.reach_bar_num += 1
            if self.reach_bar_num % 2 != 0:
                self.sh.say('I have reached the bar', self.voice)
                self.sh.say('the costumer ordered ' + self.object, self.voice)
                self.sh.say('please put the object in my arm', self.voice)
                self.pub_arm_command_1.publish('nav2arm')
                rospy.sleep(30)
                self.pub_person_pose.publish(self.person_pose)
            else:
                self.sh.say('i have reached the bar', self.voice)
                self.sh.say('i will find next costumer', self.voice)
                self.is_start_find_person = True
                self.is_find_person = False
                self.person_pose = Pose()

        if msg.data == 'in_position_person':
            self.sh.say('here is the ' + self.object, self.voice)
            self.pub_arm_command_2.publish('navi_finish')
            rospy.sleep(30)
            self.sh.say('now i will go back to bar', self.voice)
            self.pub_bar_pose.publish(self.bar_pose)

    
    def xfeiCallback(self, msg):
        self.object = ''
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        if string[-1] in symbols:
            string = string[:-1]
        for part in string.lstrip().split(","):
            for word in part.split():
                for symbol in symbols:
                    if symbol in word:
                        word = word[:-1]
                output.append(word)
        output = [item.lower() for item in output]
        print (output)
        for obj in self.objects:
            if obj in output:
                self.object = obj
        # 添加误识别
        if 'to' in output or 'now' in output or 'true' in output:
            self.object = 'tuna'
        elif 't' in output:
            self.object = 'tea'
        elif 'sprinkles' in output or 'pringle' in output:
            self.object = 'pringles'
        elif 'noodles' in output or 'instant' in output:
            self.object = 'instant noodle'
        elif 'been' in output or 'beans' in output or 'baked' in output:
            self.object = 'baked bean'
        elif 'juice' in output or 'orange' in output:
            self.object = 'orange juice'
        elif 'new' in output or 'color' in output:
            self.object = 'nutella'
        # 未获得结果
        if self.object == '':
            self.sh.say('please tell me your order again')
        else:
            self.sh.say('i heared your order is ' + self.object, self.voice)
            # 获取人的位置
            self.is_stop = True
            # 将bar的位置导航
            self.pub_bar_pose.publish(self.bar_pose)

    def cleanup(self):
        self.sh.say("I have finished restaurant task", self.voice)

if __name__ == '__main__':
    rospy.init_node("cml_control", anonymous=True)
    ctrl = restaurant()
    rospy.spin()
