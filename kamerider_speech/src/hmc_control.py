#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/04/05
    Author: Xu Yucheng
    Abstract: Code for help-me-carry project
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
import cv2
import sys
import time
import wave
import datetime
import pyaudio
#from kamerider_speech.msg import mission
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
from play_signal_sound import play_signal_sound
from turtlebot_msgs.srv import SetFollowState
from geometry_msgs.msg import Pose, Twist
from kamerider_image_msgs.msg import mission
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped

class help_me_carry(object):
    """
        class for help me carry
    """
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.question_start_signal = rospy.get_param("~question_start_signal", "/home/zmx/catkin_ws/src/kamerider_speech/sounds/question_start_signal.wav")

        self.pub_message_topic_name     = None
        self.pub_arm_command            = None
        self.pub_find_person_topic_name = None
        self.pub_car_topic_name         = None
        self.pub_cmd_vel_topic_name     = None

        self.sub_pocket_back_topic_name = None
        self.sub_xfei_back_topic_name   = None
        self.sub_is_reach_topic_name    = None
        self.sub_arm_finish_topic_name  = None
        self.sub_door_detection_topic_name = None
        self.sub_odom_topic_name        = None
        self.sub_to_speech_topic_name   = None
        self.sub_amcl_pose_topic_name = None
        #self.pub_get_pose_signal_topic_name        = None

        # Mission keywords
        self.target_room = None
        self.target_location = None
        self.target_person = None
        self.target_object = None
        self.target_mission = None
        self.mid_x = 320
        self.mid_y = 20

        # Variables
        self.mission_msg = mission()
        self.car_pose = Pose()
        self.tmp_car_pose = PoseWithCovariance()
        self.is_take_luggage = False
        self.is_door_open = False
        self.destination_room = None
        self.destination_door = None
        self.destination_place = None
        self.is_stop = False
        self.is_pub_car = False
        self.is_start = False
        self.is_pub_destination = False
        self.start_find_person = False
        self.is_find_person = False

        # Initialize sound client
        self.sh = SoundClient(blocking=True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Initialize SetFollowerState client
        self.set_state = rospy.ServiceProxy("/turtlebot_follower/change_state", SetFollowState)
        #self.set_state = rospy.ServiceProxy("/turtlebot_follower/enabled", SetFollowState)
        self.get_params()
        self.start_hmc()
        #play_signal_sound()
        print("START")
        while not self.is_find_person:
            if self.start_find_person:
                file_path = '/home/zmx/catkin_ws/data/haarcascades/haarcascade_frontalface_default.xml'
                faceCascade = cv2.CascadeClassifier(file_path)
                cap = cv2.VideoCapture(1)
                while not self.is_find_person:
                    success, img = cap.read()
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    faces = faceCascade.detectMultiScale(
                        gray,
                        scaleFactor=1.2,
                        minNeighbors=5,
                        minSize=(32, 32)
                    )
                    target_x = 0
                    target_y = 0
                    target_w = 0
                    target_h = 0
                    twist_msg = Twist()
                    if len(faces) == 0:
                        print('[INFO] Fail to detect persons')
                        twist_msg.angular.z = 0.2
                        pass
                    for (x,y,w,h) in faces:
                        if (w*h)>(target_h*target_w):
                            target_h = h
                            target_w = w
                            target_x = x
                            target_y = y
                    
                    print(target_x+target_w, target_y+target_h)
                    if abs((target_x+target_w/2)-self.mid_x)<20:
                        if abs(target_y-self.mid_y)<10:
                            self.is_find_person = True
                            self.start_find_person = False
                            self.sh.say("I have found the person", self.voice)
                            self.sh.say('please stand in front me', self.voice)
                            self.publishr_message(self.pub_arm_command, "release")
                            rospy.sleep(3)
                            self.sh.say("here you are", self.voice)
                        elif target_y-self.mid_y<10:
                            twist_msg.linear.x = -0.1
                        else:
                            twist_msg.linear.x = 0.1
                    elif (target_x+target_w/2)-self.mid_x < 20:
                        twist_msg.angular.z = 0.2
                    else:
                        twist_msg.angular.z = -0.2
                    self.pub_cmd_vel.publish(twist_msg)
                    cv2.rectangle(img, (target_x,target_y), (target_x+target_w,target_h+target_y), (0,0,255), 2)
                    cv2.imshow('find person', img)
                    cv2.waitKey(20)



    def get_params(self):
        self.pub_message_topic_name = rospy.get_param("pub_message_topic_name", "/destination")
        #self.pub_get_pose_signal_topic_name = rospy.get_param("pub_get_pose_signal_topic_name", "/get_pose")
        self.sub_is_reach_topic_name = rospy.get_param("sub_is_reach_topic_name", "/is_reach")
        self.sub_pocket_back_topic_name = rospy.get_param("sub_pocket_back_topic_name", "/lm_data")
        self.sub_xfei_back_topic_name = rospy.get_param("sub_xfei_back_topic_name", "/baidu_to_control")
        self.sub_arm_finish_topic_name = rospy.get_param("sub_arm_finish_topic_name", "/arm_done")
        self.sub_door_detection_topic_name = rospy.get_param("sub_door_detection_topic_name", "/kamerider_image/door_detect")
        self.sub_odom_topic_name = rospy.get_param("sub_odom_topic_name", "/odom")
        self.sub_person_detect_topic_name = rospy.get_param('sub_person_detect_topic_name',      '/image_to_control')
        self.sub_to_speech_topic_name          = rospy.get_param("sub_to_speech_topic_name",          '/kamerider_speech/input')
        self.sub_amcl_pose_topic_name           = rospy.get_param('sub_amcl_pose_topic_name',   'amcl_pose')

        self.pub_car_topic_name = rospy.get_param("pub_car_topic_name", "/car_pose")
        self.pub_find_person_topic_name = rospy.get_param("pub_find_person_topic_name", "/control_to_image")
        self.pub_arm_command_topic_name = rospy.get_param("pub_arm_command_topic_name", "/nav2arm")
        self.pub_cmd_vel_topic_name = rospy.get_param('pub_cmd_vel_topic_name', '/cmd_vel_mux/input/navi')
        self.pub_car = rospy.Publisher(self.pub_car_topic_name, Pose, queue_size=10)
        self.pub_message = rospy.Publisher(self.pub_message_topic_name, String, queue_size=10)
        self.pub_arm_command = rospy.Publisher(self.pub_arm_command_topic_name, String, queue_size=10)
        self.pub_find_person = rospy.Publisher(self.pub_find_person_topic_name, mission, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher(self.pub_cmd_vel_topic_name, Twist, queue_size=1)

        rospy.Subscriber(self.sub_xfei_back_topic_name, String, self.xfeiCallback)
        rospy.Subscriber(self.sub_is_reach_topic_name, String, self.isReachCallback)
        rospy.Subscriber(self.sub_arm_finish_topic_name, String, self.isArmFinishCallback)
        rospy.Subscriber(self.sub_door_detection_topic_name, String, self.doorCallback)
        rospy.Subscriber(self.sub_odom_topic_name, Odometry, self.odomCallback)
        rospy.Subscriber(self.sub_to_speech_topic_name, String, self.speechCallback)
        rospy.Subscriber(self.sub_amcl_pose_topic_name, PoseWithCovarianceStamped, self.amclCallback)
    
    def start_hmc(self):
        # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
        self.sh.say("Hello my name is Jack", self.voice)
        #self.sh.say("Please say Jack to wake me up before each command", self.voice)
        #self.sh.say("I am ready for your command if you hear", self.voice)
        #play_signal_sound()
        rospy.set_param("wait_for_command", True)
    
    #def pocketCallback(self, msg):
    def publishr_message(self, pub, msg):
        # 接受一个发布器和待发布的消息
        pub.publish(msg)
    
    def speechCallback(self, msg):
        self.sh.say(msg.data, self.voice)

    def amclCallback(self, msg):
        if self.is_stop:
            self.tmp_car_pose = msg.pose
            self.car_pose.position.x = self.tmp_car_pose.pose.position.x
            self.car_pose.position.y = self.tmp_car_pose.pose.position.y
            self.car_pose.position.z = self.tmp_car_pose.pose.position.z
            self.car_pose.orientation.x = self.tmp_car_pose.pose.orientation.x
            self.car_pose.orientation.y = self.tmp_car_pose.pose.orientation.y
            self.car_pose.orientation.z = self.tmp_car_pose.pose.orientation.z
            self.car_pose.orientation.w = self.tmp_car_pose.pose.orientation.w
            self.is_stop = False
            if self.is_pub_car == False:
                print("===============================")
                print('car_pose.position.x = ', self.car_pose.position.x)
                print('car_pose.position.y = ', self.car_pose.position.y)
                print('car_pose.position.z = ', self.car_pose.position.z)
                print('car_pose.orientation.x = ', self.car_pose.orientation.x)
                print('car_pose.orientation.y = ', self.car_pose.orientation.y)
                print('car_pose.orientation.z = ', self.car_pose.orientation.z)
                print('car_pose.orientation.w = ', self.car_pose.orientation.w)
                print("===============================")
                self.is_pub_car = True
            

    def odomCallback(self, msg):
        pass    

    def isReachCallback(self, msg):
        if msg.data == "in_position_car":
            self.sh.say("I have reached the car", self.voice)
            #self.sh.say("Now I start to find person", self.voice)
        if msg.data == "in_position":
            self.sh.say("I have reached the destination", self.voice)
            self.sh.say('Now i start to find person', self.voice)
            self.start_find_person =True

    def isArmFinishCallback(self, msg):
        if msg.data == "init_done":
            #self.is_take_luggage = True
            self.sh.say("now you can put the luggage on my arm", self.voice)
            self.publishr_message(self.pub_arm_command, 'grasp')
            rospy.sleep(3)
        if msg.data == "grasp_done":
            self.is_take_luggage = True
            self.sh.say("now you can tell me the destination", self.voice)
        if msg.data == "release_done":
            self.sh.say("please take the bag", self.voice)
            rospy.sleep(2)
            self.publishr_message(self.pub_arm_command, 'finish')
        if msg.data == "finish_done":
            self.sh.say(" now please follow me to the car",self.voice)
            rospy.sleep(2)
            self.publishr_message(self.pub_car, self.car_pose)

    def doorCallback(self, msg):
        pass
    
    def xfeiCallback(self, msg):
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
        '''if "start" in output or "let's" in output or "go" in output:
            if self.is_start == False:
                self.publishr_message(self.pub_message, "start")
                self.is_start = True
        '''
        if "failed" in output:
            self.sh.say("I did not clearly hear your command", self.voice)
            self.sh.say("Please ask me again", self.voice)

        if "follow" in output or "come" in output or "photo" in output:
            self.sh.say("Now i will start following you", self.voice)
            #self.sh.say("Please slow down if i cannot follow you", self.voice)
            state_msg = True
            response = self.set_state(state_msg)
            print("response status: ", response.result)
            #rospy.set_param("enable" , True)
            #rospy.set_param("wait_for_command" , False)
        
        if "stop" in output or "stay" in output or "car" in output or "destination" in output:
            state_msg = False
            response = self.set_state(state_msg)
            print("response status: ", response.result)
            #rospy.set_param("in_position", "true")
            #os.system("gnome-terminal -x bash -c 'roslaunch arm ed_arm_carry.launch'")
            self.sh.say("Ok i will stop following you", self.voice)
            self.sh.say("I have already remember the current location", self.voice)
            rospy.sleep(1)
            os.system("rosnode kill /camera_throttle")
            rospy.sleep(1)
            os.system("rosnode kill /follower_velocity_smoother")
            rospy.sleep(1)
            os.system("rosnode kill /turtlebot_follower")
            rospy.sleep(1)
            os.system("rosnode kill /switch")
            rospy.sleep(1)
            self.is_stop = True
            self.sh.say("Please hang the luggage on my arm", self.voice)
            rospy.sleep(1)
            #self.sh.say("I will wait for you for ten seconds", self.voice)
            self.publishr_message(self.pub_arm_command, 'init')
            rospy.sleep(1)

            #rospy.set_param("enable", False)
            #rospy.set_param("wait_for_command", True)
            #get_pose_msg = "get_pose"
            #self.publishr_message(self.pub_get_pose_signal, get_pose_msg)
            """
                @TODO
                此处对机械臂发消息，使机械臂向前伸出
            """
            
            
        if self.is_take_luggage:
            
            if "person" in output or "people" in output or "prison":
                if "kitchen" in output:
                    self.sh.say("I heared the destination is the kitchen", self.voice)
                    self.sh.say("I am on my way", self.voice)
                    self.destination_place = "kitchen"
                    self.publishr_message(self.pub_message, self.destination_place)
                    self.is_pub_destination = True
                if "living" in output or 'leaving' in output:
                    self.sh.say("I heared the destination is the living room", self.voice)
                    self.sh.say("I am on my way", self.voice)
                    self.destination_place = "livingroom"
                    self.publishr_message(self.pub_message, self.destination_place)
                    self.is_pub_destination = True
                if "bedroom" in output or "bathroom" in output:
                    self.sh.say("I heared the destination is the bedroom", self.voice)
                    self.sh.say("I am on my way", self.voice)
                    self.destination_place = "bedroom"
                    self.publishr_message(self.pub_message, self.destination_place)
                    self.is_pub_destination = True
            if self.is_pub_destination == False:
                self.sh.say('i did not hear the destination', self.voice)
                self.sh.say('please tell me again', self.voice)
            print('[INFO] destination_place is ', self.destination_place)

    def cleanup(self):
        self.sh.say("I have finished help me carry task", self.voice)

if __name__ == '__main__':
    rospy.init_node("hmc_control", anonymous=True)
    ctrl = help_me_carry()
    rospy.spin()
