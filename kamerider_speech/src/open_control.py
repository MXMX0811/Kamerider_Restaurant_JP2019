#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2018/03/02
Author: Xu Yucheng
Abstract： 订阅输出Pocketsphinx识别结果的话题，然后根据输出的结果来控制机器人
'''

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
import time
import wave
import datetime
import pyaudio
import random
from sound_play.libsoundplay import SoundClient
from play_signal_sound import play_signal_sound

class open_control():
    def __init__(self):
        rospy.init_node("open_control")
        # 指示机器人当期所处位置
        self.cupboard = False
        self.diningroom = False

        # 人手中的物体

        # Initialize sound client
        self.sh = SoundClient(blocking = True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Subscriber & Publisher
        rospy.Subscriber("/nav_to_control", String, self.navCallback)
        self.nav_pub = rospy.Publisher("/control_to_nav", String, queue_size=1)

        self.start_open()

    
    def start_open(self):
        self.sh.say("Hello my name is jack")
        self.sh.say("This our Open Challenge")
        self.sh.say("i will deliver object to help setup the dining table")
        rospy.sleep(20)
        self.sh.say("now i will take the bowl from the cupboard")
        self.move_to_location("cupboard")
        while (True):
            if self.cupboard == True:
                self.cupboard = False
                self.sh.say("please give me the bowl")
                rospy.sleep(4)
                self.sh.say("i will deliver it to the person in dining room")
                self.move_to_location("diningroom")
                break
        while (True):
            if self.diningroom == True:
                self.diningroom = False
                self.sh.say("please help me place the bowl on the plate")
                rospy.sleep(20)
                self.sh.say("now i will take the ketchup from the cupboard")
                self.move_to_location("cupboard")
                break
        while (True):
            if self.cupboard == True:
                self.cupboard = False
                self.sh.say("please give me the ketchup")
                rospy.sleep(4)
                self.sh.say("i will deliver to the person in dining room")
                self.move_to_location("diningroom")
                break
        while (True):
            if self.diningroom == True:
                self.diningroom = False
                self.sh.say("please help me place the ketchup beside the glass")
                rospy.sleep(5)
                self.sh.say("now i will take the last thing chopsticks from the cupboard")
                self.move_to_location("cupboard")
                break
        while (True):
            if self.cupboard == True:
                self.cupboard = False
                self.sh.say("please give me the chopsticks")
                rospy.sleep(4)
                self.sh.say("i will deliver to the person in dining room")
                self.move_to_location("diningroom")
                break
        while (True):
            if self.diningroom == True:
                self.diningroom = False
                self.sh.say("please help me place the chopsticks in the bowl")
                rospy.sleep(3)
                self.sh.say("now i finish the task")


    
    def navCallback(self, msg):
        if msg.data == "cupboard_arrived":
            self.cupboard = True
        if msg.data == "diningroom_arrived":
            self.diningroom = True

    def move_to_location(self, location):
        msg = String()
        msg.data = location
        self.nav_pub.publish(msg)

if __name__ == "__main__":
    control = open_control()
    rospy.spin()

    
