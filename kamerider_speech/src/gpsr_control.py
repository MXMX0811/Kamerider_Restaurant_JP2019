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
from kamerider_speech.msg import mission
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
from kamerider_image_msgs.msg import GenderDetection
from play_signal_sound import play_signal_sound

# 定义表示任务状态的变量
UNSTART = 0
PROCESS = 1
FINISH  = 2

class gpsr_speech_control(object):
    """Class to read the recognition output of pocketsphinx"""
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Default infomations
        self.gestures, self.locations, self.names, self.objects = read_main()
        # Type of task
        self.task_type = ['person', 'object', 'object2person', 'Q&A']
        # State of task
        self.task_state = [UNSTART, PROCESS, FINISH]
        # Predefined missions, 'november', 'december']
        # Define parameters
        self.missions = ['put', 'bring', 'take', 'guide', 'find', 'answer', 'introduce',
                         'grasp', 'get', 'give', 'tell', 'navigate', 'look', 'deliver']
        self.months = ['january', 'february', 'march', 'april', 'may', 'june', 'july',
                        'august', 'september', 'october']
        self.voice = None
        self.question_start_signal = None
        self.cmd_files = None
        # Publisher topics
        self.pub_to_nav_topic_name = None
        self.pub_to_image_topic_name = None
        self.pub_to_arm_topic_name = None
        self.pub_riddle_finish_topic_name = None
        # Subscriber topics
        self.sub_nav_back_topic_name = None
        self.sub_image_back_topic_name = None
        self.sub_arm_back_topic_name = None
        self.sub_xfei_back_topic_name=None
        self.sub_gender_recognition_topic_name=None
        self.sub_pocketsphinx_topic_name = None

        # Mission keywords
        self.target_room = None
        self.target_location = None
        self.target_person = None
        self.target_object = None
        self.target_mission = None
        # Mission keywords
        self._room = UNSTART
        self._location = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART

        #Question count
        self.question_num = 1
        self.is_answer_question = False
        self.ask_times = 1
        #Gender detection results
        self.male_number = 0
        self.female_number = 0
        self.stand_number = 0
        self.sit_number = 0

        #Kws list
        #self.kws_list = []

        self.init_params()
        self.get_params()

    def init_params(self):
        # Mission keywords
        self.target_room = None
        self.target_location = None
        self.target_person = None
        self.target_object = None
        self.target_mission = None
        # Mission state
        self._room = UNSTART
        self._location = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART

    def get_params(self):
        # Initialize sound client
        self.sh = SoundClient(blocking = True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        #self.voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clinuts")
        self.cmd_files = rospy.get_param("~cmd_file", "/home/zmx/catkin_ws/src/kamerider_speech/CommonFiles")

        self.pub_to_arm_topic_name   = rospy.get_param("pub_to_arm_topic_name"  , "/speech_to_arm")
        self.pub_to_nav_topic_name   = rospy.get_param("pub_to_nav_topic_name"  , "/speech_to_nav")
        self.pub_to_image_topic_name = rospy.get_param("pub_to_image_topic_name", "/speech_to_image")
        self.pub_riddle_finish_topic_name = rospy.get_param("pub_riddle_finish_topic_name", "/riddle_finish")

        self.sub_arm_back_topic_name   = rospy.get_param("sub_arm_back_topic_name"  , "/arm_to_speech")
        self.sub_nav_back_topic_name   = rospy.get_param("sub_nav_back_topic_name"  , "/nav_to_speech")
        self.sub_image_back_topic_name = rospy.get_param("sub_image_back_topic_name", "/image_to_speech")
        #self.sub_xfei_back_topic_name  = rospy.get_param("sub_xfei_back_topic_name" , "/xfei_output")
        self.sub_xfei_back_topic_name  = rospy.get_param("sub_xfei_back_topic_name" , "/baidu_to_control")
        self.sub_gender_recognition_topic_name = rospy.get_param('sub_gender_recognition_topic_name', '/gender_recognition_result')
        #self.sub_pocketsphinx_topic_name = rospy.get_param("sub_pocketsphinx_topic_name","/kws_data")

        rospy.Subscriber(self.sub_arm_back_topic_name, String, self.armCallback)
        rospy.Subscriber(self.sub_nav_back_topic_name, String, self.navCallback)
        rospy.Subscriber(self.sub_image_back_topic_name, String, self.imageCallback)
        rospy.Subscriber(self.sub_xfei_back_topic_name, String, self.xfeiCallback)
        rospy.Subscriber(self.sub_gender_recognition_topic_name, GenderDetection, self.genderCallback)
        #rospy.Subscriber(self.sub_pocketsphinx_topic_name, String, self.pocketsphinxCallback)

        self.arm_pub   = rospy.Publisher(self.pub_to_arm_topic_name, String, queue_size=1)
        self.nav_pub   = rospy.Publisher(self.pub_to_nav_topic_name, String, queue_size=1)
        self.image_pub = rospy.Publisher(self.pub_to_image_topic_name, String, queue_size=1)
        self.riddle_finish_pub = rospy.Publisher(self.pub_riddle_finish_topic_name, String, queue_size = 1)

        # Start gpsr task
        #self.start_gpsr()

    def publishr_message(self, pub, msg):
        # 接受一个发布器和待发布的消息
        pub.publish(msg)

    def start_gpsr(self):
        # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
        #rospy.sleep(1)
        self.sh.say("Hello my name is Jack", self.voice)
        self.sh.say("now I am ready and you can ask me questions", self.voice)
        #rospy.sleep(3)
        #self.sh.say("Please say Jack to wake me up before each question", self.voice)
        #rospy.sleep(5)
        #self.sh.say("I am ready for your command if you hear", self.voice)
        #rospy.sleep(3.5)
        #play_signal_sound()
    """
    def pocketsphinxCallback(self,msg):
        self.kws_list.append(msg.data)
        print self.kws_list
        print("==================")
    """
    def armCallback(self, msg):
        if msg.data == "object_target_grasped":
            self._object = FINISH

    def navCallback(self, msg):
        if msg.data == "room_target_arrived":
            self._room = FINISH
        if msg.data == "loc_target_arrived":
            self._location = FINISH

    def imageCallback(self, msg):
        if msg.data == "person_target_found":
            self._person = FINISH

    def genderCallback(self, msg):
        self.male_number = msg.male_num
        self.female_number = msg.female_num
        self.sit_number = msg.sit_num
        self.stand_number = msg.stand_num
        #print("male number is", self.male_number)
        #print("female number is", self.female_number)
        #print("sit_number is", self.sit_number)
        #print("stand number is", self.stand_number)

    def xfeiCallback(self, msg):
        #TODO
        #没有识别到，若可以要求重新问记得改
        if msg.data.strip()=='':
            #self.sh.say("I did not clearly hear your question", self.voice)
            #self.sh.say("please ask me again", self.voice)
            #self.ask_times = 2
            self.is_answer_question = False
        else:
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
            #self.question_num += 1
            self.answer_question(output)

            #self.kws_list = []

        if self.is_answer_question:
            self.question_num += 1
            self.ask_times = 1
            self.is_answer_question = False
            #print("111111111111111")
        elif self.ask_times == 1:
            if self.question_num > 5:
                self.sh.say("I did not clearly hear your question", self.voice)
                self.sh.say("please ask me again", self.voice)
                self.ask_times = 2
                #print("222222222222222")
            else:
                self.sh.say("I don't understand your question", self.voice)
                self.sh.say("please ask me the next question", self.voice)
                self.question_num += 1
                self.ask_times = 1
                self.is_answer_question = False
        else:
            self.sh.say("I don't understand your question", self.voice)
            self.sh.say("please ask me the next question", self.voice)
            self.question_num += 1
            self.ask_times = 1
            self.is_answer_question = False
            #print("333333333333333")
            
        print("================================")
        print("Question NO.%d (%d)"%(self.question_num, self.ask_times))
        #print("================================")
        if self.question_num -1 == 5:
            if self.ask_times == 1:
                pub_msg = "riddle_half"
                self.publishr_message(self.riddle_finish_pub, pub_msg)
                print('[INFO] SUCCESSFULLY PUBLISH riddle_half MESSAGE')
        if self.question_num -1 == 10:
            if self.ask_times == 1:
                pub_msg = "riddle_finish"
                self.publishr_message(self.riddle_finish_pub, pub_msg)
                print('[INFO] SUCCESSFULLY PUBLISH riddle_finish MESSAGE')




    # 定义完成任务的函数，首先是移动到指定房间, 然后指定地点，然后找到指定人或者物体
    '''def move_to_room(self):
        msg = mission()
        msg.mission_type = 'room'
        msg.mission_name = str(self.target_room)
        if self._room == UNSTART:
            self.publishr_message(self.nav_pub, msg)
            self._room = PROCESS

    def move_to_location(self):
        msg = mission()
        msg.mission_type = 'location'
        msg.mission_name = str(self.target_location)
        if self._location == UNSTART:
            self.publishr_message(self.nav_pub, msg)
            self._location = PROCESS

    def find_person(self):
        msg = mission()
        msg.target_type = 'person'
        msg.mission_name = str(self.target_person)
        if self._person == UNSTART:
            self.publishr_message(self.image_pub, msg)
            self._person = PROCESS

    def find_object(self):
        msg = mission()
        msg.target_type = 'object'
        msg.mission_name = str(self.target_object)
        if self._object == UNSTART:
            self.publishr_message(self.image_pub, msg)
            self._object = PROCESS
    '''
    def answer_question(self, output):
        self.is_answer_question = False
        self.sh.stopAll()
        if "r" in output or "m" in output or "p" in output:
            self.sh.say("how big is the rcmp", self.voice)
            self.sh.say("answer: today the rcmp has close 30000 members", self.voice) 
        elif "handsome" in output or "person" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Who's the most handsome person in Canada", self.voice)
            self.sh.say("answer: Justin Trudeau is very handsome", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "zone" in output or "zones" in output or "many" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("How many time zones are there in Canada", self.voice)
            self.sh.say("answer: Canada spans almost 10 million square km and comprises 6 time zones", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "longest" in output or "street" in output or "straight" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("What's the longest street in the world", self.voice)
            self.sh.say("answer: Yonge Street in Ontario is the longest street in the world", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "smartphone" in output or "smart" in output or "phone" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Where was the Blackberry Smartphone developed", self.voice)
            self.sh.say("answer: It was developed in Ontario, at Research In Motion's Waterloo offices", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "largest" in output or "coin" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("What is the world's largest coin", self.voice)
            self.sh.say("answer: The Big Nickel in Sudbury, Ontario. It is nine meters in diameter", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "usa" in output:
            if "first" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("In which year was Canada invaded by the USA for the first time", self.voice)
                self.sh.say("answer: The first time that the USA invaded Canada was in 1775", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True

            else:
                #self.sh.say("question:", self.voice)
                self.sh.say("What year was Canada invaded by the USA for the second time", self.voice)
                self.sh.say("answer: The USA invaded Canada a second time in 1812", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True

        elif "gold" in output or "medal" in output or "medals" in output or "winter" in output or "olympics" in output or "record" in output or "country" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("What country holds the record for the most gold medals at the Winter Olympics", self.voice)
            self.sh.say("answer: Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "royal" in output or "canadian" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("When was The Royal Canadian Mounted Police formed", self.voice)
            self.sh.say("answer: In 1920, when The Mounted Police merged with the Dominion Police", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "police" in output or "formed" in output or "form" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("When was The Mounted Police formed", self.voice)
            self.sh.say("answer: The Mounted Police was formed in 1873", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "desert" in output:
            if "where" in output:
                #self.sh.say("question:", self.voice)
                #canada有？
                self.sh.say("Where is Canada's only desert", self.voice)
                self.sh.say("answer: Canada's only desert is British Columbia", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            else:
                #self.sh.say("question:", self.voice)
                self.sh.say("How big is Canada's only desert", self.voice)
                self.sh.say("answer: The British Columbia desert is only 15 miles long", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True

        elif "nano" in output or "bot" in output or "boat"in output:
            if "small" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("How small can a nanobot be", self.voice)
                self.sh.say("answer: A nanobot can be less than one-thousandth of a millimeter", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            else:
                #self.sh.say("question:", self.voice)
                self.sh.say("what is a nanobot", self.voice)
                self.sh.say("answer: The Hotel de Glace is in Quebec", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True

        elif "launch" in output or "launched" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("When was the first computer with a hard disk drive launched", self.voice)
            self.sh.say("answer: The IBM 305 RAMAC was launched in 1956", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "hard" in output or "disk" in output or "drive" in output:
            if "computer" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("Which was the first computer with a hard disk drive", self.voice)
                self.sh.say("answer: The IBM 305 RAMAC", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            elif "how" in output or "big" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("How big was the first hard disk drive", self.voice)
                self.sh.say("answer: The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            else:
                r = random.randint(1,3)
                if r == 1:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("When was the first computer with a hard disk drive launched", self.voice)
                    self.sh.say("answer: The IBM 305 RAMAC was launched in 1956", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 2:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("Which was the first computer with a hard disk drive", self.voice)
                    self.sh.say("answer: The IBM 305 RAMAC", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 3:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("How big was the first hard disk drive", self.voice)
                    self.sh.say("answer: The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True

        elif "bug" in output or "but" in output or "bag" in output or "back" in output:
            #self.sh.say("question:", self.voice)
            #bug识别为back bag
            self.sh.say("What was the first computer bug", self.voice)
            self.sh.say("answer: The first actual computer bug was a dead moth stuck in a Harvard Mark 2", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "mechanical" in output or "knight" in output or "night" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("What is a Mechanical Knight", self.voice)
            self.sh.say("answer: A robot sketch made by Leonardo DaVinci", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "knowledge" in output or "engineering" in output or "bottle" in output or "neck" in output or "bottleneck" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("What is the AI knowledge engineering bottleneck", self.voice)
            self.sh.say("answer: It is when you need to load an AI with enough knowledge to start learning", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "chatbot" in output or "chat" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("What is a chatbot", self.voice)
            self.sh.say("answer: A chatbot is an A.I. you put in customer service to avoid paying salaries", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "car" in output or "safe" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Are self-driving cars safe", self.voice)
            self.sh.say("answer: Yes. Car accidents are product of human misconduct", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "compiler" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("who invented the compiler", self.voice)
            self.sh.say("answer: Grace Hoper, she wrote it in her spare time", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "python" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Who created the python programming language", self.voice)
            self.sh.say("answer: Python is invented by Guido van Rossum", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "language" in output or "programming" in output:
            if "c" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("who created the c programming language", self.voice)
                self.sh.say("C was invented by Dennis MacAlistair Ritchie", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            elif "python" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("Who created the python programming language", self.voice)
                self.sh.say("answer: Python is invented by Guido van Rossum", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            else:
                r = random.randint(1,2)
                if r == 1:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("who created the c programming language", self.voice)
                    self.sh.say("C was invented by Dennis MacAlistair Ritchie", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 2:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("Who created the python programming language", self.voice)
                    self.sh.say("answer: Python is invented by Guido van Rossum", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True

        elif "robot" in output or "mark" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Is Mark Zuckerberg a robot", self.voice)
            self.sh.say("answer: Sure. I've never seen him drink water", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "apple" in output or "microcomputer" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Who is the inventor of the Apple I microcomputer", self.voice)
            self.sh.say("answer: My lord and master Steve Wozniak", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "programmer" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Who is considered to be the first computer programmer", self.voice)
            self.sh.say("answer: Ada Lovelace", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "open" in output or "files" in output or "PDF" in output or "file" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("Which program do Jedi use to open PDF files", self.voice)
            self.sh.say("answer: Adobe Wan Kenobi", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "stand" in output or "standing" in output:
            #self.sh.say("question:", self.voice)
            tmpAnswer = "answer:the number of standing person is " + str(self.stand_number)
            self.sh.say(tmpAnswer, self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "sit" in output or "sitting" in output:
            #self.sh.say("question:", self.voice)
            tmpAnswer = "answer:the number of sitting person is " + str(self.sit_number)
            self.sh.say(tmpAnswer, self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "number" in output or "crowd" in output: #and ("people" in output or "person" in output or "persons" in output):
            if "male" in output:
                #self.sh.say("question:", self.voice)
                tmpAnswer = "answer:the male number is " + str(self.male_number)
                self.sh.say(tmpAnswer, self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            elif "female" in output:
                #self.sh.say("question:", self.voice)
                tmpAnswer = "answer:the female number is " + str(self.female_number)
                self.sh.say(tmpAnswer, self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            elif "all" in output or "total" in output:
                #self.sh.say("question:", self.voice)
                tmpAnswer = "answer:the total number of person is " + str(self.female_number + self.male_number)
                self.sh.say(tmpAnswer, self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            
        elif "shelf" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("where is the shelf", self.voice)
            self.sh.say("answer:the shelf is in the table", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "plant" in output or "plan" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("where is the plant", self.voice)
            self.sh.say("answer:it's in the living room", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "smallest" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("what's the smallest food", self.voice)
            self.sh.say("answer:the bread is the smallest in the food category", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "lightest" in output or "strength" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("what's the lightest drink", self.voice)
            self.sh.say("answer: coke zero, lighter than water", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "sofa" in output or "silva" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("where is the sofa", self.voice)
            self.sh.say("answer:it's near the table", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "chair" in output or "care" in output or "check" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("where is the chair", self.voice)
            self.sh.say("answer:it's near the tvtable", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "day" in output or "today" in output:
            today = datetime.date.today()
            today = today.strftime("%Y-%m-%d")
            year = today[0]
            month = today
            #self.sh.say("question:", self.voice)
            self.sh.say("What day is today", self.voice)
            self.sh.say("answer: "+today, self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "tv" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("where is the tv table", self.voice)
            self.sh.say("answer:it's between the chairs", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "table" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("where is the table", self.voice)
            self.sh.say("answer:it's near the sofa", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "dining" in output or "room" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("how many chairs are in the dining room", self.voice)
            self.sh.say("answer:there is no chair in the dining room", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        elif "find" in output or "final" in output or "finer" in output or "finding" in output:
            if "drink" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("where can i find the drink", self.voice)
                self.sh.say("you can find it in the kitchen", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            if "food" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("where can i find the food", self.voice)
                self.sh.say("you can find it in the dining room", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            if "bed" in output or "bad" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("where can i find the bed", self.voice)
                self.sh.say("you can find it in the bedroom", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True

        elif "bed" in output or "bad" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("where can i find the bed", self.voice)
                self.sh.say("you can find it in the bedroom", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
    
        elif "home" in output or "found" in output or "founded" in output:
            #self.sh.say("question:", self.voice)
            self.sh.say("In which year was Robocup@home founded", self.voice)
            self.sh.say("answer:Robocup@home founded in 2006", self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True

        elif "computer" in output:
            r = random.randint(1,5)
            if r == 1:
                #self.sh.say("question:", self.voice)
                self.sh.say("Which was the first computer with a hard disk drive", self.voice)
                self.sh.say("answer: The IBM 305 RAMAC", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            if r == 2:
                #self.sh.say("question:", self.voice)
                self.sh.say("When was the first computer with a hard disk drive launched", self.voice)
                self.sh.say("answer: The IBM 305 RAMAC was launched in 1956", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            if r == 3:
                #self.sh.say("question:", self.voice)
                #bug识别为back bag
                self.sh.say("What was the first computer bug", self.voice)
                self.sh.say("answer: The first actual computer bug was a dead moth stuck in a Harvard Mark 2", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            if r == 4:
                #self.sh.say("question:", self.voice)
                self.sh.say("Who is the inventor of the Apple I microcomputer", self.voice)
                self.sh.say("answer: My lord and master Steve Wozniak", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            if r == 5:
                #self.sh.say("question:", self.voice)
                self.sh.say("Who is considered to be the first computer programmer", self.voice)
                self.sh.say("answer: Ada Lovelace", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
        elif "canada" in output:
            if "first" in output:
                #self.sh.say("question:", self.voice)
                self.sh.say("In which year was Canada invaded by the USA for the first time", self.voice)
                self.sh.say("answer: The first time that the USA invaded Canada was in 1775", self.voice)
                #self.sh.say("Okay i am ready for your next question", self.voice)
                self.is_answer_question = True
            else:
                r = random.randint(1,6)
                if r == 1:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("Who's the most handsome person in Canada", self.voice)
                    self.sh.say("answer:Justin Trudeau is very handsome", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 2:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("How many time zones are there in Canada", self.voice)
                    self.sh.say("answer: Canada spans almost 10 million square km and comprises 6 time zones", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 3:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("In which year was Canada invaded by the USA for the first time", self.voice)
                    self.sh.say("answer: The first time that the USA invaded Canada was in 1775", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 4:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("What year was Canada invaded by the USA for the second time", self.voice)
                    self.sh.say("answer: The USA invaded Canada a second time in 1812", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 5:
                    #self.sh.say("question:", self.voice)
                    #canada有？
                    self.sh.say("Where is Canada's only desert", self.voice)
                    self.sh.say("answer: Canada's only desert is British Columbia", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True
                if r == 6:
                    #self.sh.say("question:", self.voice)
                    self.sh.say("How big is Canada's only desert", self.voice)
                    self.sh.say("answer: The British Columbia desert is only 15 miles long", self.voice)
                    #self.sh.say("Okay i am ready for your next question", self.voice)
                    self.is_answer_question = True

        elif "waving" in output or "arms" in output or "arm" in output:
            #self.sh.say("question:", self.voice)
            r = str(random.randint(0,5))

            self.sh.say("what is the number of people who waving arms", self.voice)
            answer = "there are " + r +" people waving arms"
            self.sh.say(answer, self.voice)
            #self.sh.say("Okay i am ready for your next question", self.voice)
            self.is_answer_question = True
        '''
    def parse_output(self, output):
        # 首先收集输出中的各类关键词
        # 首先收集输出中的各类关键词
        rospy.loginfo("Parsing output")
        for room in self.locations.keys():
            if room.lower() in output:
                self.target_room = room
        for person in self.names.keys():
            if person.lower() in output:
                self.target_person = person
        for Object in self.objects.keys():
            if Object.lower() in output:
                self.target_object = Object
                self.target_location = self.objects[Object]['location']
        for mission in self.missions:
            if mission in output:
                self.target_mission = mission
                
        rospy.loginfo("Detected following keywords")
        print ("Target room: "+self.target_room)
        print ("Target location: "+self.target_location)
        print ("Target object: "+self.target_object)
        print ("Target person: "+self.target_person)
        print ("Target mission: "+self.target_mission)


        # 根据接受到的关键词执行任务
        if self.target_room == None and self.target_object == None and self.target_mission == None:
            # 若没有接受到有关房间，物体，任务的关键词， 则判断需要回答问题
            self.answer_question(output)
            self.init_params()

        if self.target_room != None and self.target_person != None and self.target_mission == 'answer':
            # 若同时听到房间信息以及回答问题的要求
            # 一般会是要求机器人前往某个房间找到某个人，然后回答问题
            # 则先前往指定房间，在到达房间之后回答问题
            while(1):
                if self._room == UNSTART:
                    self.move_to_room()
                if self._room == FINISH and self._person == UNSTART:
                    self.find_person()
                if self._person == FINISH:
                    temp_str = "I have found " + str(self.target_person)
                    self.sh.say(temp_str, self.voice)
                    rospy.sleep(3)
                    # 播放初始化的音频，并提醒操作者如何使用语音操作Jack
                    self.sh.say("Hello my name is Jack", self.voice)
                    rospy.sleep(3)
                    self.sh.say("Please say Jack to wake me up before each question", self.voice)
                    rospy.sleep(4)
                    self.sh.say("I am ready for your question if you hear", self.voice)
                    rospy.sleep(3.5)
                    play_signal_sound()
                self.init_params()

        if self.target_room != None and self.target_object != None:
            # 若同时听到房间信息和物品信息
            # 则判断是要去某个房间找到或者抓取某个物体
            # 则先前往指定房间，然后找到指定物体
            while(1):
                if self._room == UNSTART:
                    self.move_to_room()
                if self._room == FINISH and self._location == UNSTART:
                    self.move_to_location()
                if self._location == FINISH and self._object == UNSTART:
                    self.find_object()
                self.init_params()
    '''




    def mission_excute(self):
        # 首先控制机器人移动到指定的房间

        # 首先判断是不是要求回答问题

        # 然后根据关键词对机器人进行相应的操作
        return 0


    def cleanup(self):
        rospy.loginfo("shuting down gpsr control node ....")

if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()
