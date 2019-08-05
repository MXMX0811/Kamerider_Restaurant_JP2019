#import roslib; roslib.load_manifest('speech')
import rospy
import re
import os
from std_msgs.msg import String
from std_msgs.msg import Int8
import time
from sound_play.libsoundplay import SoundClient
import xml.etree.ElementTree as ET
#f=open('/home/kamerider/catkin_ws/src/speech/save.txt','w')
womannumber= 0
mannumber= 0
class spr:
	def __init__(self):

		self.
		# Create the sound client object
		self.soundhandle = SoundClient()
		rospy.sleep(1)
		self.riddle_turn = rospy.Publisher('turn_signal', String, queue_size=15)
		self.soundhandle.stopAll()
		self.soundhandle.say('hello I want to play riddles',self.voice)
		rospy.sleep(3)
		self.soundhandle.say('I will turn back after ten seconds',self.voice)
		rospy.sleep(3)
		self.soundhandle.say('ten',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('nine',self.voice)
		self.riddle_turn.publish("turn_robot")#publish msg to nav
		rospy.sleep(1)
		self.soundhandle.say('eight',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('seven',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('six',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('five',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('four',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('three',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('two',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('one',self.voice)
		rospy.sleep(1)
		self.soundhandle.say('here I come',self.voice)
		rospy.sleep(1)
		rospy.Subscriber('human_num', String, self.h_num)
		rospy.Subscriber('female_num', String, self.f_num)
		rospy.Subscriber('male_num', String, self.m_num)
		rospy.Subscriber('sit_num', String, self.sit_num)
		rospy.Subscriber('stand_num', String, self.stand_num)
		rospy.Subscriber('stand_male_num', String, self.sit_man_num)
		rospy.Subscriber('stand_female_num', String, self.sit_female_num)
		rospy.Subscriber('sit_male_num', String, self.stand_man_num)
		rospy.Subscriber('stand_female_num', String, self.stand_female_num)


		rospy.Subscriber('taking_photo_signal', String, self.remind_people)
		if self.if_say==0:
			rospy.Subscriber('recognizer_output', String, self.talk_back)
	def sit_num(self,msg):
		msg.data=msg.data.lower()
		self.sit_num=msg.data

	def stand_num(self,msg):
		msg.data=msg.data.lower()
		self.stand_num=msg.data

	def sit_man_num(self,msg):
		msg.data=msg.data.lower()
		self.sit_man_num=msg.data
	def sit_female_num(self,msg):
		msg.data=msg.data.lower()
		self.sit_female_num=msg.data
	def stand_man_num(self,msg):
		msg.data=msg.data.lower()
		self.stand_man_num=msg.data
	def stand_female_num(self,msg):
		msg.data=msg.data.lower()
		self.stand_female_num=msg.data

	def h_num(self,msg):
		msg.data=msg.data.lower()
		self.soundhandle.say('I have already taken the photo',self.voice)
		rospy.sleep(3)
		self.soundhandle.say('please wait for a moment',self.voice)
		rospy.sleep(3)
		self.crowd_num=msg.data
		print "human number is " + msg.data
		self.soundhandle.say('human number is  '+msg.data,self.voice)
		rospy.sleep(4)

	def f_num(self,msg):
		msg.data=msg.data.lower()
                womannumber= msg.data
		print "female number is " + msg.data
		self.female_num=msg.data
		self.soundhandle.say('female number is  '+msg.data,self.voice)
		rospy.sleep(4)

	def m_num(self,msg):
		msg.data=msg.data.lower()
		print "male number is " + msg.data
                mannumber= msg.data
		self.male_num=msg.data
		self.soundhandle.say('male number is ' +msg.data,self.voice)
		rospy.sleep(4)
		self.soundhandle.say('who wants to play riddles with me',self.voice)
		rospy.sleep(3.5)
		self.soundhandle.say('please stand in front of me and wait for five seconds',self.voice)
		rospy.sleep(8.5)
		self.soundhandle.say('please ask me after you hear',self.voice)
		rospy.sleep(2.5)
		self.soundhandle.playWave(self.question_start_signal+"/question_start_signal.wav")
		rospy.sleep(1.3)
		self.soundhandle.say('Im ready',self.voice)
		rospy.sleep(1.3)
		self.soundhandle.playWave(self.question_start_signal+"/question_start_signal.wav")
		rospy.sleep(1.3)
		self.w=1




if __name__=="__main__":
	rospy.init_node('SPR_sample')
	print('[INFO] =========== STARTING SPR ==========')
	SPR()
	rospy.spin()
