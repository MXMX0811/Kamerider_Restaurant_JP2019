#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/2/1
Author: Xu Yucheng
Abstract: Code for person detection
'''
import os
import cv2
import math
import rospy
import roslib
import base64
from aip import AipBodyAnalysis
from std_msgs.msg import Int16
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from kamerider_image_msgs.msg import mission



class person_detection:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # 定义baidu ai 的APP_ID、API_KEY、SECRET_KEY
        """
        根据认证信息获取access_token.
        access_token的有效期为30天，切记需要每30天进行定期更换，或者每次请求都拉取新token；
        """
        self.APP_ID     = '16002184'
        self.API_KEY    = 'W6jgTXcHFZioPcmXWilfaGO2'
        self.SECRET_KEY = 'oNewRv2mtqfknwzq3rgEFc3sP4SAbI0v'
        
        #ros params
        self.msg_count = 0
        self._turn = False
        self.find_person = False
        self.start_detect = False
        self.is_go_forward = True
        self.angle_count = 0
        self.take_photo_signal = False
        self.sub_image_raw_topic_name=None
        self.sub_control_topic_name = None
        self.pub_person_detect_topic_name=None
        self.sub_point_number_topic_name = None
        self.close_step = 0
        self.pic_num = 0
        self.is_stop_turn = False

        self.path_to_save_image=None
        self.pub_result=None
        self.move_pub = None
        self.speech_pub = None
        self.client = None
        self.get_params()
        #self.turn_robot(1.8)

        print('1')
        cap=cv2.VideoCapture(1)
        
        #if success:
            #self.start_detect = True
            #self.take_photo_signal = False
        while not rospy.is_shutdown():
            if self.take_photo_signal:
                if self.is_stop_turn:
                    print('find person')
                    while self.is_go_forward:
                        print("go_forward")
                        self.forward_robot(0.3)
                    rospy.loginfo("Person Target Reached!")
                    control_msg = String()
                    speech_msg = String()
                    control_msg.data = "person_target_found"
                    speech_msg.data = "I have reached the target person"
                    self.speech_pub.publish(speech_msg)
                    self.pub_result.publish(control_msg)
                    self.take_photo_signal = False
                if self.find_person == False:
                    
                    #if self.start_detect == True:
                    success,img=cap.read()
                    cv2.imshow('img', img)
                    cv2.waitKey(10)
                    # self.detection(img)
                    cv2.imwrite(self.path_to_save_image, img)
                    print('start detect')
                    self.detection()
                    self.pic_num += 1
               
                

    def get_params(self):
        #self.sub_image_raw_topic_name          = rospy.get_param('sub_image_raw_topic_name',          '/kinect_up/rgb/image_raw')
        self.sub_control_topic_name            = rospy.get_param("sub_control_topic_name",            '/control_to_image')
        self.pub_person_detect_topic_name      = rospy.get_param('pub_person_detect_topic_name',      '/image_to_control')
        self.pub_to_move_base_topic_name       = rospy.get_param('pub_to_move_base_topic_name',       '/cmd_vel_mux/input/navi')
        self.pub_to_speech_topic_name          = rospy.get_param("pub_to_speech_topic_name",          '/kamerider_speech/input')
        self.path_to_save_image                = rospy.get_param('path_to_save_image',                '/home/zmx/catkin_ws/src/kamerider_image/kamerider_image_detection/result/person_image_capture.jpg')
        self.path_to_save_result               = rospy.get_param('path_to_save_result',               '/home/zmx/catkin_ws/src/kamerider_image/kamerider_image_detection/result/person_detection_result.jpg')  
        self.sub_point_number_topic_name       = rospy.get_param('sub_point_number_topic',            "/point_num")   
        #定义R发布器和订阅器，话题名通过ROS PARAM参数服务器获取
        #rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        rospy.Subscriber(self.sub_control_topic_name, mission, self.controlCallback)
        rospy.Subscriber(self.sub_point_number_topic_name, Int16, self.pointCallback)
        self.pub_result = rospy.Publisher(self.pub_person_detect_topic_name, String, queue_size=1)
        self.move_pub   = rospy.Publisher(self.pub_to_move_base_topic_name, Twist, queue_size=1)
        self.speech_pub = rospy.Publisher(self.pub_to_speech_topic_name, String, queue_size=1)
    
    def controlCallback(self, msg):
        if msg.mission_type == "person":
            self.find_person = False
            self.take_photo_signal = True

    def pointCallback(self,msg):
        if self.is_stop_turn:
            print("[INFO] RECEIVE point number ", msg.data)
            if msg.data < 40000:
                self.close_step += 1
            if self.close_step > 15:
                self.is_go_forward = False

    '''
    def imageCallback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        #cv2.imshow("current", cv_image)
        #cv2.waitKey(10)
        if self.take_photo_signal == True:
            if self.find_person == False:
                print ("[INFO] Start to take photo")
                try:
                    cv2.imwrite(self.path_to_save_image, cv_image)
                    cv2.imshow("frame", cv_image)
                    cv2.waitKey(100)
                except CvBridgeError as e:
                    print (e)
            else:
                pass
            #self.start_detect = True
            #self.take_photo_signal = False
    '''

    def image_encode(self):
        with open(self.path_to_save_image, 'rb') as f:
            return f.read()

    #检测函数
    def detection(self):
        self.start_detect = False
        encode_image = self.image_encode()
        print (type(encode_image))
        image = cv2.imread(self.path_to_save_image)
        #初始化aipfacce对象
        client = AipBodyAnalysis(self.APP_ID, self.API_KEY, self.SECRET_KEY)
        #接收返回的检测结果
        result = client.bodyAttr(encode_image)
        #print (result)
        num = result["person_num"]
        area = []
        point = []
        person_x = 0
        height = 0
        try:
            if result["person_num"] == 0:
                print("person_num =", result["person_num"])
                rospy.loginfo("Start Turning robot")
                self.turn_robot(0.7)
                self.angle_count += 0.7
                #self.find_person = False
                #self.take_photo_signal = True

            if result["person_num"] > 1:
                print("person_num =", result["person_num"])
                self.speech_pub.publish("I can see more than one person, I have found the target person")
                self.speech_pub.publish("Please stand in front of me and lead me")
                for i in range(num):
                    person_info = result["person_info"][i]
                    point1 = (person_info["location"]["left"], person_info["location"]["top"])
                    point2 = (person_info["location"]["left"]+person_info["location"]["width"], person_info["location"]["top"]+person_info["location"]["height"])
                    cv2.rectangle(image, point1, point2, (0,0,255), 2)
                    tmpPoint = (point1[0] + point2[0])/2
                    tmpArea = (point1[0] - point2[0]) * (point1[1] - point2[1])
                    point.append(tmpPoint)
                    area.append(tmpArea)
                tmpArea = area[0]
                person_x = point[0]
                for i in range(len(area)):
                    if tmpArea < area[i]:
                        person_x = point[i]
                        tmpArea = area[i]
                        height = result["person_info"][i]["location"]["top"]
                cv2.imwrite(self.path_to_save_result, image)
                #self.find_person = True
                #self.find_person = True
                #msg = String()
                #msg.data = "person_target_found"
                #self.pub_result.publish(msg)

            if result["person_num"] == 1:
                print("person_num =", result["person_num"])
                person_info = result["person_info"][0]
                person_x = person_info["location"]["left"] + person_info["location"]["width"]/2
                person_y = person_info["location"]["top"] + person_info["location"]["height"]/2
                point1 = (person_info["location"]["left"], person_info["location"]["top"])
                point2 = (person_info["location"]["left"]+person_info["location"]["width"], person_info["location"]["top"]+person_info["location"]["height"])
                height = person_info["location"]["top"]
                cv2.rectangle(image, point1, point2, (0,0,255), 2)
                cv2.imwrite(self.path_to_save_result, image)
                #self.find_person = True
        except KeyError:
            print("NO PERSON DETECTED")
            pass

        print("========================")
        print("area =", area)
        print("point =", point)
        print("person_x =", person_x)
        print("height =", height)
        print("========================")
        if person_x-320 > 80:
            self.turn_robot(-0.5)
            print("turn right")
            
        elif person_x -320 < -80:
            self.turn_robot(0.5)
            print("turn left")
            
        else:
            print("Start going forward")
            self.is_stop_turn = True
            self.find_person = True
            '''
            while self.is_go_forward:
                print("go_forward")
                self.forward_robot(0.45)
            self.find_person = True
            rospy.loginfo("Person Target Reached!")
            control_msg = String()
            speech_msg = String()
            control_msg.data = "person_target_found"
            speech_msg.data = "I have reached the target person"
            self.speech_pub.publish(speech_msg)
            self.pub_result.publish(control_msg)
            self.is_stop_turn = False
            '''
            '''
            if (height < 240):
                self.forward_robot(0.5)
                self.find_person = True
                rospy.loginfo("Person Target Reached!")
                control_msg = String()
                speech_msg = String()
                control_msg.data = "person_target_found"
                speech_msg.data = "I have reached the target person"
                self.speech_pub.publish(speech_msg)
                self.pub_result.publish(control_msg) 
            else:
                rospy.loginfo("Person Target Reached!")
                control_msg = String()
                speech_msg = String()
                control_msg.data = "person_target_found"
                speech_msg.data = "I have reached the target person"
                self.speech_pub.publish(speech_msg)
                self.pub_result.publish(control_msg) 
            '''
            


        
            
    def turn_robot(self, angle):
        vel = Twist()
        count = 0
        time = 2
        rate = 10
        r = rospy.Rate(rate)
        num = time*10
        theta = angle/time
        vel.angular.z = theta
        while (count < num):
            count += 1
            self.move_pub.publish(vel)
            r.sleep()
        vel.angular.z = 0.0
        self.move_pub.publish(vel)
        print ("Turning robot")
    
    def forward_robot(self, dis):
        vel = Twist()
        count = 0
        time = 4
        vel.linear.x = dis/time
        rate = 10
        r = rospy.Rate(rate)
        num = time*10
        while (count < num):
            count += 1
            self.move_pub.publish(vel)
            r.sleep()
        
        vel.linear.x = 0.0
        self.move_pub.publish(vel)
        print ("Going forward")
    def cleanup(self):
        #self.sh.say("I have finished help me carry task", self.voice)
        print('[INFO] shut down person_detection node')

if __name__ == '__main__':
    #初始化节点
    rospy.init_node('person_detection')
    print ('----------init----------')
    print ('-----WAITING FOR IMAGE-----')
    detector = person_detection()
    rospy.spin()
        



        
        
