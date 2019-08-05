#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import rospy
import roslib
import base64
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32, String

class locate_sound:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # 打开端口
        #端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
        self.port="/dev/ttyUSB0"
        #波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
        self.bps=115200
        #超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
        self.timex=2
        # 打开串口，并得到串口对象
        """
        port_list = list(serial.tools.list_ports.comports())
        for port in port_list:
            
            port = port[:12]
            print(port)
            self.port = port
            try:
                self.serial = serial.Serial(port=self.port, baudrate=self.bps, bytesize=8, parity='N', stopbits=1, timeout=self.timex)
            except AttributeError:
                continue
        """
        self.serial = serial.Serial(port=self.port, baudrate=self.bps, bytesize=8, parity='N', stopbits=1, timeout=self.timex)
        print('[INFO] Port Parameters: ', self.serial)
        #ros param
        self.angular = 0.0
        self.count = 0
        self.pub_sound_angular_topic_name=None
        self.pub_kws_data_topic_name = None
        self.get_params()
        while True:
            #不能重复问问题：self.count == 5，可以：self.count == 10
            #if self.count == 10:
            #    break
            #elif self.serial.in_waiting:
            #try:
            data = self.serial.readline(self.serial.in_waiting )
            #data = str(data)
            #json.dumps(data).decode('unicode-escape')
            try:
                json.dumps(data).decode('unicode-escape')
            except UnicodeDecodeError:
                json.dumps(data).decode('utf-8')
            
            if 'angle' in data:
                
                self.angular = data[15:18]
                result = ''
                for i in range(len(self.angular)):
                    if self.angular[i] != ' ':
                        result += self.angular[i]
                    else:
                        break
                #print(result)
                self.angular = float(result)
                print('=================NO.%s================='%(self.count+1))
                print('[INFO] Receive Data: ', self.angular)
                angular_msg = Float32()
                angular_msg.data = self.angular
                jack_msg = String()
                jack_msg.data='jack '
                self.pub_sound_angular.publish(angular_msg)
                self.pub_kws_data.publish(jack_msg)
                self.count += 1
            #except UnicodeDecodeError:
            #    pass
        print('[INFO] Stop locate sound')
        self.serial.close()


    def get_params(self):
        self.pub_sound_angular_topic_name = rospy.get_param('pub_sound_angular_topic_name', '/sound_angular')
        self.pub_kws_data_topic_name = rospy.get_param('pub_kws_data_topic_name', '/kws_data')
        self.pub_sound_angular = rospy.Publisher(self.pub_sound_angular_topic_name, Float32, queue_size=1)
        self.pub_kws_data = rospy.Publisher(self.pub_kws_data_topic_name, String, queue_size=1)

    def cleanup(self):
        self.serial.close()
        print('[INFO] Close the port')

if __name__ == '__main__':
    #初始化节点
    rospy.init_node('locate_sound')
    locate_sound()
    rospy.spin()
