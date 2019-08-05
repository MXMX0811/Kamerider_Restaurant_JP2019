#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os

os.system('gnome-terminal -x bash -c "roslaunch restaurant speech_all_res.launch"')
os.system('sleep 3')
os.system('gnome-terminal -x bash -c "roslaunch restaurant res_navigation.launch"')
os.system('sleep 3')
