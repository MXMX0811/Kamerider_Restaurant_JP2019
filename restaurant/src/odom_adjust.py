#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
# 此处根据自定义消息类型实际位置进行修改
from kamerider_control_msgs.msg import MoveRobot, Mission, Result

class OdomAdjust(): 
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_adjust', anonymous=False)
        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        # How fast will we update the robot's movement?
        rate = 20
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)
        self._target = None
        # Set the forward linear speed to 0.15 meters per second 
        self.linear_speed = 0.15
        ############### Set the travel distance in meters
        self.goal_distance = 0
        # Set the rotation speed in radians per second
        self.angular_speed = 0.5
        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = radians(1.0)
        # Set the rotation angle to Pi radians (180 degrees)
        # goal_angle = pi
        #Set the required position for arm to grasp
        self.required_pos = Point()
        self.required_pos.x = 0.3756
        self.required_pos.y  = 0.01619
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        # Set the odom frame
        self.odom_frame = '/odom'
        rospy.loginfo("Ready for adjust robot pose by using odom")


        # Publisher to control the robot's speed
        self.pub_result = rospy.Publisher("/odom_to_control", Result, queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
        self.control_sub = rospy.Subscriber("/control_to_odom", MoveRobot, self.control_callback)

    def control_callback(self, msg):       
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        # Initialize the position variable as a Point type
        position = Point()
        # Get the starting position values     
        (position, rotation) = self.get_odom()
        
        if msg.type == "forward":
            # 需要机器人向前移动时
            rospy.loginfo("Received a move forward command")
            self._target = "forward"
            self.go_straight(msg.distance, position)
            self.cmd_vel.publish(Twist())
            self.publish_result()
        
        if msg.type == "turn":
            # 需要转动机器人时
            rospy.loginfo("Receive a turn robot command")
            self._target = "turn"
            self.move_around(msg.angle, rotation)
            self.cmd_vel.publish(Twist())
            self.publish_result()
        
        if msg.type == "observation":
            rospy.loginfo("Move to the observe position")
            self._target = "observation"
            self.move_around(msg.angle, rotation)
            self.cmd_vel.publish(Twist())
            self.go_straight(msg.distance, position)
            self.cmd_vel.publish(Twist())
            self.publish_result()
        
        if msg.type == "customer":
            rospy.loginfo("Move to the customer")
            self._target = "customer"
            self.go_straight(msg.distance, position)
            self.cmd_vel.publish(Twist())
            self.publish_result()

        rospy.sleep(1)
    
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
    def go_straight(self,goal_distance,start_pos):
        move_cmd = Twist()
                
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        
        if goal_distance < 0:
            move_cmd.linear.x = -self.linear_speed
            goal_distance = -goal_distance
                    
        x_start = start_pos.x
        y_start = start_pos.y
        
        # Keep track of the distance traveled
        traveled_distance = 0
        
        # Enter the loop to move along a side
        while traveled_distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            self.r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()
            
            # Compute the Euclidean distance from the start
            traveled_distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

        # Stop the robot 
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        print("go {} meters long end!".format(goal_distance))
        return


    def move_around(self,goal_radius,start_rotation):
        move_cmd = Twist()

        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed
        if goal_radius < 0:
            move_cmd.angular.z = -self.angular_speed
            goal_radius = -goal_radius

        # Track the last angle measured
        last_angle = start_rotation
        
        # Track how far we have turned
        turn_angle = 0
        
        while abs(turn_angle + self.angular_tolerance) < abs(goal_radius) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)
            
            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
            
        # Stop the robot 
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        print("Turn {} radius  end!".format(goal_radius))
    
    def publish_result(self):
        res = Result()
        res.mission_type = self._target
        res.result = "success"
        self.pub_result.publish(res)
    

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':    
    OdomAdjust()
    rospy.spin()
