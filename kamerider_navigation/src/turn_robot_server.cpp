#include <ros/ros.h>
#include <string.h>
#include <cmath>
//#include <math.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kamerider_navigation/turn_robotAction.h>

#define PI 3.1415926
float destination_goal;
int i;
int plus = 160;

using namespace std;

class turn_robot_action
{
private:
    float odom_angular;
    ros::Time current_time;
    ros::Time last_time;
    float last_angular;
    float current_angular;
  
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<kamerider_navigation::turn_robotAction> as_;
    //必须先声明NodeHandle 否则会有奇奇怪怪的错误
    //by ROS WIKI

    //action name
    std::string action_name_;

    //Twsit 类型的消息
    geometry_msgs::Twist vel;

    //发布消息控制机器人转动
    ros::Publisher twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
    ros::Publisher is_turn_over_pub = nh_.advertise<std_msgs::Bool>("/is_turn_over", 1);
    //订阅里程计信息
    ros::Subscriber odom_sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &turn_robot_action::OdomCallback, this);

    //声明用来发布 feedback/result 的消息
    kamerider_navigation::turn_robotFeedback feedback_;
    kamerider_navigation::turn_robotResult result_;

public:

    //构造函数
    turn_robot_action(std::string name) :
        current_angular(0.0), last_angular(0.0), odom_angular(0.0), 
        current_time(0.0), last_time(0.0), as_(nh_, name, boost::bind(&turn_robot_action::executeCallback, this, _1), false),
        action_name_(name)
        {
            as_.start();
        }
    //析构函数
    ~turn_robot_action(void) {}

    void OdomCallback(const nav_msgs::Odometry msg)
    {
        if(last_time != current_time)
        {
            //last_time = ros::Time::now();
            //current_time = ros::Time::now();
            
            last_time = current_time;
            current_time = ros::Time::now();
            
            last_angular = current_angular;
            current_angular = msg.twist.twist.angular.z;
            
            float delta_time = (current_time - last_time).toSec();
            float tmp_angular = (current_angular + last_angular)/2;
            
            odom_angular += tmp_angular * delta_time;
            //odom_angular = msg.pose.orientation.z / msg.pose.orientation.w;
            //odom_angular = abs(2 * atan(odom_angular));
        }
    }

    //执行回调函数，在收到来自客户端的请求的时候执行操作
    //将会由客户端传来一个目标转动角度
    void executeCallback (const kamerider_navigation::turn_robotGoalConstPtr& goal)
    {
        //if we receive the command of client, set all the variables to zero
        current_time = ros::Time::now();
        last_time = ros::Time::now();
        odom_angular = 0;
        last_angular = 0;
        current_angular = 0;
        ROS_INFO("%s: Executing, setting goal twist angle %f", action_name_.c_str(), destination_goal);
        //destination_goal = int((goal->goal_angle*180 + plus)/PI);
        if (goal->goal_angle >= 3.14)
        {
            destination_goal = 0.9*(2*PI-goal->goal_angle);
        }
        else
        {
            destination_goal = -0.9*goal->goal_angle;
        }
        ROS_INFO("DESTINATION GOAL IS %f", destination_goal);
        bool success = true;
        double rate = 50;
        //ros::Rate loopRate (rate);

        //开始执行收到的action
        //根据收到的转动目标角度不同使用不同的转动角速度
        if (true)
        //(fabs(goal->goal_angle) < PI/12)
        {
            float angular_speed = PI / 6;
            float turn_duration = destination_goal / angular_speed;
            int ticks = int (turn_duration * rate);

            // check that preempt has not been requested by the client
            //for (int i=0; i<ticks; i++)
            i=0;
            while(true)
            {
                if (as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO ("%s: Preempted", action_name_.c_str());

                    //设置当前action的状态为被抢占
                    as_.setPreempted();
                    success = false;
                    break;
                }

                if (destination_goal > 0)
                {
                    
                    vel.linear.x = 0;
                    vel.linear.y = 0;
                    vel.linear.z = 0;
                    vel.angular.x = 0;
                    vel.angular.y = 0;
                    vel.angular.z = angular_speed;
                    twist_pub_.publish (vel);
                    i++;
                    feedback_.current_angle = i*angular_speed;

                    cout<<"CURRENT GOAL IS "<<odom_angular<<". DESTINATION GOAL IS "<<destination_goal<<endl;
                    if(abs(odom_angular-destination_goal)<0.1)
                    {
                        break;
                    }
                    as_.publishFeedback (feedback_);
                    ros::spinOnce();
                    //loopRate.sleep();
                }

                if (destination_goal < 0)
                {
                    vel.linear.x = 0;
                    vel.linear.y = 0;
                    vel.linear.z = 0;
                    vel.angular.x = 0;
                    vel.angular.y = 0;
                    vel.angular.z = -angular_speed;
                    twist_pub_.publish (vel);
                    i++;
                    feedback_.current_angle = i*(angular_speed);
                    /*if(abs(feedback_.current_angle - destination_goal)<1)
                    {
                      break;
                    }*/
                    cout<<"CURRENT GOAL IS "<<odom_angular<<". DESTINATION GOAL IS "<<destination_goal<<endl;
                    if(abs(odom_angular-destination_goal)<0.1)
                    {
                        break;
                    }
                    as_.publishFeedback (feedback_);
                    ros::spinOnce();
                    //loopRate.sleep();
                }
            }
        }

       

        if (success)
        {
            result_.final_angle = feedback_.current_angle;
            ROS_INFO ("%s: Succeeded", action_name_.c_str());
            std_msgs::Bool is_turn_over_msg;
            is_turn_over_msg.data = true;
            is_turn_over_pub.publish(is_turn_over_msg);
            as_.setSucceeded(result_);
        }

    }

};

int main(int argc, char** argv)
{
    ros::init (argc, argv, "turn_robot_server");
    turn_robot_action turn_robot("turn_robot");
    ros::spin();

    return 0;
}
