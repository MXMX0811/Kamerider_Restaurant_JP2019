/*
Date: 2018/12/20
Author: Xu Yucheng
Abstract: Code for GPSR 当从语音识别出来要去的地点之后导航到指定地点，然后开始寻找待抓取的物体
*/
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>
#include <string.h>
//navigation中需要使用的位姿信息头文件
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
//move_base头文件
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include<actionlib/client/simple_action_client.h>
#include<stdlib.h>
#include<cstdlib>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool ifNavigate = false;
bool is_reach_car = false;
bool is_pub = false;
std::string nav_flag;
std_msgs::String send_flag;
geometry_msgs::Pose goal_pose;

ros::Publisher nav_pub;
ros::Subscriber destination_sub;
ros::Subscriber car_pose_sub;
std::string sub_car_pose_topic_name = "/car_pose";
std::string NAV_PUBLISH_TOPIC = "/is_reach";
std::string sub_destination_topic_name = "/destination";

struct location_pose
{
    string location_name="default";
    float x=0;
    float y=0;
    float z=0;
    float ori_x=0;
    float ori_y=0;
    float ori_z=0;
    float ori_w=1;
};

vector<location_pose> poses;
void set_poses()
{
    ifstream waypoints;
    waypoints.open("/home/keaixin/catkin_ws/src/kamerider_navigation/waypoints.txt", ios::out);
    string data, line;
    vector<string> str; 
    location_pose temp;

    if (waypoints)
    {
        while (getline (waypoints, line))
        {
            stringstream line_data(line);
            str.clear();
            while (getline(line_data, data, ','))
            {
                str.push_back(data);
            }
            cout << "--------pose information--------" << endl;
            for (int i=0; i<str.size(); i++)
            {
                cout << str[i] << " ";
            }
            cout << endl;
            temp.location_name = str[0];
            temp.x = atof(str[1].c_str());
            temp.y = atof(str[2].c_str());
            temp.z = atof(str[3].c_str());
            temp.ori_x = atof(str[4].c_str());
            temp.ori_y = atof(str[5].c_str());
            temp.ori_z = atof(str[6].c_str());
            temp.ori_w = atof(str[7].c_str());
            poses.push_back(temp);
            cout << "location_name: " << str[0] << endl;
            cout << "x: " << atof(str[1].c_str()) << endl;
            cout << "y: " << atof(str[2].c_str()) << endl;
            cout << "z: " << atof(str[3].c_str()) << endl;
            cout << "ori_x: " << atof(str[4].c_str()) << endl;
            cout << "ori_y: " << atof(str[5].c_str()) << endl;
            cout << "ori_z: " << atof(str[6].c_str()) << endl;
            cout << "ori_w: " << atof(str[7].c_str()) << endl;
            cout << "-------------------------------" << endl;
            cout << endl;
            }

    }
    else
    {
        std::cout << "No such waypoints file" << std::endl;
    }
}


void destinationCallback(const std_msgs::String::ConstPtr& msg)
{
    string target_name = msg->data;

    /*cout<<"[INFO] target_name = "<<target_name<<endl;
    if(target_name == "bedroom_door" || target_name == "diningroom_door" 
        || target_name == "livingroom_door" || target_name == "kitchen_door" 
        || target_name == "corridor_door" || target_name == "bathroom_door")
    {
        send_flag.data = "in_position_door";
        ROS_INFO("RECEIVE THE DOOR LOCATION");
    }
    else
    {
        send_flag.data = "in_position";
        ROS_INFO("RECEIVE THE ROOM LOCATION");
    }
    if(target_name == "start")
    {
        send_flag.data = "in_position_start";
        ROS_INFO("GOING TO START POINT");
    }*/
    send_flag.data = "in_position";
    is_reach_car = false;
    ROS_INFO("RECEIVE THE ROOM LOCATION");
    for(int i=0;i<poses.size();i++)
    {
        if(poses[i].location_name == target_name)
        {
            cout << "[NOTICE] NOW I WILL GO TO " << poses[i].location_name << endl;
            goal_pose.position.x = poses[i].x;
            goal_pose.position.y = poses[i].y;
            goal_pose.position.z = poses[i].z;
            goal_pose.orientation.x = poses[i].ori_x;
            goal_pose.orientation.y = poses[i].ori_y;
            goal_pose.orientation.z = poses[i].ori_z;
            goal_pose.orientation.w = poses[i].ori_w;
            ifNavigate = true;
            

        }
    }
}

void carPoseCallback(const geometry_msgs::Pose msg)
{
    goal_pose.position.x = msg.position.x;
    goal_pose.position.y = msg.position.y;
    goal_pose.position.z = msg.position.z;
    goal_pose.orientation.x = msg.orientation.x;
    goal_pose.orientation.y = msg.orientation.y;
    goal_pose.orientation.z = msg.orientation.z;
    goal_pose.orientation.w = msg.orientation.w;
    ifNavigate = true;
    is_reach_car = true;
    ROS_INFO("RECEIVE CAR_POSE");
    cout<<goal_pose.position.x<<" "<<
        goal_pose.position.y<<" "<<
        goal_pose.position.z<<" "<<
        goal_pose.orientation.x<<" "<<
        goal_pose.orientation.y<<" "<<
        goal_pose.orientation.z<<" "<<
        goal_pose.orientation.w<<" "<<endl;

}

int main (int argc, char** argv)
{
    ROS_INFO ("start GPSR navigation");
    ros::init (argc, argv, "gpsr_navigation");
    ros::NodeHandle nh;

    //初始化导航点
    set_poses ();
    nav_pub = nh.advertise<std_msgs::String> (NAV_PUBLISH_TOPIC, 10);
    destination_sub = nh.subscribe (sub_destination_topic_name, 10, destinationCallback);
    car_pose_sub = nh.subscribe(sub_car_pose_topic_name, 10, carPoseCallback);
    //MoveBaseClient mc_("move_base_client", true);
    MoveBaseClient mc_("move_base", true);

    move_base_msgs::MoveBaseGoal nav_goal;

    while (ros::ok())
    {
        if (ifNavigate)
        {
            ROS_INFO ("Start Navigating to Target Position");
            nav_goal.target_pose.header.frame_id = "map";
            nav_goal.target_pose.header.stamp = ros::Time::now();
            nav_goal.target_pose.pose = geometry_msgs::Pose (goal_pose);
            cout<<goal_pose.position.x<<","<<goal_pose.position.y<<","<<goal_pose.position.z<<","
            <<goal_pose.orientation.x<<","<<goal_pose.orientation.y<<","<<goal_pose.orientation.z<<","
            <<goal_pose.orientation.w<<endl;

            while (!mc_.waitForServer (ros::Duration(5.0)))
            {
                ROS_INFO ("Waiting For the Server");
            }
            mc_.sendGoal (nav_goal);
            mc_.waitForResult (ros::Duration (40.0));
            

            if (mc_.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO ("SUCCESSFULLY REACHED %s", nav_flag);
                ifNavigate = false;
                if (is_reach_car)
                {
                    send_flag.data = "in_position_car";
                    nav_pub.publish (send_flag);
                }
                else
                {
                    nav_pub.publish (send_flag);
                }
            }
        }
        ros::spinOnce();
    }
    return EXIT_FAILURE;
}
