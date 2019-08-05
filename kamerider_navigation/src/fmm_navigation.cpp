/*
Date: 2019/5/21
Author: Guo Zijian
Abstract: 
    Subscribe:  /destination
    Publish:    /is_reach
    get the destination pose and publish /is_reach message when reach the destination
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
bool is_finish = false;
bool is_get_pose = false;
std::string nav_flag;
std_msgs::String send_flag;
geometry_msgs::Pose goal_pose;

ros::Publisher nav_pub;
ros::Subscriber destination_sub;
ros::Subscriber person_pos_sub;
std::string NAV_PUBLISH_TOPIC = "/is_reach";
std::string sub_destination_topic_name = "/destination";
std::string sub_person_pos_topic_name = "/person_pos";

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
    waypoints.open("~/catkin_ws/src/kamerider_navigation/waypoints.txt", ios::out);
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
    cout<<"[INFO] target_name = "<<target_name<<endl;

    send_flag.data = "in_position";
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
            is_get_pose = true;
        }
    }
    if (!is_get_pose)
    {
        send_flag.data = "failure";
        ROS_INFO("RETURN TO START POINT");
        ifNavigate = false;
        nav_pub.publish (send_flag);
    }
}

void personPosCallback(const geometry_msgs::Pose msg)
{
    ifNavigate = true;
    send_flag.data = "in_position_person";
    goal_pose.position.x = msg.position.x;
    goal_pose.position.y = msg.position.y;
    goal_pose.position.z = msg.position.z;
    goal_pose.orientation.x = msg.orientation.x;
    goal_pose.orientation.y = msg.orientation.y;
    goal_pose.orientation.z = msg.orientation.z;
    goal_pose.orientation.w = msg.orientation.w;
}

int main (int argc, char** argv)
{
    ROS_INFO ("start GPSR navigation");
    ros::init (argc, argv, "fmm_navigation");
    ros::NodeHandle nh;

    //初始化导航点
    set_poses ();
    nav_pub = nh.advertise<std_msgs::String> (NAV_PUBLISH_TOPIC, 10);
    destination_sub = nh.subscribe (sub_destination_topic_name, 10, destinationCallback);
    person_pos_sub = nh.subscribe (sub_person_pos_topic_name, 10, personPosCallback);
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
                nav_pub.publish (send_flag);
            }
        }
        ros::spinOnce();
    }
    return EXIT_FAILURE;
}
