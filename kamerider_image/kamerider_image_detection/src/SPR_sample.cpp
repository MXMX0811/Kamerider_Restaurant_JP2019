//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//navigation中需要使用的位姿信息头文件
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
//c++
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <random>
#include <time.h>
//opencv
#include<opencv2/opencv.hpp>
//kamerider_image_msgs
#include <kamerider_image_msgs/FaceDetection.h>
#include <kamerider_image_msgs/GenderDetection.h>
#include <kamerider_image_msgs/BoundingBox.h>
//std_msgs
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
//kamerider_navigation
#include <kamerider_navigation/turn_robotAction.h>
//move_base头文件
#include<move_base_msgs/MoveBaseGoal.h>
#include<move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include<actionlib/client/simple_action_client.h>
#include<cstdlib>
//soundplay
#include <sound_play/sound_play.h>

using namespace std;
using namespace cv;

//导航client
typedef actionlib::SimpleActionClient<kamerider_navigation::turn_robotAction> Client; //turn robot
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //move robot

class SPR
{
private:
  enum Step
  {
    PreStart,
    Start,
    Turn,
    Detect_Face_Gender,
    Play_Riddle,
    Circling_Crowd,
    Leave,
    Finish,
  };

  int step;
  const float PI = 3.1415926;
  bool is_receive_gender_detect_result;
  bool is_pub_take_photo_signal;
  bool is_pub_turn;
  bool is_turn_over;
  bool is_riddle_half_finish;
  bool is_riddle_finish;
  bool is_pub_riddle_start;
  bool is_first_circling_crowd;
  bool is_random_turn;
  float total_angle;
  bool is_pub_gender_result;
  bool is_finish_record;
  geometry_msgs::Pose goal_pose;//目标位置
  kamerider_navigation::turn_robotGoal goal;//声源角度
  kamerider_navigation::turn_robotGoal last_goal;

  //kamerider_image_msgs::FaceDetection face_detection_msg;
  kamerider_image_msgs::GenderDetection gender_detection_msg;

  void init()
  {
    //如果有导航，将start改为prestart
    step = Start;
    is_receive_gender_detect_result = false;
    is_pub_take_photo_signal = false;
    is_pub_turn = false;
    is_turn_over = false;
    is_riddle_half_finish = false;
    is_pub_riddle_start = false;
    is_riddle_finish = false;
    is_first_circling_crowd = true;
    is_random_turn = false;
    total_angle = 0;
    is_pub_gender_result = false;
    is_finish_record = false;
  }

  void gender_detection_callback(const kamerider_image_msgs::GenderDetection msg)
  {
    ROS_INFO("RECEIVE THE GENDER DETECTION RESULTS");
    gender_detection_msg.male_num = msg.male_num;
    gender_detection_msg.female_num = msg.female_num;
    gender_detection_msg.sit_num = msg.sit_num;
    gender_detection_msg.stand_num = msg.stand_num;
    is_receive_gender_detect_result = true;
  }

  void is_turn_over_callback(const std_msgs::Bool msg)
  {
    is_turn_over = msg.data;
  }

  void riddle_finish_callback(const std_msgs::String msg)
  {
    if(msg.data == "riddle_half")
    {
      is_riddle_half_finish = true;
    }
    if(msg.data == "riddle_finish")
    {
      is_riddle_finish = true;
    }
  }

  void locateSoundCallback(const std_msgs::Float32 msg)
  {
    if (step == Circling_Crowd)
    {
      /*
      if(msg.data>180||msg.data==180)
      {
        goal.goal_angle = 2*PI - PI * msg.data/180;
      }
      else
      {
        goal.goal_angle = -PI*msg.data/180;
      }
      cout<<"[INFO] Receive the angle: "<<goal.goal_angle<<endl;
      is_random_turn = true;
      is_finish_record = false;
      */
      goal.goal_angle = PI*msg.data/180;
      cout<<"[INFO] Receive the angle: "<<goal.goal_angle<<endl;
      is_random_turn = true;
      is_finish_record = false;
    }
  }

  void publish_take_photo_signal(ros::Publisher &publisher)
  {
    ROS_INFO("PUBLISH TAKE_PHOTO_SIGNAL");
    std_msgs::String is_take_photo_msg;
    is_take_photo_msg.data = "take_photo";
    publisher.publish(is_take_photo_msg);
  }

  void recordCallback(const std_msgs::String msg)
  {
    if(msg.data == "record_finish")
    {
      is_finish_record = true;
    }
  }


public:
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                     const kamerider_navigation::turn_robotResultConstPtr& result)
  {
      ROS_INFO ("I have turned %f degree, waiting for your further command", result->final_angle);
      //is_turn_over = true;
  }
  void activeCallback ()
  {
      ROS_INFO ("Goal actived");
  }
  void feedbackCallback (const kamerider_navigation::turn_robotFeedbackConstPtr& feedback)
  {
      ROS_INFO ("current angle is %f", feedback->current_angle);
  }

  int run(int argc, char **argv)
  {
    ros::init(argc, argv, "SPR_sample");

    ros::NodeHandle node;
    //语音客户端
    sound_play::SoundClient sound_client;

    //advertise topics
    string pub_take_photo_signal_topic_name;
    string pub_image_topic_name;
    string gender_recognition_result;
    //string pub_riddle_start_topic_name;

    //subscribe topics
    string sub_gender_recognition_topic_name;
    string sub_riddle_finish_topic_name;
    string sub_turn_over_topic_name;
    string sub_locate_sound_topic_name;
    string sub_record_finish_topic_name;

    node.param<string>("pub_take_photo_signal_topic_name",        pub_take_photo_signal_topic_name,   "/take_photo_signal");
    //node.param<string>("pub_riddle_start_topic_name",   pub_riddle_start_topic_name,  "/riddle_start");
    node.param<string>("sub_gender_recognition_topic_name",       sub_gender_recognition_topic_name,  "/kamerider_image/gender_recognition");
    node.param<string>("sub_riddle_finish_topic_name",            sub_riddle_finish_topic_name,       "/riddle_finish");
    node.param<string>("sub_turn_over_topic_name",                sub_turn_over_topic_name,           "/is_turn_over");
    node.param<string>("sub_locate_sound_topic_name",             sub_locate_sound_topic_name,        "/sound_angular");
    node.param<string>("pub_image_topic_name",                    pub_image_topic_name,               "/image");
    node.param<string>("gender_recognition_result",               gender_recognition_result,          "/gender_recognition_result");
    node.param<string>("sub_record_finish_topic_name",            sub_record_finish_topic_name,       "/record_finish");
    ros::Publisher take_photo_signal    = node.advertise<std_msgs::String>(pub_take_photo_signal_topic_name, 10);
    //ros::Publisher riddle_start         = node.advertise<std_msgs::String>(pub_riddle_start_topic_name, 1);
    ros::Publisher pub_image                = node.advertise<sensor_msgs::Image>(pub_image_topic_name, 10);
    ros::Publisher pub_gender_recognition_result = node.advertise<kamerider_image_msgs::GenderDetection>(gender_recognition_result, 1);
    
    ros::Subscriber is_turn_over_signal = node.subscribe<std_msgs::Bool>(sub_turn_over_topic_name, 1, &SPR::is_turn_over_callback, this);
    ros::Subscriber gender_detection    = node.subscribe<kamerider_image_msgs::GenderDetection>(sub_gender_recognition_topic_name, 10, &SPR::gender_detection_callback, this);
    ros::Subscriber riddle_finish       = node.subscribe<std_msgs::String>(sub_riddle_finish_topic_name, 10, &SPR::riddle_finish_callback, this);
    ros::Subscriber sound_angular       = node.subscribe<std_msgs::Float32>(sub_locate_sound_topic_name, 1, &SPR::locateSoundCallback, this);
    ros::Subscriber record_finish       = node.subscribe<std_msgs::String>(sub_record_finish_topic_name, 1, &SPR::recordCallback, this);
    init();

    while(ros::ok())
    {
      switch(step)
      {
        //如果需要机器人自己进场
        case PreStart:
        {
          MoveBaseClient  mc_("move_base", true); //建立导航客户端
	        move_base_msgs::MoveBaseGoal naviGoal; //导航目标点
          // the start position
          goal_pose.position.x = 10.8219;
          goal_pose.position.y = 6.91687;
          goal_pose.position.z = 0;
          goal_pose.orientation.x = 0;
          goal_pose.orientation.y = 0;
          goal_pose.orientation.z = 0.764717;
          goal_pose.orientation.w = 0.644366;
          naviGoal.target_pose.header.frame_id = "map"; 
          naviGoal.target_pose.header.stamp = ros::Time::now();
          naviGoal.target_pose.pose = geometry_msgs::Pose(goal_pose);
          while(!mc_.waitForServer(ros::Duration(5.0)))
          {
            //等待服务初始化
            ROS_INFO("Waiting for the server");
          }
          mc_.sendGoal(naviGoal);
          mc_.waitForResult(ros::Duration(60.0));
          //导航反馈直至到达目标点      
          if(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Yes! The robot has moved to the start point");
            step++;
          }
          break;
        }
        case Start:
        {
          
          ros::topic::waitForMessage<std_msgs::String>("/start",node);
          	
          cout<<"======================================="<<endl;
          ROS_INFO("START SPEECH AND PERSON RECOGNITION");
          ros::Duration(1).sleep();
          sound_client.say("Hello I wanna play riddles");
          ros::Duration(3).sleep();
          sound_client.say("I will turn back after ten seconds");
          ros::Duration(3).sleep();
          ros::Duration(5).sleep();
          //want to play riddle
          //step++;
          step = step + 1;
          break;
        }
        case Turn:
        {
          if(!is_pub_turn)
          {
            cout<<"======================================="<<endl;
            ROS_INFO("READY TO TURN");
            Client client("turn_robot", true);
            ROS_INFO ("Waiting for action server to start");
            client.waitForServer();
            ROS_INFO ("Action server actived, start sending goal");

            kamerider_navigation::turn_robotGoal goal;
            goal.goal_angle = PI;
            //client.sendGoal(goal, &SPR::doneCallback, &SPR::activeCallback, &SPR::feedbackCallback, this);
            client.sendGoal(goal);
            is_pub_turn = true;
          }
          if(is_turn_over)
          {
            step++;
            is_turn_over = false;
            ros::Duration(5).sleep();
          }
          break;
          //ros::spinOnce();
        }
        case Detect_Face_Gender:
        {
          //is_take_photo = true;
          if(is_receive_gender_detect_result)
          {
            cout<<"======================================="<<endl;

            int male_num = gender_detection_msg.male_num;
            int female_num = gender_detection_msg.female_num;
            int sit_num = gender_detection_msg.sit_num;
            int stand_num = gender_detection_msg.stand_num;
            int face_num = male_num + female_num;
            ROS_INFO("face_num = %d", face_num);
            ROS_INFO("male_num = %d", male_num);
            ROS_INFO("female_num = %d", female_num);
            ROS_INFO("sit_num = %d", sit_num);
            ROS_INFO("stand_num = %d", stand_num);

            ros::Duration(3).sleep();
            sound_client.say("I have taken a picture");
            ros::Duration(2.5).sleep();
            sound_client.say("Here are the results");
            ros::Duration(2.5).sleep();

            string str = "The human number is " + to_string(face_num);
            sound_client.say(str);
            ros::Duration(3).sleep();

            str = "The male number is " + to_string(male_num);
            sound_client.say(str);
            ros::Duration(2.5).sleep();

            str = "The female number is " + to_string(female_num);
            sound_client.say(str);
            ros::Duration(2.5).sleep();

            str = "The number of sitting persons is " + to_string(sit_num);
            sound_client.say(str);
            ros::Duration(3.5).sleep();

            str = "The number of standing persons is " + to_string(stand_num);
            sound_client.say(str);
            ros::Duration(3).sleep();

            //is_receive_gender_detect_result = false;
            //speak
            step++;
          }
          else
          {  
            VideoCapture cap(1);
            if(!cap.isOpened())
            {
                ROS_ERROR("FAILED TO OPEN CAPTURE");
            }
            Mat frame;
            ROS_INFO("open the webcam");
            while (!is_receive_gender_detect_result)
            {
              cap>>frame;
              imshow("frame", frame);
              waitKey(1);
              sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
              pub_image.publish(msg);
              publish_take_photo_signal(take_photo_signal);
              ros::spinOnce();
            }
            publish_take_photo_signal(take_photo_signal);
          }
          break;
        }
        case Play_Riddle:
        {
          if(!is_pub_riddle_start)
          {
            cout<<"======================================="<<endl;
            cout<<"START PLAYING RIDDLES"<<endl;
            system("rosnode kill /gender_recognition");
            ROS_INFO("KILL gender_recognition NODE");
            ros::Duration(3).sleep();
            /*sound_client.say("who wants to play riddles with me");
            ros::Duration(3.5).sleep();
            sound_client.say("please stand in front of me");
            ros::Duration(4).sleep();*/
            sound_client.say("Hello my name is Jack");
            ros::Duration(3).sleep();
            sound_client.say("before each question please say hi Jack to me ");
            ros::Duration(5).sleep();
            //sound_client.say("I am ready for your command if you hear");
            //ros::Duration(4).sleep();
            //sound_client.playWave("/home/keaixin/catkin_ws/src/kamerider_speech/sounds/question_start_signal.wav");
            system("gnome-terminal -x bash -c \"roslaunch kamerider_speech speech_all.launch && read\"");
            ros::Duration(2).sleep();
            system("gnome-terminal -x bash -c \"rosrun kamerider_speech locate_sound.py && read\"");
            ros::Duration(2).sleep();
            sound_client.say("Now you can ask me questions");
            ROS_INFO("FINISH PUB_RIDDLE_START");
            is_pub_riddle_start = true;
          }
          if(!is_pub_gender_result)
          {
            pub_gender_recognition_result.publish(gender_detection_msg);
            is_pub_gender_result = true;
          }
          //cout<<"===================="<<endl;
          //cout<<is_riddle_half_finish<<endl;
          //cout<<"===================="<<endl;
          if(is_riddle_half_finish)
          {
            step++;
          }
          break;
        }
        case Circling_Crowd:
        {
          //std::cout<<"-------------------------------";
          if(is_first_circling_crowd)
          {
            cout<<"======================================="<<endl;
            cout<<"START PLAYING RIDDLES"<<endl;
            sound_client.say("Please stand around me and wait 5 second");
            ros::Duration(4).sleep();
            //system("gnome-terminal -x bash -c \"rosrun kamerider_speech locate_sound.py && read\"");

            //kill sound_play node
            //system("rosnode kill /sound_play");
            //ros::Duration(2).sleep();
            //system("rosnode kill /get_audio");
            //ros::Duration(2).sleep();
            //ROS_INFO("KILL sound_play NODE");
            //ROS_INFO("KILL get_audio NODE");
            //ros::Duration(1).sleep();
            //system("gnome-terminal -x bash -c \"rosrun sound_play soundplay_node.py && read\"");
            //ROS_INFO("RESTART THE sound_play NODE");
            //ROS_INFO("RESTART THE get_audio NODE");
            //ros::Duration(2).sleep();
            sound_client.say("now you can ask the rest of the questions");
            ros::Duration(4).sleep();
            is_first_circling_crowd = false;
          }
          if(is_random_turn)
          {
            ROS_INFO("READY TO TURN");
            Client client("turn_robot", true);
            ROS_INFO ("Waiting for action server to start");
            client.waitForServer();
            ROS_INFO ("Action server actived, start sending goal");

            srand((int)time(0));
            total_angle += goal.goal_angle;
            client.sendGoal(goal);
            is_random_turn = false;
          }
          if(is_riddle_finish)
          {
            //step++;
            system("rosnode kill /locate_sound");
            step = step + 1;
          }
          break;
        }
        case Leave:
        {
          cout<<"======================================="<<endl;
          cout<<"SUCCESSFULLY FINISH ALL THE TASKS"<<endl;
          cout<<"READY TO LEAVE"<<endl;
          //system("gnome-terminal -x bash -c \"roslaunch turtlebot_rviz_launchers view_navigation.launch && read\"");
          //ros::Duration(5).sleep();

          sound_client.say("You have asked me ten questions");
          ros::Duration(3).sleep();
          sound_client.say("I will leave right away");
          //system("gnome-terminal -x bash -c \"roslaunch kamerider_image_detection SPR_navigation.launch && read\"");
          ros::Duration(5).sleep();
          
          MoveBaseClient  mc_("move_base", true); //建立导航客户端
	        move_base_msgs::MoveBaseGoal naviGoal; //导航目标点
          // the exit position
          goal_pose.position.x = -3.0254;
          goal_pose.position.y = 6.48455;
          goal_pose.position.z = 0;
          goal_pose.orientation.x = 0;
          goal_pose.orientation.y = 0;
          goal_pose.orientation.z = -0.963965;
          goal_pose.orientation.w = 0.266029;
          naviGoal.target_pose.header.frame_id = "map"; 
          naviGoal.target_pose.header.stamp = ros::Time::now();
          naviGoal.target_pose.pose = geometry_msgs::Pose(goal_pose);
          while(!mc_.waitForServer(ros::Duration(5.0)))
          {
            //等待服务初始化
            ROS_INFO("Waiting for the server");
          }
          mc_.sendGoal(naviGoal);
          mc_.waitForResult(ros::Duration(60.0));
          //导航反馈直至到达目标点      
          if(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Yes! The robot has moved to the exit");
            step++;
          }
          break;
        }
        case Finish:
        {
          ROS_INFO("I HAVE LEAVE THE AREA SUCCESSFULLY");
          return EXIT_SUCCESS;
        }
      }//end switch
      ros::spinOnce();
    }//end while
    return EXIT_SUCCESS;

  }//end run
};



int main(int argc, char **argv)
{
  //ros::init(argc, argv, "Speech_and_Person_Recognition");
  //ros::NodeHandle SPR_node;
  SPR SPR_sample;
  return SPR_sample.run(argc, argv);
}
