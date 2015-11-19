
// General Includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <cmath>

// ROS Includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <ros/package.h>
#include <kdl/frames.hpp>
#include <std_msgs/String.h>

#include "TrackUserNI.h"
#include "NiteSampleUtilities.h"

using namespace std;

class TrackUserNode
{
	ros::Rate rate;
	ros::NodeHandle nh; // Node that will publish messages

	ros::Publisher userCalibratedPub;

	ros::Subscriber startTrackingSub;
	ros::Subscriber stopTrackingSub;

	// openni tracking module
	TrackUserNI trackUserNI;
	std::map<std::string,std::string> map_joints;
	std::string topic_name;
	std::vector<std::string> openni_joints;
	pr2_controllers_msgs::JointTrajectoryGoal goal;
	std::vector<int>  jointIndexes;
	bool isTracking;
	bool isCalibrated;
	actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* ac;
	ros::Time previous = ros::Time::now();


public:
	TrackUserNode(int argc,char **argv, ros::NodeHandle& _nh):rate(30),nh(_nh)
	{
		// create publishers


	  nh.getParam("TrackUserNode/joint_correspondence",map_joints);
	  nh.getParam("TrackUserNode/topic_name",topic_name);


		userCalibratedPub = nh.advertise<std_msgs::String>("userCalibrated",1, true);

		 ac = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(topic_name, true);
		ROS_INFO("Waiting for action server to start.");
		ac->waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// create subscribers
		startTrackingSub = nh.subscribe("startTracking", 1, &TrackUserNode::startTrackingHandle, this);
		stopTrackingSub = nh.subscribe("stopTracking", 1, &TrackUserNode::stopTrackingHandle, this);

		isTracking = false;
		isCalibrated = false;
		openni_joints.resize(22);
		openni_joints[0] = "HeadYaw";
		openni_joints[1] = "HeadPitch";
		openni_joints[2] = "ShoulderPitchL";
		openni_joints[3] = "ShoulderRollL";
		openni_joints[4] = "ShoulderYawL";
		openni_joints[5] = "ElbowRollL";
		openni_joints[6] = "ShoulderPitchR";
		openni_joints[7] = "ShoulderRollR";
		openni_joints[8] = "ShoulderYawR";
		openni_joints[9] = "ElbowRollR";
		openni_joints[10] = "HipYawPitchL";
		openni_joints[11] = "HipRollL";
		openni_joints[12] = "HipPitchL";
		openni_joints[13] = "KneePitchL";
		openni_joints[14] = "AnkleRollL";
		openni_joints[15] = "AnklePitchL";
		openni_joints[16] = "HipYawPitchR";
		openni_joints[17] = "HipRollR";
		openni_joints[18] = "HipPitchR";
		openni_joints[19] = "KneePitchR";
		openni_joints[20] = "AnkleRollR";
		openni_joints[21] = "AnklePitchR";
		mappingJoints();
		trackUserNI.init(argc,argv);
	}

	void stopTrackingHandle(const std_msgs::String::ConstPtr &motionName)
	{
		isTracking = false;
		std::cout<<"stop tracking handle"<<std::endl;
	}

	void startTrackingHandle(const std_msgs::String::ConstPtr &motionName)
	{
		std::cout << "tracking started" << std::endl;
	}

	void mappingJoints(){
	  std::cout<<"I am here"<<std::endl;
	  for(map<string,string>::iterator it = map_joints.begin(); it != map_joints.end(); ++it) {
	        goal.trajectory.joint_names.push_back(it->first);
	        if(it->second.compare("None") != 0){
	          std::size_t i = 0;
	          for (i = 0; i != openni_joints.size(); ++i) {
              //std::cout << *i << std::endl;
              if(it->second.compare(openni_joints[i]) == 0){
                  jointIndexes.push_back(i);
                  break;
              }
            }
	          if(i==openni_joints.size())
	            jointIndexes.push_back(1000);
	        }
	        else{
	          jointIndexes.push_back(1000);
	        }
	        //std::cout << it->second << "\n";
	  }
	  for(std::size_t k=0;k< jointIndexes.size();k++){
	    std::cout<<"JointIndexes "<<jointIndexes[k]<<"\n";
	  }
	  std::cout<<"I am here 2"<<std::endl;

	}

	void run()
	{

		while(nh.ok())
		{
		    ros::Duration dur = ros::Time::now() - previous;
		    previous = ros::Time::now();
		    std::cout<<"TimePassed: "<<dur.toSec()<<std::endl;
			  trackUserNI.waitAndUpdateAll();

				// if calibrated inform subscribers

					//std::cout<<"User Calibrated"<<std::endl;
					// publish calibrated message
					//std_msgs::String calibratedMsg;
					//calibratedMsg.data = "userCalibrated";
					//userCalibratedPub.publish(calibratedMsg);


				// do all processing here
			  int check = trackUserNI.isTrackingUser();
				if (check==5)
				{
				    //std::cout<<"Tracking the user"<<std::endl;
						vector<float> joints = trackUserNI.trackUser();
						goal.trajectory.points.resize(1);
						goal.trajectory.points[0].positions.resize(goal.trajectory.joint_names.size());
						for (std::size_t i = 0; i< jointIndexes.size();i++){
						  if(jointIndexes[i] == 1000)
						    goal.trajectory.points[0].positions[i] = 0.0;
						  else
						    goal.trajectory.points[0].positions[i] = joints[jointIndexes[i]];
						}


						goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
						ac->sendGoalAndWait(goal);

				}
				else if(check==6){
				    //std::cout<<"Can not track the user"<<std::endl;
				}
				else if(check==3){
				     //out of scene
				  //std::cout<<"I am out of SCENEE!!"<<std::endl;
				  //ros::shutdown();

				}

				ros::spinOnce();
				rate.sleep();

		}
	}

};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"nao_trackuser_node");
    ros::NodeHandle nh;

    TrackUserNode trackUserNode(argc,argv, nh);
    trackUserNode.run();


    nite::NiTE::shutdown();
    return 0;
}

