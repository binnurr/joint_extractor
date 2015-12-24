
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

using namespace std;

class TrackUserNode
{
	ros::Rate rate;
	ros::NodeHandle nh; // Node that will publish messages

	ros::Subscriber startTrackingSub;
	ros::Subscriber stopTrackingSub;

	bool isTracking;
	map<string,string> map_joints;
	string topic_name;
	map<string,string> jointMinVals;
	map<string,string> jointMaxVals;
	vector<string> openni_joints;
	vector<float> minLimits;
	vector<float> maxLimits;
	pr2_controllers_msgs::JointTrajectoryGoal goal;
	vector<int>  jointIndexes;
	ros::Time previous = ros::Time::now();
	actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* ac;
	TrackUserNI trackUserNI;


public:
	TrackUserNode(int argc,char **argv, ros::NodeHandle& _nh):rate(30),nh(_nh)
	{
	  nh.getParam("TrackUserNode/joint_correspondence",map_joints);
	  nh.getParam("TrackUserNode/topic_name",topic_name);
	  nh.getParam("TrackUserNode/joint_minValues",jointMinVals);
	  nh.getParam("TrackUserNode/joint_maxValues",jointMaxVals);

		ac = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(topic_name, true);
		ROS_INFO("Waiting for action server to start.");
		ac->waitForServer(); //will wait for infinite time
		ROS_INFO("Action server started, sending goal.");

		// create subscribers
		startTrackingSub = nh.subscribe("startTracking", 1, &TrackUserNode::startTrackingHandle, this);
		stopTrackingSub = nh.subscribe("stopTracking", 1, &TrackUserNode::stopTrackingHandle, this);

		isTracking = false;
		openni_joints.resize(trackUserNI.numJoints);
		minLimits.resize(trackUserNI.numJoints);
		maxLimits.resize(trackUserNI.numJoints);
		openni_joints[0] = "HeadPitch";
		openni_joints[1] = "HeadRoll";
		openni_joints[2] = "ShoulderPitchL";
		openni_joints[3] = "ShoulderRollL";
		openni_joints[4] = "ShoulderYawL";
		openni_joints[5] = "ElbowRollL";
		openni_joints[6] = "ShoulderPitchR";
		openni_joints[7] = "ShoulderRollR";
		openni_joints[8] = "ShoulderYawR";
		openni_joints[9] = "ElbowRollR";
		openni_joints[10] = "HipPitchL";
		openni_joints[11] = "HipRollL";
		openni_joints[12] = "KneePitchL";
		openni_joints[13] = "HipPitchR";
		openni_joints[14] = "HipRollR";
		openni_joints[15] = "KneePitchR";
		openni_joints[16] = "TorsoBending";

		mappingJoints();
		trackUserNI.init(argc,argv,&minLimits,&maxLimits);
	}

	void stopTrackingHandle(const std_msgs::String::ConstPtr &motionName)
	{
		isTracking = false;
		ROS_INFO("Tracking Stopped !!");
	}

	void startTrackingHandle(const std_msgs::String::ConstPtr &motionName)
	{
	  ROS_INFO("Tracking Started !!");
	}

	void mappingJoints(){
	  map<string,string>::const_iterator corrIter ;
	  map<string,string>::const_iterator minIter ;
	  map<string,string>::const_iterator maxIter ;

	  for(corrIter = map_joints.begin(),minIter = jointMinVals.begin(),maxIter = jointMaxVals.begin() ; corrIter != map_joints.end() && minIter != jointMinVals.end() && maxIter != jointMaxVals.end();
	      ++corrIter, ++minIter, ++maxIter) {
	    goal.trajectory.joint_names.push_back(corrIter->first);
	    std::cout<<"this one: "<<corrIter->first<<std::endl;
	    if(corrIter->second.compare("None") != 0){
	      std::size_t i = 0;
	      for (i = 0; i != openni_joints.size(); ++i) {
	        if(corrIter->second.compare(openni_joints[i]) == 0){
	          jointIndexes.push_back(i);
	          minLimits[i] = atof(minIter->second.c_str());
	          maxLimits[i] = atof(maxIter->second.c_str());
	          break;
	        }
	      }
	      if(i==openni_joints.size())
	        jointIndexes.push_back(1000);
	    }
	    else{
	      jointIndexes.push_back(1000);
	    }
	  }
	}

	void run()
	{

		while(nh.ok())
		{
//		    ros::Duration dur = ros::Time::now() - previous;
//		    previous = ros::Time::now();
//		    std::cout<<"TimePassed: "<<dur.toSec()<<std::endl;
			  trackUserNI.waitAndUpdateAll();

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

