/*
 * TrackUserNI.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: binnur
 */

#include "TrackUserNI.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <math.h>

#include <ros/ros.h>

using namespace std;

static const float PI = 3.1415;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
  {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

TrackUserNI::TrackUserNI()
{
	//initiation for angles
	shoulderPitch=0.0;
	shoulderRoll=0.0;
	elbowYaw=0.0;
	elbowRoll=0.0;
	shoulderPitchLeft=0.0;
	shoulderRollLeft=0.0;
	elbowYawLeft=0.0;
	elbowRollLeft=0.0;
	head_front=0.0;
	head_side=0.0;
	anglebetweenelLeft=0.0;
	anglebetweenel=0.0;
	hip=0.0;
	hip_angle=0.0;

	rightanklepitch = 0.0;
	leftanklepitch = 0.0;

	rshoulderXZ=0.0;
	rshoulderYZ=0.0;
	rshoulderXY=0.0;
	lshoulderXZ=0.0;
	lshoulderYZ=0.0;
	lshoulderXY=0.0;
	relbowXZ=0.0;
	relbowYZ=0.0;
	relbowXY=0.0;
	lelbowXZ=0.0;
	lelbowYZ=0.0;
	lelbowXY=0.0;
	rwristXZ=0.0;
	rwristYZ=0.0;
	rwristXY=0.0;
	lwristXZ=0.0;
	lwristYZ=0.0;
	lwristXY=0.0;

	hiptokneeright_XY=0.0;
	hiptokneeright_YZ=0.0;
	hiptokneeright_XZ=0.0;
	kneetoankleright_XY=0.0;
	kneetoankleright_YZ=0.0;
	kneetoankleright_XZ=0.0;
	betweenknee=0.0;

	Lhiptokneeright_XY=0.0;
	Lhiptokneeright_YZ=0.0;
	Lhiptokneeright_XZ=0.0;
	Lkneetoankleright_XY=0.0;
	Lkneetoankleright_YZ=0.0;
	Lkneetoankleright_XZ=0.0;
	Lbetweenknee=0.0;
	jointarray = new Vector3D[17];
	jointvectorarray = new Vector3D[12];
}

TrackUserNI::~TrackUserNI()
{

}

void TrackUserNI::init(int argc, char **argv)
{

  nite::NiTE::initialize();
  if (argc > 1)
  {
    const char* deviceURI = openni::ANY_DEVICE;
    std::cout<<"Argument is available"<<std::endl;
    deviceURI = argv[1];
    openniRc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());


    openniRc = device.open(deviceURI);
    device.getPlaybackControl()->setSpeed(-1);
    if (openniRc != openni::STATUS_OK)
    {
      printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
    }


    niteRc = userTracker.create(&device);
    if (niteRc != nite::STATUS_OK)
    {
      printf("Couldn't create user tracker\n");

    }

  }
  else
  {
    niteRc = userTracker.create();
    if (niteRc != nite::STATUS_OK)
    {
      printf("Couldn't create user tracker \n");

    }

  }
  printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

}

void TrackUserNI::waitAndUpdateAll()
{
	niteRc = userTracker.readFrame(&userTrackerFrame);
	if (niteRc != nite::STATUS_OK)
	{
		printf("Get next frame failed\n");
	}
}

int TrackUserNI::updateUserState(const nite::UserData& user, unsigned long long ts)
{
  int check = 0;
  if (user.isNew())
    USER_MESSAGE("New")
  else if (user.isVisible() && !g_visibleUsers[user.getId()])
    USER_MESSAGE("Visible")
  else if (!user.isVisible() && g_visibleUsers[user.getId()]){
    USER_MESSAGE("Out of Scene")
    check = 5;
  }
  else if (user.isLost())
    USER_MESSAGE("Lost")

  g_visibleUsers[user.getId()] = user.isVisible();


  if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
  {
    switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
    {
    case nite::SKELETON_NONE:
      USER_MESSAGE("Stopped tracking.")
      break;
    case nite::SKELETON_CALIBRATING:
      USER_MESSAGE("Calibrating...")
      break;
    case nite::SKELETON_TRACKED:
      USER_MESSAGE("Tracking!")
      break;
    case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
    case nite::SKELETON_CALIBRATION_ERROR_HANDS:
    case nite::SKELETON_CALIBRATION_ERROR_LEGS:
    case nite::SKELETON_CALIBRATION_ERROR_HEAD:
    case nite::SKELETON_CALIBRATION_ERROR_TORSO:
      USER_MESSAGE("Calibration Failed... :-|")
      break;
    }
  }
  return check;
}

int TrackUserNI::isTrackingUser()
{
  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	bool check=false;
//	std::cout << "isTrackingUser user.size=" << users.getSize() << std::endl;
	for (int i = 0; i < users.getSize(); ++i) {

		const nite::UserData& user = users[i];
		if(updateUserState(user,userTrackerFrame.getTimestamp())== 5){
		  return 3;
		}
		if (user.isNew())
		{
			userTracker.startSkeletonTracking(user.getId());
			return 4;
		}

		else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED){
			return 5;
		}
		else
			return 6;
	}
	//return check;
}

void TrackUserNI::trackUser2(){
  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
  for (int i = 0; i < users.getSize(); ++i){
      const nite::UserData& user = users[i];

      if (user.getSkeleton().getState() != nite::SKELETON_TRACKED)
        continue;

      head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
      neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
      righthip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
      rightknee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
      rightankle = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
      rightfoot = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
      lefthip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
      leftknee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
      leftankle = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
      leftfoot = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
      rightshoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
      rightelbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
      righthand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
      leftshoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
      leftelbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
      lefthand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
      userPoint = user.getSkeleton().getJoint(nite::JOINT_TORSO);

      if(leftshoulder.getPositionConfidence()> .5){
        //printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
        //printf("%d. (%5.2f, %5.2f, %5.2f, %5.2f)\n", user.getId(), head.getOrientation().w, head.getOrientation().x, head.getOrientation().y,head.getOrientation().z);
       float q0 = leftshoulder.getOrientation().w;
       float q1 = leftshoulder.getOrientation().x;
       float q2 = leftshoulder.getOrientation().y;
       float q3 = leftshoulder.getOrientation().z;
       float alpha = atan( (2*(q0*q1+q2*q3))/(1-2*(q1*q1+q2*q2)));
       float beta = asin(2*(q0*q2-q3*q1));
       float gamma = atan( (2*(q0*q3+q1*q2))/(1-2*(q2*q2+q3*q3)));

       printf("%.2f %.2f %.2f \n", (alpha*90)/1.57, (beta*90)/1.57, (gamma*90)/1.57);
      }
  }

}

vector<float> TrackUserNI::trackUser()
{
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
//	std::cout << "trackUser user.size=" << users.getSize() << std::endl;
	vector<float> joints(22);

	for (int i = 0; i < users.getSize(); ++i){
		const nite::UserData& user = users[i];

		if (user.getSkeleton().getState() != nite::SKELETON_TRACKED)
			continue;

		head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
		neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
		righthip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
		rightknee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
		rightankle = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
		rightfoot = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
		lefthip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
		leftknee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
		leftankle = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
		leftfoot = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
		rightshoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
		rightelbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
		righthand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
		leftshoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
		leftelbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
		lefthand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
		userPoint = user.getSkeleton().getJoint(nite::JOINT_TORSO);

		//printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
		nite::Quaternion quad = leftshoulder.getOrientation();
		float x= quad.x;
		float y= quad.y;
		float z= quad.z;
		float w= quad.w;

		float roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
		float pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
		float yaw   =  asin(2*x*y + 2*z*w);
		printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), pitch, roll, yaw);


		jointarray[HumanJoints::Head].pointsXYZ[0] = head.getPosition().x;
		jointarray[HumanJoints::Head].pointsXYZ[1] = head.getPosition().y;
		jointarray[HumanJoints::Head].pointsXYZ[2] = head.getPosition().z;

		jointarray[HumanJoints::Neck].pointsXYZ[0] = neck.getPosition().x;
		jointarray[HumanJoints::Neck].pointsXYZ[1] = neck.getPosition().y;
		jointarray[HumanJoints::Neck].pointsXYZ[2] = neck.getPosition().z;

		jointarray[HumanJoints::RightHip].pointsXYZ[0] = righthip.getPosition().x;
		jointarray[HumanJoints::RightHip].pointsXYZ[1] = righthip.getPosition().y;
		jointarray[HumanJoints::RightHip].pointsXYZ[2] = righthip.getPosition().z;

		jointarray[HumanJoints::RightKnee].pointsXYZ[0] = rightknee.getPosition().x;
		jointarray[HumanJoints::RightKnee].pointsXYZ[1] = rightknee.getPosition().y;
		jointarray[HumanJoints::RightKnee].pointsXYZ[2] = rightknee.getPosition().z;

		jointarray[HumanJoints::RightAnkle].pointsXYZ[0] = rightankle.getPosition().x;
		jointarray[HumanJoints::RightAnkle].pointsXYZ[1] = rightankle.getPosition().y;
		jointarray[HumanJoints::RightAnkle].pointsXYZ[2] = rightankle.getPosition().z;

		jointarray[HumanJoints::RightFoot].pointsXYZ[0] = rightfoot.getPosition().x;
		jointarray[HumanJoints::RightFoot].pointsXYZ[1] = rightfoot.getPosition().y;
		jointarray[HumanJoints::RightFoot].pointsXYZ[2] = rightfoot.getPosition().z;

		jointarray[HumanJoints::LeftHip].pointsXYZ[0] = lefthip.getPosition().x;
		jointarray[HumanJoints::LeftHip].pointsXYZ[1] = lefthip.getPosition().y;
		jointarray[HumanJoints::LeftHip].pointsXYZ[2] = lefthip.getPosition().z;

		jointarray[HumanJoints::LeftKnee].pointsXYZ[0] = leftknee.getPosition().x;
		jointarray[HumanJoints::LeftKnee].pointsXYZ[1] = leftknee.getPosition().y;
		jointarray[HumanJoints::LeftKnee].pointsXYZ[2] = leftknee.getPosition().z;

		jointarray[HumanJoints::LeftAnkle].pointsXYZ[0] = leftankle.getPosition().x;
		jointarray[HumanJoints::LeftAnkle].pointsXYZ[1] = leftankle.getPosition().y;
		jointarray[HumanJoints::LeftAnkle].pointsXYZ[2] = leftankle.getPosition().z;

		jointarray[HumanJoints::LeftFoot].pointsXYZ[0] = leftfoot.getPosition().x;
		jointarray[HumanJoints::LeftFoot].pointsXYZ[1] = leftfoot.getPosition().y;
		jointarray[HumanJoints::LeftFoot].pointsXYZ[2] = leftfoot.getPosition().z;

		jointarray[HumanJoints::RightShoulder].pointsXYZ[0] = rightshoulder.getPosition().x;
		jointarray[HumanJoints::RightShoulder].pointsXYZ[1] = rightshoulder.getPosition().y;
		jointarray[HumanJoints::RightShoulder].pointsXYZ[2] = rightshoulder.getPosition().z;

		jointarray[HumanJoints::RightElbow].pointsXYZ[0] = rightelbow.getPosition().x;
		jointarray[HumanJoints::RightElbow].pointsXYZ[1] = rightelbow.getPosition().y;
		jointarray[HumanJoints::RightElbow].pointsXYZ[2] = rightelbow.getPosition().z;

		jointarray[HumanJoints::RightHand].pointsXYZ[0] = righthand.getPosition().x;
		jointarray[HumanJoints::RightHand].pointsXYZ[1] = righthand.getPosition().y;
		jointarray[HumanJoints::RightHand].pointsXYZ[2] = righthand.getPosition().z;

		jointarray[HumanJoints::LeftShoulder].pointsXYZ[0] = leftshoulder.getPosition().x;
		jointarray[HumanJoints::LeftShoulder].pointsXYZ[1] = leftshoulder.getPosition().y;
		jointarray[HumanJoints::LeftShoulder].pointsXYZ[2] = leftshoulder.getPosition().z;

		jointarray[HumanJoints::LeftElbow].pointsXYZ[0] = leftelbow.getPosition().x;
		jointarray[HumanJoints::LeftElbow].pointsXYZ[1] = leftelbow.getPosition().y;
		jointarray[HumanJoints::LeftElbow].pointsXYZ[2] = leftelbow.getPosition().z;

		jointarray[HumanJoints::LeftHand].pointsXYZ[0] = lefthand.getPosition().x;
		jointarray[HumanJoints::LeftHand].pointsXYZ[1] = lefthand.getPosition().y;
		jointarray[HumanJoints::LeftHand].pointsXYZ[2] = lefthand.getPosition().z;

		jointarray[HumanJoints::UserCOM].pointsXYZ[0] = userPoint.getPosition().x;
		jointarray[HumanJoints::UserCOM].pointsXYZ[1] = userPoint.getPosition().y;
		jointarray[HumanJoints::UserCOM].pointsXYZ[2] = userPoint.getPosition().z;


		float upperArm = vectorDistance(jointarray[HumanJoints::RightShoulder], jointarray[HumanJoints::RightElbow]);
		float lowerArm = vectorDistance(jointarray[HumanJoints::RightHand], jointarray[HumanJoints::RightElbow]);

		for(int i=0;i<3;i++){
			jointvectorarray[HumanJointVectors::RightHipToKnee].pointsXYZ[i] = jointarray[HumanJoints::RightHip].pointsXYZ[i] - jointarray[HumanJoints::RightKnee].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::RightKneeToAnkle].pointsXYZ[i] = jointarray[HumanJoints::RightKnee].pointsXYZ[i] - jointarray[HumanJoints::RightAnkle].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::RightHipToAnkle].pointsXYZ[i] = jointarray[HumanJoints::RightHip].pointsXYZ[i] - jointarray[HumanJoints::RightAnkle].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::LeftHipToKnee].pointsXYZ[i] = jointarray[HumanJoints::LeftHip].pointsXYZ[i] - jointarray[HumanJoints::LeftKnee].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::LeftKneeToAnkle].pointsXYZ[i] = jointarray[HumanJoints::LeftKnee].pointsXYZ[i] - jointarray[HumanJoints::LeftAnkle].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::LeftHipToAnkle].pointsXYZ[i] = jointarray[HumanJoints::LeftHip].pointsXYZ[i] - jointarray[HumanJoints::LeftAnkle].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::RightShoulderToElbow].pointsXYZ[i] = jointarray[HumanJoints::RightShoulder].pointsXYZ[i] - jointarray[HumanJoints::RightElbow].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::RightElbowToHand].pointsXYZ[i] = jointarray[HumanJoints::RightHand].pointsXYZ[i] - jointarray[HumanJoints::RightElbow].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::RightShoulderToHand].pointsXYZ[i] = jointarray[HumanJoints::RightShoulder].pointsXYZ[i] - jointarray[HumanJoints::RightHand].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::LeftShoulderToElbow].pointsXYZ[i] = jointarray[HumanJoints::LeftShoulder].pointsXYZ[i] - jointarray[HumanJoints::LeftElbow].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::LeftElbowToHand].pointsXYZ[i] = jointarray[HumanJoints::LeftHand].pointsXYZ[i] - jointarray[HumanJoints::LeftElbow].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::LeftShoulderToHand].pointsXYZ[i] = jointarray[HumanJoints::LeftShoulder].pointsXYZ[i] - jointarray[HumanJoints::LeftHand].pointsXYZ[i];
			jointvectorarray[HumanJointVectors::HeadToNeck].pointsXYZ[i] = jointarray[HumanJoints::Head].pointsXYZ[i] - jointarray[HumanJoints::Neck].pointsXYZ[i];
		}

		float sEMAG = vectorNorm(jointvectorarray[HumanJointVectors::RightShoulderToElbow]);
		float hEMAG = vectorNorm(jointvectorarray[HumanJointVectors::RightElbowToHand]);
		float SHEMAG = vectorNorm(jointvectorarray[HumanJointVectors::RightShoulderToHand]);

		float sEMAGLeft = vectorNorm(jointvectorarray[HumanJointVectors::LeftShoulderToElbow]);
		float hEMAGLeft = vectorNorm(jointvectorarray[HumanJointVectors::LeftElbowToHand]);
		float SHEMAGLeft = vectorNorm(jointvectorarray[HumanJointVectors::LeftShoulderToHand]);

		float sEMAG1 = vectorNorm(jointvectorarray[HumanJointVectors::RightHipToKnee]);
		float hEMAG1 = vectorNorm(jointvectorarray[HumanJointVectors::RightKneeToAnkle]);
		float SHEMAG1 = vectorNorm(jointvectorarray[HumanJointVectors::RightHipToAnkle]);

		float sEMAG2 = vectorNorm(jointvectorarray[HumanJointVectors::LeftHipToKnee]);
		float hEMAG2 = vectorNorm(jointvectorarray[HumanJointVectors::LeftKneeToAnkle]);
		float SHEMAG2 = vectorNorm(jointvectorarray[HumanJointVectors::LeftHipToAnkle]);

		anglebetweenel= acos(((SHEMAG*SHEMAG)-((sEMAG*sEMAG)+(hEMAG*hEMAG)))/(-2*hEMAG*sEMAG))*180/PI;
		anglebetweenelLeft= acos(((SHEMAGLeft*SHEMAGLeft)-((sEMAGLeft*sEMAGLeft)+(hEMAGLeft*hEMAGLeft)))/(-2*hEMAGLeft*sEMAGLeft))*180/PI;

		betweenknee=acos(((SHEMAG1*SHEMAG1)-((sEMAG1*sEMAG1)+(hEMAG1*hEMAG1)))/(-2*hEMAG1*sEMAG1))*180/PI;
		Lbetweenknee=acos(((SHEMAG2*SHEMAG2)-((sEMAG2*sEMAG2)+(hEMAG2*hEMAG2)))/(-2*hEMAG2*sEMAG2))*180/PI;

		float headXY = angleZXY(jointvectorarray[HumanJointVectors::HeadToNeck]);
		float headYZ = angleXYZ(jointvectorarray[HumanJointVectors::HeadToNeck]);

		float hipXZ = angleYXZ(jointarray[HumanJoints::RightHip]);
		float kneeXZ = angleYXZ(jointarray[HumanJoints::RightKnee]);
		float ankleXZ = angleYXZ(jointarray[HumanJoints::RightAnkle]);

		float hipYZ = angleXYZ(jointarray[HumanJoints::RightHip]);
		float kneeYZ = angleXYZ(jointarray[HumanJoints::RightKnee]);
		float ankleYZ = angleXYZ(jointarray[HumanJoints::RightAnkle]);

		float hipXY = angleZXY(jointarray[HumanJoints::RightHip]);
		float kneeXY = angleZXY(jointarray[HumanJoints::RightKnee]);
		float ankleXY = angleZXY(jointarray[HumanJoints::RightAnkle]);

		float LkneeYZ = angleXYZ(jointarray[HumanJoints::LeftKnee]);
		float LhipYZ = angleXYZ(jointarray[HumanJoints::LeftHip]);

		rshoulderXZ = angleYXZ(jointarray[HumanJoints::RightShoulder]);
		relbowXZ = angleYXZ(jointarray[HumanJoints::RightElbow]);
		rwristXZ = angleYXZ(jointarray[HumanJoints::RightHand]);
		lshoulderXZ = angleYXZ(jointarray[HumanJoints::LeftShoulder]);
		lelbowXZ = angleYXZ(jointarray[HumanJoints::LeftElbow]);
		lwristXZ = angleYXZ(jointarray[HumanJoints::LeftHand]);

		rshoulderYZ = angleXYZ(jointarray[HumanJoints::RightShoulder]);
		relbowYZ = angleXYZ(jointarray[HumanJoints::RightElbow]);
		rwristYZ = angleXYZ(jointarray[HumanJoints::RightHand]);
		lshoulderYZ = angleXYZ(jointarray[HumanJoints::LeftShoulder]);
		lelbowYZ = angleXYZ(jointarray[HumanJoints::LeftElbow]);
		lwristYZ = angleXYZ(jointarray[HumanJoints::LeftHand]);

		rshoulderXY = angleZXY(jointarray[HumanJoints::RightShoulder]);
		relbowXY = angleZXY(jointarray[HumanJoints::RightElbow]);
		rwristXY = angleZXY(jointarray[HumanJoints::RightHand]);
		lshoulderXY = angleZXY(jointarray[HumanJoints::LeftShoulder]);
		lelbowXY = angleZXY(jointarray[HumanJoints::LeftElbow]);
		lwristXY = angleZXY(jointarray[HumanJoints::LeftHand]);


		Lhiptokneeright_YZ=LkneeYZ-LhipYZ;
		hiptokneeright_XY=kneeXY-hipXY;
		hiptokneeright_YZ=kneeYZ-hipYZ;
		hiptokneeright_XZ=kneeXZ-hipXZ;
		kneetoankleright_XY=kneeXY-ankleXY;
		kneetoankleright_YZ=kneeYZ-ankleYZ;
		kneetoankleright_XZ=kneeXZ-ankleXZ;
		shoulderPitch=(-1)*(rshoulderXZ-relbowXZ);
		shoulderRoll=(-1)*(rshoulderYZ-relbowYZ);
		shoulderPitchLeft=(-1)*(lshoulderXZ-lelbowXZ);
		shoulderRollLeft=(-1)*(lshoulderYZ-lelbowYZ);
		elbowYaw=(-1)*(relbowXY-rwristXY);
		elbowRoll=(-1)*(relbowYZ-rwristYZ);
		elbowYawLeft=(-1)*(lelbowXY-lwristXY);
		elbowRollLeft=(-1)*(lelbowYZ-lwristYZ);

		///TODO: Burda -1* vardı teleop_nao_ni.cpp de, bu sadece robot için mi buna bak
		hiptokneeright_XY = angle((jointarray[HumanJoints::RightKnee].pointsXYZ[2] - jointarray[HumanJoints::RightHip].pointsXYZ[2]), (jointarray[HumanJoints::RightKnee].pointsXYZ[1] - jointarray[HumanJoints::RightHip].pointsXYZ[1]), (jointarray[HumanJoints::RightKnee].pointsXYZ[0] - jointarray[HumanJoints::RightHip].pointsXYZ[0]))/10.0;
		Lhiptokneeright_XY = angle((jointarray[HumanJoints::LeftKnee].pointsXYZ[2] - jointarray[HumanJoints::LeftHip].pointsXYZ[2]), (jointarray[HumanJoints::LeftKnee].pointsXYZ[1] - jointarray[HumanJoints::LeftHip].pointsXYZ[1]), (jointarray[HumanJoints::LeftKnee].pointsXYZ[0] - jointarray[HumanJoints::LeftHip].pointsXYZ[0]))/10.0;

		hiptokneeright_YZ = angleXYZ(takeMinusVector(jointvectorarray[HumanJointVectors::RightHipToKnee]))/10.0;
		Lhiptokneeright_YZ = angleXYZ(takeMinusVector(jointvectorarray[HumanJointVectors::LeftHipToKnee]))/10.0;
		shoulderPitchLeft = angleYXZ(takeMinusVector(jointvectorarray[HumanJointVectors::LeftShoulderToElbow]))/10.0;
		shoulderRoll = angleXYZ(takeMinusVector(jointvectorarray[HumanJointVectors::RightShoulderToElbow]))/10.0;
		shoulderPitch = angleYXZ(takeMinusVector(jointvectorarray[HumanJointVectors::RightShoulderToElbow]))/10.0;
		shoulderRollLeft = angleXYZ(takeMinusVector(jointvectorarray[HumanJointVectors::LeftShoulderToElbow]))/10.0;

		head_front = (90- (angleYXZ(jointvectorarray[HumanJointVectors::HeadToNeck])/10.0))*(jointvectorarray[HumanJointVectors::HeadToNeck].pointsXYZ[2]/fabsf(jointvectorarray[HumanJointVectors::HeadToNeck].pointsXYZ[2]));
		head_side = headYZ;

		float a=shoulderPitch*PI/180;
		float b=shoulderRoll*PI/180;
		float c=-90-(180-anglebetweenel)*PI/180;
		float d=0;

		float m=cos(a)*cos(b);
		float n=-cos(a)*sin(b);
		float k=sin(a)*cos(b);
		float g=-sin(a)*sin(b);

		float A=-m*sin(c)+n*cos(c);
		float B=-k*sin(c)+g*cos(c);
		float C=-sin(c)*sin(b)+cos(b)*cos(c);
		float D=-cos(c)*m-sin(c)*n;
		float E=-cos(c)*k-sin(c)*g;
		float F=-cos(c)*sin(b)-sin(c)*cos(b);

		float a_left=shoulderPitchLeft*PI/180;
		float b_left=shoulderRollLeft*PI/180;
		float c_left=-90-(180-anglebetweenelLeft)*PI/180;
		float d_left=0;

		float m_left=cos(a_left)*cos(b_left);
		float n_left=-cos(a_left)*sin(b_left);
		float k_left=sin(a_left)*cos(b_left);
		float g_left=-sin(a_left)*sin(b_left);

		float A_left=-m_left*sin(c_left)+n_left*cos(c_left);
		float B_left=-k_left*sin(c_left)+g_left*cos(c_left);
		float C_left=-sin(c_left)*sin(b_left)+cos(b_left)*cos(c_left);
		float D_left=-cos(c_left)*m_left-sin(c_left)*n_left;
		float E_left=-cos(c_left)*k_left-sin(c_left)*g_left;
		float F_left=-cos(c_left)*sin(b_left)-sin(c_left)*cos(b_left);

		float Shoulder_X = jointarray[HumanJoints::RightShoulder].pointsXYZ[0];
		float Shoulder_Y = jointarray[HumanJoints::RightShoulder].pointsXYZ[1];
		float Shoulder_Z = jointarray[HumanJoints::RightShoulder].pointsXYZ[2];

		float Shoulder_X_left = jointarray[HumanJoints::LeftShoulder].pointsXYZ[0];
		float Shoulder_Y_left = jointarray[HumanJoints::LeftShoulder].pointsXYZ[1];
		float Shoulder_Z_left = jointarray[HumanJoints::LeftShoulder].pointsXYZ[2];

		float len1=310;
		float len2=280;

		float Calc_Wrist_Z = (-sin(d)*A+cos(d)*sin(a))*Shoulder_Z-D*Shoulder_Y+(cos(d)*A+sin(d)*sin(a))*Shoulder_X+(cos(d)*A+sin(d)*sin(a))*len2+m*len1;
		float Calc_Wrist_Y = (-sin(d)*B-cos(d)*cos(a))*Shoulder_Z-E*Shoulder_Y+(cos(d)*B-sin(d)*cos(a))*Shoulder_X+(cos(d)*B-sin(d)*cos(a))*len2+k*len1;
		float Calc_Wrist_X = -F*Shoulder_Y+C*cos(d)*Shoulder_X+C*cos(d)*len2+sin(b)*len1;

		float Calc_Wrist_Z_left = (-sin(d_left)*A_left+cos(d_left)*sin(a_left))*Shoulder_Z_left-D_left*Shoulder_Y_left+(cos(d_left)*A_left+sin(d_left)*sin(a_left))*Shoulder_X_left+(cos(d_left)*A_left+sin(d_left)*sin(a_left))*len2+m_left*len1;
		float Calc_Wrist_Y_left = (-sin(d_left)*B_left-cos(d_left)*cos(a_left))*Shoulder_Z_left-E_left*Shoulder_Y_left+(cos(d_left)*B_left-sin(d_left)*cos(a_left))*Shoulder_X_left+(cos(d_left)*B_left-sin(d_left)*cos(a_left))*len2+k_left*len1;
		float Calc_Wrist_X_left = -F_left*Shoulder_Y_left+C_left*cos(d_left)*Shoulder_X_left+C_left*cos(d_left)*len2+sin(b_left)*len1;

		float pos_Z=-1*((cos(d)*A+sin(d)*sin(a))*len2+m*len1)+Shoulder_Z;
		float pos_Y=(cos(d)*B-sin(d)*cos(a))*len2+k*len1+Shoulder_Y;
		float pos_X=C*cos(d)*len2+sin(b)*len1+Shoulder_X;

		float pos_Z_left=-1*((cos(d_left)*A_left+sin(d_left)*sin(a_left))*len2+m_left*len1)+Shoulder_Z_left;
		float pos_Y_left=(cos(d_left)*B_left-sin(d_left)*cos(a_left))*len2+k_left*len1+Shoulder_Y_left;
		float pos_X_left=-1*(C_left*cos(d_left)*len2+sin(b_left)*len1)+Shoulder_X_left;


		float wanted=0;
		float wanted_left=0;

		if(fabsf(jointarray[HumanJoints::RightHand].pointsXYZ[0]-pos_X) > fabsf(jointarray[HumanJoints::RightHand].pointsXYZ[1]-pos_Y) && fabsf(jointarray[HumanJoints::RightHand].pointsXYZ[0]-pos_X) > fabsf(jointarray[HumanJoints::RightHand].pointsXYZ[2]-pos_Z)){
			if((jointarray[HumanJoints::RightHand].pointsXYZ[0]-Shoulder_X-sin(b)*len1)/(C*len2) > 1 || (jointarray[HumanJoints::RightHand].pointsXYZ[0]-Shoulder_X-sin(b)*len1)/(C*len2) < -1){
				wanted=90;
			}
			else{
				wanted=acos((jointarray[HumanJoints::RightHand].pointsXYZ[0]-Shoulder_X-sin(b)*len1)/(C*len2))*180/PI;
			}
		}
		else{
			float term1=(B*jointarray[HumanJoints::RightHand].pointsXYZ[2]-B*jointarray[HumanJoints::RightShoulder].pointsXYZ[2]+A*jointarray[HumanJoints::RightHand].pointsXYZ[1]-A*jointarray[HumanJoints::RightShoulder].pointsXYZ[1]+B*m*len1-A*k*len1)/(-B*sin(a)*len2-A*cos(a)*len2);
			if( term1<=1 && term1 >= -1){
				wanted=asin(term1)*-180/PI;}
			else {
				wanted = 90;
			}

		}
		if(fabsf(jointarray[HumanJoints::LeftHand].pointsXYZ[0]-pos_X_left) > fabsf(jointarray[HumanJoints::LeftHand].pointsXYZ[1]-pos_Y_left) && fabsf(jointarray[HumanJoints::LeftHand].pointsXYZ[0]-pos_X_left) > fabsf(jointarray[HumanJoints::LeftHand].pointsXYZ[2]-pos_Z_left)){
			if((-jointarray[HumanJoints::LeftHand].pointsXYZ[0]+Shoulder_X_left-sin(b_left)*len1)/(C_left*len2) > 1 || (-jointarray[HumanJoints::LeftHand].pointsXYZ[0]+Shoulder_X_left-sin(b_left)*len1)/(C_left*len2) < -1){
				wanted_left=90;
			}
			else{
				wanted_left=acos((-jointarray[HumanJoints::LeftHand].pointsXYZ[0]+Shoulder_X_left-sin(b_left)*len1)/(C_left*len2))*180/PI;
			}
		}
		else{
			float term2=(B_left*jointarray[HumanJoints::LeftHand].pointsXYZ[2]-B_left*jointarray[HumanJoints::LeftShoulder].pointsXYZ[2]+A_left*jointarray[HumanJoints::LeftHand].pointsXYZ[1]-A_left*jointarray[HumanJoints::LeftShoulder].pointsXYZ[1]+B_left*m_left*len1-A_left*k_left*len1)/(-B_left*sin(a_left)*len2-A_left*cos(a_left)*len2);

			if( term2<=1 && term2 >= -1){
				wanted_left=asin(term2)*-180/PI;}
			else {
				wanted_left = 90;
			}
		}
		// shoulder Roll daki -1* lari kaldırdım
		joints[0] = 0.0;
		joints[1] = 0.0;
		joints[2] = -1*(((int)shoulderPitchLeft*PI/180)+0.8);   //shoulderPitch
		joints[3] = 1.5-(-1*(int)shoulderRollLeft*PI/180);       //shoulderRoll
		joints[4] = -1*(int)wanted_left*PI/180;         //elbowYaw
		joints[5] = (((180-(int)anglebetweenelLeft)*PI/180)-0.4); //elbowRoll
		joints[6] = -1*(((int)shoulderPitch*PI/180)+0.8) ;
		joints[7] = -1*(1.5-((int)shoulderRoll*PI/180));
		joints[8] =  (int)wanted*PI/180;
		joints[9] = (((180-(int)anglebetweenel)*PI/180)-0.4);
		joints[10] = 0.0;
		joints[11] = (-1*(int)Lhiptokneeright_YZ*PI/180);
		joints[12] = (int)Lhiptokneeright_XY*PI/180 ;
		joints[13] = (180-(int)Lbetweenknee)*PI/180;
		joints[14] = 0.0;
		joints[15] = 0.0;
		joints[16] = 0.0;
		joints[17] = (-1*(int)hiptokneeright_YZ*PI/180);
		joints[18] = (int)hiptokneeright_XY*PI/180;
		joints[19] = ((180-(int)betweenknee)*PI/180);
		joints[20] = 0.0;
		joints[21] = 0.0;

		std::cout<<"ShoulderRollLEFT: "<<joints[3]<<"ShoulderRollRIGHT: "<<joints[7]<<std::endl;
		std::cout<<"ShoulderPitchLEFT: "<<joints[2]<<"ShoulderPitchRIGHT: "<<joints[6]<<std::endl;

		//joints[3] = 0.0;
		//joints[7] = 0.0;
		//joints[2] = -0.7;
		//joints[6] = -0.7;

		// remove invalid joint angles
		joints[4] = -0.5;
		joints[8] = 0.5;
		for (int i = 0; i < joints.size(); i++)
		{
			if (joints[i] > 4 || joints[i] < -4)
			{
				joints[i] = 0;
			}
		}
	}

	return joints;
}


inline float TrackUserNI::angle(float x,float y, float z){
	float angle = 0.0;
	angle = atan(x/sqrt((y*y)+(z*z)))* 1800 / PI;
	return angle;
}

inline float TrackUserNI::vectorDistance(Vector3D vector1, Vector3D vector2){
	float result = 0.0;
	for(int i=0;i<3;i++){
		result = result + ((vector1.pointsXYZ[i] - vector2.pointsXYZ[i]) * (vector1.pointsXYZ[i] - vector2.pointsXYZ[i]));
	}
	result = sqrt(result);
	return result;
}

inline float TrackUserNI::vectorNorm (Vector3D vector1){
	float result = 0.0;
	for(int i=0;i<3;i++){
		result = result + (vector1.pointsXYZ[i] * vector1.pointsXYZ[i]);
	}
	result = sqrt(result);
	return result;
}

inline Vector3D TrackUserNI::takeMinusVector (Vector3D vector1){

	for(int i=0;i<3;i++){
		vector1.pointsXYZ[i] = -vector1.pointsXYZ[i];
	}
	return vector1;
}

inline float TrackUserNI::angleZXY(Vector3D vector1){
	float angle = 0.0;
	angle = atan(vector1.pointsXYZ[2]/ sqrt((vector1.pointsXYZ[0]*vector1.pointsXYZ[0])+(vector1.pointsXYZ[1]*vector1.pointsXYZ[1])))* 1800 / PI;
	return angle;
}

inline float TrackUserNI::angleXYZ(Vector3D vector1){
	float angle = 0.0;
	angle = atan(vector1.pointsXYZ[0]/ sqrt((vector1.pointsXYZ[1]*vector1.pointsXYZ[1])+(vector1.pointsXYZ[2]*vector1.pointsXYZ[2])))* 1800 / PI;
	return angle;
}

inline float TrackUserNI::angleYXZ(Vector3D vector1){
	float angle = 0.0;
	angle = atan(vector1.pointsXYZ[1]/ sqrt((vector1.pointsXYZ[0]*vector1.pointsXYZ[0])+(vector1.pointsXYZ[2]*vector1.pointsXYZ[2])))* 1800 / PI;
	return angle;
}

