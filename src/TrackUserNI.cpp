/*
 * TrackUserNI.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: binnur
 */

#include "TrackUserNI.h"
#include <iostream>
#include <fstream>
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
	jointarray = new Vector3D[15];
	jointvectorarray = new Vector3D[14];
  numJoints = 17;
  joints.resize(numJoints);
	outfile.open("legs.txt");
}

TrackUserNI::~TrackUserNI(){}

void TrackUserNI::init(int argc, char **argv, vector<float> *minLimits, vector<float> *maxLimits)
{
  minLimitsNI = minLimits;
  maxLimitsNI = maxLimits;

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
		updateUserState(user,userTrackerFrame.getTimestamp());

		if (user.isNew())
		{
		  userTracker.setSkeletonSmoothingFactor(0.85);
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

// Rotates array of 3D vectors (vectors) by an angle theta about vector rot.
vector<Vector3D> TrackUserNI::rodrigues_rot(vector<Vector3D> vectors, Vector3D rot, float theta){

  float vectorLength = vectorNorm(rot);
  div(rot,vectorLength); // normalize rotation axis

  int No = vectors.size(); // number of vectors in array
  vector<Vector3D> v_rot; // initialize rotated vector array

  Vector3D crosskv; // initialize cross product of rot and vector(i).
  for (int i=0; i<No ;i++){
    Vector3D temp;
    crosskv = cross(rot, vectors.at(i));
    for (int k=0;k<3;k++){
      temp.pointsXYZ[k] = mul(vectors.at(i),cos(theta)).pointsXYZ[k]+
          mul(crosskv,sin(theta)).pointsXYZ[k]+
          mul(rot, dot(rot,vectors.at(i))*(1-cos(theta))).pointsXYZ[k];
    }
    v_rot.push_back(temp);
  }
  return v_rot;
}

vector<float> TrackUserNI::extractAng(Vector3D bone, float rotAng){

  // find pitch angle
  vector <Vector3D> axes;
  axes.push_back(Vector3D(0,1,0));
  axes.push_back(Vector3D(0,0,1));

  Vector3D rot(1,0,0);
  vector <Vector3D> rotatedAxes;
  rotatedAxes = rodrigues_rot(axes,rot, rotAng);

  Vector3D AV (0, 0, 0);
  Vector3D BV (1, 0, 0);
  Vector3D CV = rotatedAxes.at(0);
  Vector3D DV(0, 0, 0);
  Vector3D EV (-bone.pointsXYZ[2],
               -bone.pointsXYZ[0],
               bone.pointsXYZ[1]);
  // normal vector to plane ABC
  Vector3D N = cross(sub(BV,AV), sub(CV,AV));
  // angle between plane and line, equals pi/2 - angle between D-E and N
  float pitch = PI/2 -acos(dot(sub(EV,DV), N)/vectorNorm(N)/vectorNorm(sub(EV,DV)));

  // find roll angle
  axes.clear();
  axes.push_back(rotatedAxes.at(0));
  axes.push_back(rotatedAxes.at(1));
  rotatedAxes = rodrigues_rot(axes, rot, pitch);
  BV = rotatedAxes.at(0);
  CV = rotatedAxes.at(1);
  N = cross(sub(BV,AV), sub(CV,AV));
  // angle between plane and line, equals pi/2 - angle between D-E and N
  float roll = PI/2 -acos(dot(sub(EV,DV), N)/vectorNorm(N)/vectorNorm(sub(EV,DV)));
  vector<float> angles;
  angles.push_back(pitch);
  angles.push_back(roll);
  return angles;

}

Vector3D TrackUserNI::convertPoint3ftoVector3D (const nite::Point3f& point){
   Vector3D vector(point.x, point.y, point.z);
   return vector;

}

void TrackUserNI::clipJoints (vector<float> &joints){
    for (int i = 0; i< joints.size(); i++){
      if(joints[i] < minLimitsNI->at(i))
        joints[i] = minLimitsNI->at(i);
      else if (joints[i] > maxLimitsNI->at(i))
        joints[i] = maxLimitsNI->at(i);
    }

}

vector<float> TrackUserNI::trackUser()
{
  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

  for (int i = 0; i < users.getSize(); ++i){
    const nite::UserData& user = users[i];

    if (user.getSkeleton().getState() != nite::SKELETON_TRACKED)
      continue;

    // get joints from skeleton
    head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
    neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
    righthip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
    rightknee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
    rightfoot = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
    lefthip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
    leftknee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
    leftfoot = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
    rightshoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
    rightelbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
    righthand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
    leftshoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
    leftelbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
    lefthand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
    torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);

    // get positions from joints
    jointarray[HumanJoints::Head] = convertPoint3ftoVector3D(head.getPosition());
    jointarray[HumanJoints::Neck] = convertPoint3ftoVector3D(neck.getPosition());
    jointarray[HumanJoints::RightHip] = convertPoint3ftoVector3D(righthip.getPosition());
    jointarray[HumanJoints::RightKnee] = convertPoint3ftoVector3D(rightknee.getPosition());
    jointarray[HumanJoints::RightFoot] = convertPoint3ftoVector3D(rightfoot.getPosition());
    jointarray[HumanJoints::LeftHip] = convertPoint3ftoVector3D(lefthip.getPosition());
    jointarray[HumanJoints::LeftKnee] = convertPoint3ftoVector3D(leftknee.getPosition());
    jointarray[HumanJoints::LeftFoot] = convertPoint3ftoVector3D(leftfoot.getPosition());
    jointarray[HumanJoints::RightShoulder] = convertPoint3ftoVector3D(rightshoulder.getPosition());
    jointarray[HumanJoints::RightElbow] = convertPoint3ftoVector3D(rightelbow.getPosition());
    jointarray[HumanJoints::RightHand] = convertPoint3ftoVector3D(righthand.getPosition());
    jointarray[HumanJoints::LeftShoulder] = convertPoint3ftoVector3D(leftshoulder.getPosition());
    jointarray[HumanJoints::LeftElbow] = convertPoint3ftoVector3D(leftelbow.getPosition());
    jointarray[HumanJoints::LeftHand] = convertPoint3ftoVector3D(lefthand.getPosition());
    jointarray[HumanJoints::Torso] = convertPoint3ftoVector3D(torso.getPosition());


    // calculate vector(s) between two adjacent joints

    jointvectorarray[HumanJointVectors::RightHipToKnee] = sub(jointarray[HumanJoints::RightKnee], jointarray[HumanJoints::RightHip]);
    jointvectorarray[HumanJointVectors::RightKneeToAnkle] = sub(jointarray[HumanJoints::RightFoot], jointarray[HumanJoints::RightKnee]);
    jointvectorarray[HumanJointVectors::RightHipToAnkle] = sub(jointarray[HumanJoints::RightFoot], jointarray[HumanJoints::RightHip]);
    jointvectorarray[HumanJointVectors::LeftHipToKnee] = sub(jointarray[HumanJoints::LeftKnee], jointarray[HumanJoints::LeftHip]);
    jointvectorarray[HumanJointVectors::LeftKneeToAnkle] = sub(jointarray[HumanJoints::LeftFoot], jointarray[HumanJoints::LeftKnee]);
    jointvectorarray[HumanJointVectors::LeftHipToAnkle] = sub(jointarray[HumanJoints::LeftFoot], jointarray[HumanJoints::LeftHip]);
    jointvectorarray[HumanJointVectors::RightShoulderToElbow] = sub(jointarray[HumanJoints::RightElbow], jointarray[HumanJoints::RightShoulder]);
    jointvectorarray[HumanJointVectors::RightElbowToHand] = sub(jointarray[HumanJoints::RightHand], jointarray[HumanJoints::RightElbow]);
    jointvectorarray[HumanJointVectors::RightShoulderToHand] = sub(jointarray[HumanJoints::RightHand], jointarray[HumanJoints::RightShoulder]);
    jointvectorarray[HumanJointVectors::LeftShoulderToElbow] = sub(jointarray[HumanJoints::LeftElbow], jointarray[HumanJoints::LeftShoulder]);
    jointvectorarray[HumanJointVectors::LeftElbowToHand] = sub(jointarray[HumanJoints::LeftHand], jointarray[HumanJoints::LeftElbow]);
    jointvectorarray[HumanJointVectors::LeftShoulderToHand] = sub(jointarray[HumanJoints::LeftHand], jointarray[HumanJoints::LeftShoulder]);
    jointvectorarray[HumanJointVectors::HeadToNeck] = sub(jointarray[HumanJoints::Head], jointarray[HumanJoints::Neck]);
    jointvectorarray[HumanJointVectors::NeckToTorso] = sub(jointarray[HumanJoints::Neck], jointarray[HumanJoints::Torso]);



    vector<float> shoulder_angles = extractAng(jointvectorarray[HumanJointVectors::LeftShoulderToElbow], -0.7);
    shoulderPitchLeft = -1*shoulder_angles.at(0);
    shoulderRollLeft = shoulder_angles.at(1);

    shoulder_angles = extractAng(jointvectorarray[HumanJointVectors::RightShoulderToElbow], 0.7);

    shoulderPitch = -1*shoulder_angles.at(0);
    //std::cout<<shoulderPitch<<std::endl;
    shoulderRoll = -1*shoulder_angles.at(1);

    elbowRollLeft = 0.75*PI - angleBtwVecs(jointvectorarray[HumanJointVectors::LeftShoulderToElbow], jointvectorarray[HumanJointVectors::LeftElbowToHand]);
    elbowRoll = 0.75*PI - angleBtwVecs(jointvectorarray[HumanJointVectors::RightShoulderToElbow], jointvectorarray[HumanJointVectors::RightElbowToHand]);

    vector<float> hip_angles = extractAng(jointvectorarray[HumanJointVectors::LeftHipToKnee], 1.57);
    hipPitchLeft = -1*hip_angles.at(0);  //figure1
    hipRollLeft = -1*hip_angles.at(1);  //figure2

    hip_angles = extractAng(jointvectorarray[HumanJointVectors::RightHipToKnee], 1.57);

    hipPitch = hip_angles.at(0);  //figure3
    hipRoll = hip_angles.at(1);  //figure4

    kneePitchLeft = -1*(PI - angleBtwVecs(jointvectorarray[HumanJointVectors::LeftHipToKnee], jointvectorarray[HumanJointVectors::LeftKneeToAnkle]));
    kneePitch = PI - angleBtwVecs(jointvectorarray[HumanJointVectors::RightHipToKnee], jointvectorarray[HumanJointVectors::RightKneeToAnkle]);

    torsoBending = extractAng(jointvectorarray[HumanJointVectors::NeckToTorso],1.57).at(1);
    /*outfile<<hipPitchLeft<<" "<<hipRollLeft<<" "
        <<hipPitch<<" "<<hipRoll<<" "
        <<kneePitchLeft<<" "<<kneePitch<<std::endl;
    cout<<"Torso "<<torsoBending<<endl;*/

    joints[0] = 0.0;
    joints[1] = 0.0;

    joints[2] = shoulderPitchLeft;   //shoulderPitch
    joints[3] = shoulderRollLeft;       //shoulderRoll
    joints[4] = -0.5;         //elbowYaw
    joints[5] = elbowRollLeft; //elbowRoll

    joints[6] = shoulderPitch;
    joints[7] = shoulderRoll;
    joints[8] = 0.5;
    joints[9] = elbowRoll;


    joints[10] = hipPitchLeft;
    joints[11] = hipRollLeft;
    joints[12] = kneePitchLeft;

    joints[13] = hipPitch;
    joints[14] = hipRoll;
    joints[15] = kneePitch;

    joints[16] = torsoBending;
    //clipJoints(joints);

  }

  return joints;
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

inline void TrackUserNI::add (Vector3D &vector, float op){
  for(int i=0;i<3;i++){
    vector.pointsXYZ[i] += op;
  }
}

inline void TrackUserNI::div (Vector3D &vector, float op){
  for(int i=0;i<3;i++){
    vector.pointsXYZ[i] /= op;
  }
}
inline Vector3D TrackUserNI::mul (Vector3D vector, float op){
  for(int i=0;i<3;i++){
    vector.pointsXYZ[i] *= op;
  }
  return vector;
}
inline Vector3D TrackUserNI::sub (Vector3D vector1, Vector3D vector2){
  Vector3D vector;
  for(int i=0;i<3;i++){
    vector.pointsXYZ[i] = vector1.pointsXYZ[i] - vector2.pointsXYZ[i];
  }
  return vector;
}

inline float TrackUserNI::dot (Vector3D vector1, Vector3D vector2){
  float dot = 0.0;
  for(int i=0;i<3;i++){
    dot += vector1.pointsXYZ[i] * vector2.pointsXYZ[i];
  }
  return dot;
}

inline Vector3D TrackUserNI::cross (Vector3D k, Vector3D v){
  Vector3D crosskv;
  crosskv.pointsXYZ[0] = k.pointsXYZ[1]*v.pointsXYZ[2] - k.pointsXYZ[2]*v.pointsXYZ[1];
  crosskv.pointsXYZ[1] = k.pointsXYZ[2]*v.pointsXYZ[0] - k.pointsXYZ[0]*v.pointsXYZ[2];
  crosskv.pointsXYZ[2] = k.pointsXYZ[0]*v.pointsXYZ[1] - k.pointsXYZ[1]*v.pointsXYZ[0];
  return crosskv;
}

inline float TrackUserNI::angleBtwVecs (Vector3D k, Vector3D v){
  return PI-acos(dot(k,v)/(vectorNorm(k)*vectorNorm(v)));
}

