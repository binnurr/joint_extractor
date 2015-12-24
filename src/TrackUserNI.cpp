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
	outfile.open("legs.txt");  // TODO: remove after debugging
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
  if(vectorLength == 0.0){
    //vectorLength = 0.0001;
  }
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

vector<float> TrackUserNI::extractAng(Vector3D bone, float rotAng, int right){

  // find pitch angle
  vector <Vector3D> axes;
  axes.push_back(Vector3D(0,right*1,0));
  axes.push_back(Vector3D(0,0,1));

  Vector3D rot(1,0,0);
  vector <Vector3D> rotatedAxes;
  rotatedAxes = rodrigues_rot(axes,rot, rotAng);

  Vector3D AV (0, 0, 0);
  Vector3D BV (1, 0, 0);
  Vector3D CV = rotatedAxes.at(0);
  Vector3D DV(0, 0, 0);
  Vector3D EV (-bone.pointsXYZ[2],
                bone.pointsXYZ[0],
               -bone.pointsXYZ[1]);
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

Vector3D TrackUserNI::extendVector3D (Vector3D bone, float length){
   float ori_length = vectorNorm(bone);
   float new_y = bone.pointsXYZ[1] + (bone.pointsXYZ[1]/ori_length)*length;
   float new_x = bone.pointsXYZ[0] + abs(cos(asin(bone.pointsXYZ[1]/ori_length))*length*cos(atan(bone.pointsXYZ[2]/bone.pointsXYZ[0])))* (bone.pointsXYZ[0]/fabs(bone.pointsXYZ[0]));
   float new_z = bone.pointsXYZ[2] + abs(cos(asin(bone.pointsXYZ[1]/ori_length))*length*sin(atan(bone.pointsXYZ[2]/bone.pointsXYZ[0])))* (bone.pointsXYZ[2]/fabs(bone.pointsXYZ[2]));
   Vector3D ext_vector(new_x, new_y, new_z);
   Vector3D DV = sub(ext_vector,bone);
   return DV;

}

void TrackUserNI::printVec (string name, Vector3D v){
  std::cout<<std::fixed << std::setprecision(3)<<name<<": "<<v.pointsXYZ[0]<<" "<<v.pointsXYZ[1]<<" "<<v.pointsXYZ[2]<<std::endl;
  return;

}

float TrackUserNI::rotateAxes (float pitch, float elbowRoll, bool rightSide){

  Vector3D x(1,0,0);
  Vector3D y(0,1,0);
  Vector3D z(0,0,1);
  vector<Vector3D> vectors;
  vectors.push_back(y);
  y = rodrigues_rot(vectors, x, -(0.7+ pitch)).at(0);
  z = cross(y,x);

  int right = 0;
  if(rightSide)
    right = 3;

  float len = vectorNorm(jointvectorarray[1 + right]);
  Vector3D extended = extendVector3D(jointvectorarray[0 + right],len);
  vectors.clear();
  vectors.push_back(extended);
  Vector3D new_rot (-z.pointsXYZ[1], z.pointsXYZ[2], -z.pointsXYZ[0]);
  Vector3D bended = rodrigues_rot(vectors, new_rot, (elbowRoll+0.8)).at(0);
  Vector3D calculated = sum(bended, jointvectorarray[0 + right]);


  Vector3D diff = sub(jointvectorarray[2 + right], calculated);

  float shoulderYaw = 0.0;
  if( diff.pointsXYZ[0] > 100 || diff.pointsXYZ[1] > 100 || diff.pointsXYZ[2] > 100){

     y = Vector3D(-y.pointsXYZ[1], y.pointsXYZ[2], -y.pointsXYZ[0]);
     div(y,vectorNorm(y)); // normalize rotation axis
     div(bended,vectorNorm(bended));
     Vector3D crosskv = cross(y, bended);
     Vector3D final = jointvectorarray[1 + right];
     div(final,vectorNorm(final));

     float min = 5.0;
     float theta = 100.0;
     vectors.clear();
     vectors.push_back(bended);
     Vector3D temp;
     float angle;
     for (int i=-180;i<=180;i++){
       temp = rodrigues_rot(vectors, y, (i/180.0)*PI).at(0);
       angle = angleBtwVecs (temp, final);
       if (fabs(angle) < fabs(min)){
         min = angle;
         theta = (i/180.0)*PI;
       }
     }
     shoulderYaw = theta;
  }
  return shoulderYaw;
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



    vector<float> shoulder_angles = extractAng(jointvectorarray[HumanJointVectors::LeftShoulderToElbow], -0.7, 1);
    shoulderPitchLeft = shoulder_angles.at(0);
    shoulderRollLeft = shoulder_angles.at(1);

    //std::cout<<"PL: "<<shoulderPitchLeft<<"RL: "<<shoulderRollLeft<<std::endl;

    shoulder_angles = extractAng(jointvectorarray[HumanJointVectors::RightShoulderToElbow], 0.7, -1);

    shoulderPitch = -1*shoulder_angles.at(0);
    shoulderRoll = shoulder_angles.at(1);

    //std::cout<<"PR: "<<shoulderPitch<<"RR: "<<shoulderRoll<<std::endl;

    elbowRollLeft = 0.75*PI - angleBtwVecs(jointvectorarray[HumanJointVectors::LeftShoulderToElbow], jointvectorarray[HumanJointVectors::LeftElbowToHand]);
    elbowRoll = 0.75*PI - angleBtwVecs(jointvectorarray[HumanJointVectors::RightShoulderToElbow], jointvectorarray[HumanJointVectors::RightElbowToHand]);

    //std::cout<<"EL: "<<elbowRollLeft<<"ER: "<<elbowRoll<<std::endl;

    shoulderYawLeft = -1*rotateAxes(shoulderPitchLeft, elbowRollLeft, false);
    shoulderYaw = -1*rotateAxes(shoulderPitch, elbowRoll, true);

    vector<float> hip_angles = extractAng(jointvectorarray[HumanJointVectors::LeftHipToKnee], -1.57, -1);
    hipRollLeft = hip_angles.at(0);
    hipPitchLeft = hip_angles.at(1);

    hip_angles = extractAng(jointvectorarray[HumanJointVectors::RightHipToKnee], -1.57, 1);

    hipRoll = hip_angles.at(0);
    hipPitch = hip_angles.at(1);

    //std::cout<< std::fixed << std::setprecision(3)<<hipPitchLeft<<" "<<std::fixed << std::setprecision(3)<<hipRollLeft<<" "<<
      //  std::fixed << std::setprecision(3)<<hipPitch<<" "<<std::fixed << std::setprecision(3)<<hipRoll<<std::endl;

    kneePitchLeft = -1*(PI - angleBtwVecs(jointvectorarray[HumanJointVectors::LeftHipToKnee], jointvectorarray[HumanJointVectors::LeftKneeToAnkle]));
    kneePitch = PI - angleBtwVecs(jointvectorarray[HumanJointVectors::RightHipToKnee], jointvectorarray[HumanJointVectors::RightKneeToAnkle]);

    //std::cout<< std::fixed << std::setprecision(3)<<kneePitchLeft<<" "<<std::fixed << std::setprecision(3)<<kneePitch<<std::endl;
    torsoBending = PI/2-extractAng(jointvectorarray[HumanJointVectors::NeckToTorso],0,-1).at(0);

    headPitch = PI/2-extractAng(jointvectorarray[HumanJointVectors::HeadToNeck],0,-1).at(0);

    joints[0] = headPitch;
    joints[1] = 0.0;

    joints[2] = shoulderPitchLeft;
    joints[3] = shoulderRollLeft;
    joints[4] = shoulderYawLeft;
    joints[5] = elbowRollLeft;

    joints[6] = shoulderPitch;
    joints[7] = shoulderRoll;
    joints[8] = shoulderYaw;
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

inline Vector3D TrackUserNI::sum (Vector3D vector1, Vector3D vector2){
  Vector3D vector;
  for(int i=0;i<3;i++){
    vector.pointsXYZ[i] = vector1.pointsXYZ[i] + vector2.pointsXYZ[i];
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

