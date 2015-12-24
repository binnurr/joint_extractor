/*
 * TrackUserNI.h
 *
 *  Created on: Oct 1, 2015
 *      Author: binnur
 */

#ifndef TRACKUSERNI_H_
#define TRACKUSERNI_H_

#include <OpenNI.h>
#include <NiTE.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "joint_def.h"


struct Vector3D{
	float pointsXYZ [3] ;
	Vector3D(){}
	Vector3D(float x, float y, float z){
	  pointsXYZ[0] = x;
	  pointsXYZ[1] = y;
	  pointsXYZ[2] = z;
	}
};

using namespace std;
class TrackUserNI
{
public:
	TrackUserNI();
	virtual ~TrackUserNI();

	int numJoints;
	vector<float> * minLimitsNI;
	vector<float> * maxLimitsNI;
	vector<float> joints;
  Vector3D * jointarray;
  ofstream outfile;

	// Prime Sense Variables and objects begin
	nite::UserTracker userTracker;
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status niteRc;
	openni::Status openniRc;
	openni::Device device;

	//initiation for angles
	float shoulderPitch;
	float shoulderYaw;
	float shoulderRoll;
	float elbowYaw;
	float elbowRoll;
	float shoulderPitchLeft;
	float shoulderYawLeft;
	float shoulderRollLeft;
	float elbowYawLeft;
	float elbowRollLeft;
	float head_front;
	float head_side;
	float hipRoll;
	float hipPitch;
	float hipRollLeft;
	float hipPitchLeft;
	float kneePitch;
	float kneePitchLeft;
	float torsoBending;
	float headPitch;

	Vector3D * jointvectorarray;

	nite::SkeletonJoint rightshoulder;
	nite::SkeletonJoint rightelbow;
	nite::SkeletonJoint righthand;
	nite::SkeletonJoint righthip;
	nite::SkeletonJoint rightknee;
	nite::SkeletonJoint rightfoot;
	nite::SkeletonJoint lefthip;
	nite::SkeletonJoint leftknee;
	nite::SkeletonJoint leftfoot;
	nite::SkeletonJoint leftshoulder;
	nite::SkeletonJoint leftelbow;
	nite::SkeletonJoint lefthand;
	nite::SkeletonJoint neck;
	nite::SkeletonJoint head;
	nite::SkeletonJoint torso;

	void init(int argc, char **argv, vector<float> *minLimits, vector<float> *maxLimits);
	void waitAndUpdateAll();
	vector<float> trackUser();
	vector<float> extractAng(Vector3D bone, float rotAng, int right);
	vector<Vector3D> rodrigues_rot(vector<Vector3D> vectors, Vector3D rot, float theta);
	void clipJoints(vector<float> & joints);
	Vector3D convertPoint3ftoVector3D (const nite::Point3f& point);
	int isTrackingUser();
	int updateUserState(const nite::UserData& user, unsigned long long ts);
	Vector3D extendVector3D (Vector3D bone, float length);
	float rotateAxes (float pitch, float elbowRoll, bool rightSide);
	void printVec (string name, Vector3D v);
	//float clipJointAngle(float angle, String )

	inline float vectorDistance(Vector3D vector1, Vector3D vector2);
	inline float vectorNorm (Vector3D vector1);
	inline void add (Vector3D &vector, float op);
	inline void div (Vector3D &vector, float op);
	inline Vector3D mul (Vector3D vector, float op);
	inline Vector3D sub (Vector3D vector1, Vector3D vector2);
	inline Vector3D sum (Vector3D vector1, Vector3D vector2);
	inline float dot (Vector3D vector1, Vector3D vector2);
	inline Vector3D cross (Vector3D k, Vector3D v);
	inline float angleBtwVecs (Vector3D k, Vector3D v);

};

#endif /* TRACKUSERNI_H_ */
