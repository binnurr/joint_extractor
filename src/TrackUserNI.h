/*
 * TrackUserNI.h
 *
 *  Created on: Aug 26, 2013
 *      Author: binnur
 */

#ifndef TRACKUSERNI_H_
#define TRACKUSERNI_H_

#include <OpenNI.h>
#include <NiTE.h>
#include <vector>

#include <string>
#include "joint_def.h"

struct Vector3D{
	float pointsXYZ [3] ;
	float orientations [3];
};

class TrackUserNI
{
public:
	TrackUserNI();
	virtual ~TrackUserNI();

	void init(int argc, char **argv);

	void waitAndUpdateAll();

	std::vector<float> trackUser();
	void trackUser2();
	int isTrackingUser();
	Vector3D * jointarray;
	Vector3D * orientationarray;

private:

	inline float angle(float x,float y, float z);
	inline float angleZXY(Vector3D vector1);
	inline float angleXYZ(Vector3D vector1);
	inline float angleYXZ(Vector3D vector1);
	inline float vectorNorm (Vector3D vector1);
	inline Vector3D takeMinusVector (Vector3D vector1);
	inline float vectorDistance(Vector3D vector1, Vector3D vector2);
	int updateUserState(const nite::UserData& user, unsigned long long ts);

	// Prime Sense Variables and objects begin
	nite::UserTracker userTracker;
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status niteRc;
	openni::Status openniRc;
	openni::Device device;



	//initiation for angles
	float shoulderPitch;
	float shoulderRoll;
	float elbowYaw;
	float elbowRoll;
	float shoulderPitchLeft;
	float shoulderRollLeft;
	float elbowYawLeft;
	float elbowRollLeft;
	float head_front;
	float head_side;
	float anglebetweenelLeft;
	float anglebetweenel;
	float hip;
	float hip_angle;

	float rightanklepitch;
	float leftanklepitch;

	float rshoulderXZ;
	float rshoulderYZ;
	float rshoulderXY;
	float lshoulderXZ;
	float lshoulderYZ;
	float lshoulderXY;
	float relbowXZ;
	float relbowYZ;
	float relbowXY;
	float lelbowXZ;
	float lelbowYZ;
	float lelbowXY;
	float rwristXZ;
	float rwristYZ;
	float rwristXY;
	float lwristXZ;
	float lwristYZ;
	float lwristXY;

	float hiptokneeright_XY;
	float hiptokneeright_YZ;
	float hiptokneeright_XZ;
	float kneetoankleright_XY;
	float kneetoankleright_YZ;
	float kneetoankleright_XZ;
	float betweenknee;

	float Lhiptokneeright_XY;
	float Lhiptokneeright_YZ;
	float Lhiptokneeright_XZ;
	float Lkneetoankleright_XY;
	float Lkneetoankleright_YZ;
	float Lkneetoankleright_XZ;
	float Lbetweenknee;


	Vector3D * jointvectorarray;

	nite::SkeletonJoint rightshoulder;
	nite::SkeletonJoint rightelbow;
	nite::SkeletonJoint righthand;
	nite::SkeletonJoint righthip;
	nite::SkeletonJoint rightknee;
	nite::SkeletonJoint rightankle;
	nite::SkeletonJoint rightfoot;
	nite::SkeletonJoint lefthip;
	nite::SkeletonJoint leftknee;
	nite::SkeletonJoint leftankle;
	nite::SkeletonJoint leftfoot;
	nite::SkeletonJoint leftshoulder;
	nite::SkeletonJoint leftelbow;
	nite::SkeletonJoint lefthand;
	nite::SkeletonJoint neck;
	nite::SkeletonJoint head;
	nite::SkeletonJoint userPoint;

};




#endif /* TRACKUSERNI_H_ */
