/*
 * joint_def.h
 *
 *  Created on: Jun 22, 2014
 *      Author: binnur
 */

#ifndef JOINT_DEF_H_
#define JOINT_DEF_H_

class HumanJoints {
public:
enum HumanJs{
	UserCOM,
	Neck,
	Head,
	LeftShoulder,
	LeftElbow,
	LeftHand,
	RightShoulder,
	RightElbow,
	RightHand,
	LeftHip,
	LeftKnee,
	LeftAnkle,
	RightHip,
	RightKnee,
	RightAnkle,
	LeftFoot,
	RightFoot

};
};

class HumanJointVectors {
public:
enum HumanJVectors{
	LeftShoulderToElbow,
	LeftElbowToHand,
	LeftShoulderToHand,
	RightShoulderToElbow,
	RightElbowToHand,
	RightShoulderToHand,
	LeftHipToKnee,
	LeftKneeToAnkle,
	LeftHipToAnkle,
	RightHipToKnee,
	RightKneeToAnkle,
	RightHipToAnkle,
	HeadToNeck
};
};



#endif /* JOINT_DEF_H_ */
