#include "KinectFramework.h"

void KFHandSpeedController::drawController()
{
}

void KFHandSpeedController::processUserGestures()
{

}

void KFHandSpeedController::calculateHumanLength(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel){
	rHand = getJointPos(skel, XN_SKEL_LEFT_HAND);
	lHand = getJointPos(skel, XN_SKEL_RIGHT_HAND);
	
	oldDiff = newDiff;
	newDiff= lHand.y ;
	cout << "TIME DERIVATIVE IS: " <<  abs(oldDiff-newDiff) << "\n";
	if(rHand.x-lHand.x < 50){
	}
		//cout << " STARTING TRACKING";
}

