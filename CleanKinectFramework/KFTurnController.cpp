#include "KinectFramework.h"
void KFTurnController::drawController()
{
	joystickView.setTo(0);
	cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			cv::rectangle(joystickView,cv::Rect(i*220,j*160,220,160),cv::Scalar(255,155,0));

	//cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	//cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	//cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	//cv::circle(joystickView,cv::Point(320-20/2,240-20/2-fLean),20,cv::Scalar(0,60,60),3);

}
void KFTurnController::processUserGestures()
{

	cv::Point3d leftShPt = getJointPos(currentSkelReal, XN_SKEL_LEFT_SHOULDER);
	cv::Point3d rightShPt = getJointPos(currentSkelProj, XN_SKEL_RIGHT_SHOULDER);

	cv::Point3d headPt = getJointPos(currentSkelReal, XN_SKEL_HEAD);
	xMovement = rightShPt.z-leftShPt.z;

	std::cout << "SHOULDER LEAN IS " <<xMovement <<"\n";
	//std::cout << "LEFT LEAN IS " <<sideLean <<"\n";
	cv::Point3d torsoPt = getJointPos(currentSkelReal, XN_SKEL_TORSO);

	zMovement = (headPt.z - torsoPt.z)*-1 ;
	//std::cout << "REAL FRONT MOVEMENT IS : "<<zMovement <<"\n";
		if (xMovement < -100) {    // head is forward
		if (!moveLeft) {
			std::cout <<"MOVING FORWARD" <<"\n";
			moveLeft=true;
			//keyFwd(true);
		}
	}  else {   // not forward
		if (moveLeft) {
			moveLeft = false;
			//keyFwd(false);			
		}
	}
		if (xMovement> 180) {    // head is forward
		if (!moveRight) {
			std::cout <<"MOVING RIGHT" <<"\n";
			moveRight=true;
		}
	}  else {   // not forward
		if (moveRight) {
			moveRight = false;
		}
	}
	if (zMovement > 100) {    // head is forward
		if (!moveFwd) {
			std::cout <<"MOVING FORWARD" <<"\n";
			moveFwd=true;
			//keyFwd(true);
		}
	}  else {   // not forward
		if (moveFwd) {
			moveFwd = false;
			//keyFwd(false);			
		}
	}
	if (zMovement < -120) {    // head is forward
		if (!isLeanBack) {			
			isLeanBack=true;
			//keyBack(true);
		}
	}  else {   // not forward
		if (isLeanBack) {
			isLeanBack = false;
			//keyBack(false);

		}
	}	
}
