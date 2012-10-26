#include "KinectFramework.h"
void KFLeanController::drawController()
{
	joystickView.setTo(0);
	cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			cv::rectangle(joystickView,cv::Rect(i*220,j*160,220,160),cv::Scalar(255,155,0));

	//cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	//cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	//cv::rectangle(joystickView,cv::Rect(320-50/2,240-50/2,50,50),cv::Scalar(255,155,0));

	cv::circle(joystickView,cv::Point(320-20/2,240-20/2-fLean),20,cv::Scalar(0,60,60),3);

}
void KFLeanController::processUserGestures()
{
	//cv::Point3d leftElb = getJointPos(currentSkelReal, XN_SKEL_LEFT_ELBOW);
	//cv::Point3d leftHand = getJointPos(currentSkelReal, XN_SKEL_LEFT_HAND);
	//float wow = distApart(leftElb,leftHand);
	//cout << "HAND SIZE IS " << wow <<"\n";
	leanLeft();
	leanRight();
	jump();
	cv::Point3d leftHipPt = getJointPos(currentSkelReal, XN_SKEL_LEFT_HIP);
	cv::Point3d headPt = getJointPos(currentSkelReal, XN_SKEL_HEAD);
	double leftLean = headPt.x-leftHipPt.x;
	//cout << "LEFT LEAN IS " <<leftLean <<"\n";
	cv::Point3d torsoPt = getJointPos(currentSkelReal, XN_SKEL_TORSO);

	fLean = (headPt.z - torsoPt.z)*-1 ;//+ lowerArmLength;
	//std:: cout <<"leaning forward is " << fLean <<"\n";
	if (fLean > 100) {    // head is forward
		if (!isLeanFwd) {		
			std::cout << "KEY FORWARD\n";
			isLeanFwd=true;
			keyFwd(true);

		}
	}  else {   // not forward
		if (isLeanFwd) {
			isLeanFwd = false;
			keyFwd(false);			
		}
	}
	if (fLean < -120) {    // head is forward
		if (!isLeanBack) {			
			isLeanBack=true;
			keyBack(true);
		}
	}  else {   // not forward
		if (isLeanBack) {

			isLeanBack = false;
			keyBack(false);

		}
	}	
	cv::Point3d rightHipPt = getJointPos(currentSkelReal, XN_SKEL_RIGHT_HIP);
}
