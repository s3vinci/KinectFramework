#ifndef KINECTFRAMEWORK_H_INCLUDED
#define KINECTFRAMEWORK_H_INCLUDED

#include <XnOpenNI.h>
#include <XnList.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/highgui/highgui.hpp"
#include <map>
#include <windows.h>


using namespace xn;
using namespace cv;
using namespace std;

static const float NECK_LEN = 50.0f;
static const float LOWER_ARM_LEN = 150.0f;
static const float ARM_LEN = 400.0f;

/*
This is the basic discrete controller
*/




class KFController
{
public:
	float zMovement,xMovement;
	bool moveFwd,moveLeft,moveRight;

	void keyFwd(bool);
	void keyBack(bool);
	void leanForward();
	void leanBack();
	void leanLeft();
	void leanRight();
	void jump();
	void turnLeft();
	void turnRight();

	void rightHandUp();
	void rightHandFwd();

	void leftHandUp();
	void leftHandFwd();

	cv::Point3d torsoPt;
	cv::Mat* depthImage;
	cv::Mat joystickView;
	float neckLength,lowerArmLength,armLength;
	KFController(){
		neckLength = NECK_LEN;            
		lowerArmLength = LOWER_ARM_LEN;   
		armLength = ARM_LEN;  

		joystickView = Mat(480, 640, CV_8UC3, Scalar::all(65));
	}
	std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> *userSkels;
	std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> *userSkelsReal;
	std::map<XnSkeletonJoint, XnSkeletonJointPosition> currentSkelProj;
	std::map<XnSkeletonJoint, XnSkeletonJointPosition> currentSkelReal;

	void calculateHumanLength(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel);
	virtual void drawController()=0;
	virtual void processUserGestures()=0;

	void setUserGenerator(xn::UserGenerator& usergen);
	void setUserSkels(std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> &userSkeletons);
	void setUserSkelsReal(std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> &userSkeletons);	
	cv::Point3d getJointPos(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel, XnSkeletonJoint j);
	float distApart(cv::Point3d& p1, cv::Point3d& p2);

	void updateController(XnUserID userID);
protected:
	double horizontalLean;
	double frontLean;
	bool isLeanFwd,isLeanLeft,isLeanRight,isTurnLeft,isTurnRight,isLeanBack;
	cv::Point3d torsoOld;
};

class KFLocationController : public KFController
{
public:

	float zOffset, xOffset;
	cv::Point3d initialPosition;
	bool startPosition, bothHandsUp;
	KFLocationController() : KFController(){
		startPosition=false;
		bothHandsUp=false;
	}
	void drawController();
	void processUserGestures();
	void calculateHumanLength(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel);
};

class KFHandSpeedController : public KFController
{
public: 
	float oldDiff,newDiff;
	float differences;
	bool handsNear;
	void calculateHumanLength(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel);
	cv::Point3d lHand, rHand;
	bool startPosition;
	KFHandSpeedController() : KFController(){
		handsNear=false;
	}
	void drawController();
	void processUserGestures();
};

class KFLeanController : public KFController
{
public:
	float fLean;
	void drawController();
	void processUserGestures();
	KFLeanController() : KFController(){
		isLeanFwd= false;
		isLeanLeft=false;
		isLeanRight=false;
		torsoOld = cv::Point3d(0.0f,0.0f,0.0f);
	}
private:
	
};


class KFCVWrapper
{
public:
	bool focusNearestUser;
	bool runKinect;
	KFController* m_gestureController;
	const XnUserID* currentUser;
	KFCVWrapper();
	int init();


	static KFCVWrapper& getInstance()
	{
		static KFCVWrapper singleton; // calls constructor
		return singleton;
	}	

	std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> userSkels;
	std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> userSkelsReal;
	void updateJoints(XnUserID userID);
	void updateJoint(XnUserID userID, XnSkeletonJoint joint);

	bool update();
	void getRGB(Mat *rgb);
	void getRawDepth(Mat *rawDepth);
	void getCombo(Mat *combo);
	void getDisplayDepth(Mat *displayDepth);

	void setKinectController(KFController& controller){
		m_gestureController = &controller;
	}

private:
	void drawStickFigure();
	
	//static KFCVWrapper *_instance;
	void processUsers();
	void nearUserDetect();
	bool userDetected;

	bool started;
	Context g_context;
	xn::UserGenerator mUserGenerator;
	xn::SkeletonCapability* p_mSC;

	ScriptNode g_scriptNode;
	DepthGenerator g_depth;
	ImageGenerator g_image;

	DepthMetaData g_depthMD;
	ImageMetaData g_imageMD;

	Mat *_rgbImage;
	Mat *_depthImage;
	Mat *_depthColor;
	Mat *_comboImage;
	Mat *_rawDepth;
	Mat joystickImage;

	 static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
     static void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
     static void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
     static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
     static void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus calibrationError, void* pCookie);
};

class KFTurnController : public KFController
{
public:
	void drawController();
	void processUserGestures();
	KFTurnController() : KFController(){
		moveFwd=moveLeft=moveRight=false;
		isLeanFwd= false;
		isLeanLeft=false;
		isLeanRight=false;
		torsoOld = cv::Point3d(0.0f,0.0f,0.0f);
	}
private:

};


/*
class KFTurnController : KFControllerInterface
{
void drawController();
void processUserGestures();
}

class KSteeringWheelController : KFControllerInterface
{
void drawController();
void processUserGestures();
}
*/
#endif