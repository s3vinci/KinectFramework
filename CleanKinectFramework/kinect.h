#ifndef __QKINECTWRAPPER_H
#define __QKINECTWRAPPER_H
#include <XnOpenNI.h>
#include <XnList.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/highgui/highgui.hpp"
#include <windows.h>


namespace QKinect
{
	/**
	\brief Status of the body tracking
	**/
	enum CalibrationStatus
	{
		CalibrationStart = 0,
		CalibrationEndSuccess = 1,
		CalibrationEndFail = 2
	};

	/**
	\brief Status of the body tracking
	**/
	enum KinectStatus
	{
		Idle = 0,
		Initializing = 1,
		OkRun = 2,
		ErrorStop = 3
	};

	/**
	\brief Status of the body tracking
	**/
	enum BodyStatus
	{
		Tracking = 0,
		Calibrating = 1,
		LookingForPose = 2
	};
	/**
	\brief Enumeration of body joints
	**/
	enum BodyJoints
	{
		Head = 0,
		Neck = 1,
		LeftShoulder = 2,
		LeftElbow = 3,
		LeftHand = 4,
		RightShoulder = 5,
		RightElbow = 6,
		RightHand = 7,
		Torso = 8,
		LeftHip = 9,
		LeftKnee = 10,
		LeftFoot = 11,
		RightHip = 12,
		RightKnee = 13,
		RightFoot = 14,
	};

	/**
	\brief Body structure containing the user id, status, skeleton 3D coords, and skeleton projection
	**/
	typedef struct {
		XnUserID id;
		// Status: 0=tracking, 1=calibrating, 2=looking for pose
		BodyStatus status;
		// Center of mass and its projection.
		XnPoint3D com, proj_com;
		// Whether the com projection is valid
		bool proj_com_valid;
		// Whether the body tracked
		//bool tracked;
		// 3D coordinates of the joints
		XnSkeletonJointPosition joints[15];
		// Projected coordinates of the joints
		XnPoint3D proj_joints[15];
		// Whether the projection is valid / has been computed. Projections are not computed if the joint is too uncertain.
		bool proj_joints_valid[15];
	} Body;

	/**
	\brief Bodies: vector of Body - to hold the data of multiple tracked persons
	**/
	typedef std::vector<Body> Bodies;

	// static const float NECK_LEN = 50.0f;
	// static const float LOWER_ARM_LEN = 150.0f;
	// static const float ARM_LEN = 400.0f;
	/*
	class KFController{
	public:
	float neckLength,lowerArmLength,armLength;
	float distApart(XnPoint3D p1, XnPoint3D p2);
	//  void setUserSkels(Bodies& bodies);
	void processGestures(Bodies& bodies);
	KFController(){
	neckLength = NECK_LEN;
	lowerArmLength = LOWER_ARM_LEN;
	armLength = ARM_LEN;
	}
	};
	*/
	class QKinectWrapper
	{
	public:

		QKinectWrapper();
		virtual ~QKinectWrapper();
		void run();

		void start();
		void stop();


		bool isRunning();
		bool isStopped();

		// void setSkeletonPen(const QPen & pen);
		void setTextPen(const QPen & pen);
		void setFont(const QFont &font);
		void setDisplayInfoDepth(bool draw);
		void setDisplayInfoImage(bool draw);
		void setDisplaySkeletonDepth(bool draw);
		void setDisplaySkeletonImage(bool draw);
		unsigned getFrameID();
		double getTimestamp();
		void getCameraDepthBodies(QImage &camera,QImage &depth,Bodies &bodies,double &ts,unsigned &fid);
		void getCameraDepth(QImage &camera,QImage &depth,double &ts,unsigned &fid);
		cv::Mat getDepth();
		cv::Mat getCamera();

	private:
		int a;
	}
}
#endif // __QKINECTWRAPPER_H

