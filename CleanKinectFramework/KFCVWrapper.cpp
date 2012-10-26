#include "KinectFramework.h"
xn::UserGenerator *g_UserGenerator;

void XN_CALLBACK_TYPE KFCVWrapper::User_NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("New user identified: %d\n", user);

	if (generator.GetSkeletonCap().NeedPoseForCalibration())
	{
		generator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
	}
	else
	{
		generator.GetSkeletonCap().RequestCalibration(user, TRUE);
	}
}

void XN_CALLBACK_TYPE KFCVWrapper::User_LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("User %d lost\n", user);
}

void XN_CALLBACK_TYPE UserExit(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("User %d exit\n", user);
}

void XN_CALLBACK_TYPE UserReEnter(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("User %d reenter\n", user);
}

void XN_CALLBACK_TYPE  KFCVWrapper::UserCalibration_CalibrationStart(xn::SkeletonCapability& skeleton, XnUserID user, void* pCookie)
{
	printf("Calibration start for user %d\n", user);
}

void XN_CALLBACK_TYPE  KFCVWrapper::UserCalibration_CalibrationComplete(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* pCookie)
{
	printf("Calibration complete for user %d: %s\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"Success":"Failure");
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		skeleton.StartTracking(user);
	}
	else if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
	{
		printf("Manual abort occurred, stop attempting to calibrate!");
	}
	else if (skeleton.NeedPoseForCalibration())
	{

		g_UserGenerator->GetPoseDetectionCap().StartPoseDetection("Psi", user);
	}
	else
	{
		skeleton.RequestCalibration(user, TRUE);
	}
}

void XN_CALLBACK_TYPE KFCVWrapper::UserPose_PoseDetected(xn::PoseDetectionCapability& poseDetection, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose '%s' detected for user %d\n", strPose, nId);
	g_UserGenerator->GetSkeletonCap().RequestCalibration(nId, FALSE);
	g_UserGenerator->GetPoseDetectionCap().StopPoseDetection(nId);
}

KFCVWrapper::KFCVWrapper() {

	started = false;
	userDetected=false;

}

int KFCVWrapper::init()
{
	//	internalThread_ =boost::thread(&KFCVWrapper::_boostThread, this);
	// internalThread_.detach();
	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal= g_context.Init();
	if(nRetVal!=XN_STATUS_OK){
		return 0;
	}
	g_context.SetGlobalMirror(true); // set mirror
	g_depth.Create(g_context);
	g_image.Create(g_context);
	XnMapOutputMode outputMode;
	outputMode.nFPS = 30;
	outputMode.nXRes = 640;
	outputMode.nYRes = 480;

	g_image.SetMapOutputMode(outputMode);
	g_depth.SetMapOutputMode(outputMode);
	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);


	//g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);
	_rgbImage = new Mat(480, 640, CV_8UC3, Scalar::all(0));
	_comboImage = new Mat(480, 640, CV_8UC3, Scalar::all(0));
	_depthImage = new Mat(480, 640, CV_8UC1, Scalar::all(0));
	_rawDepth = new Mat(480, 640, CV_16UC1, Scalar::all(0));

	_depthColor = new Mat((480, 660, CV_8UC3, Scalar::all(0)));

	joystickImage = Mat(480, 640, CV_8UC3, Scalar::all(65));

	//Create User Generator
	mUserGenerator.Create(g_context);
	g_UserGenerator = &mUserGenerator;
	//Rigister callback functio
	XnCallbackHandle hUserCB;
	mUserGenerator.RegisterUserCallbacks(User_NewUser,User_LostUser,NULL,hUserCB);

	//mUserGenerator.RegisterToUserExit(UserExit, NULL, hUserCB);
	//mUserGenerator.RegisterToUserReEnter(UserReEnter, NULL, hUserCB);
	//Rigister callback functions of skeleton capability

	xn::SkeletonCapability mSC = mUserGenerator.GetSkeletonCap();
	mSC.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);//Only upper body
	XnCallbackHandle hCalibCB;
	p_mSC = &mSC;


	XnCallbackHandle hCalibStart, hCalibComplete;
	XnCallbackHandle hPoseDetected;

	mUserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibStart);
	mUserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibComplete);
	mUserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);

	m_gestureController->setUserSkels(userSkels);
	m_gestureController->setUserSkelsReal(userSkelsReal);

	m_gestureController->depthImage=_depthColor;

	g_context.StartGeneratingAll(); 


	started = true;
	return started;



	return 1;
}

bool KFCVWrapper::update(){

	if (!started) return false;

	XnStatus rc = XN_STATUS_OK;

	const XnDepthPixel* pDepth;

	// Read a new frame
	rc = g_context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return  false;
	}

	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);

	pDepth = g_depthMD.Data();

	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	const XnDepthPixel* pDepthRow = g_depthMD.Data();

	for (XnUInt y = 0; y < g_imageMD.YRes(); ++y){

		const XnRGB24Pixel* pImage = pImageRow;
		const XnDepthPixel* pDepth = pDepthRow;

		for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage,++pDepth){

			_rgbImage->at<Vec3b>(y,x) = Vec3b(pImage->nBlue, pImage->nGreen , pImage->nRed);

			if (*pDepth != 0)

				_rawDepth->at<short>(y,x) = *pDepth;
			else

				_rawDepth->at<short>(y,x) = 0 ;
		}

		pDepthRow += g_depthMD.XRes();
		pImageRow += g_imageMD.XRes();
	}

	double min, max;

	Mat aux(480,640,CV_8UC3);

	minMaxLoc(*_rawDepth, &min, &max,NULL,NULL);

	_rawDepth->convertTo(*_depthImage, CV_8UC1, 255.0/max);

	cvtColor(*_depthImage,aux,CV_GRAY2BGR);

	cvtColor(*_depthImage,*_depthColor,CV_GRAY2RGB);


	//getUser();

	processUsers();

	_rgbImage->copyTo(*_comboImage);
	aux.copyTo(*_comboImage, *_depthImage);

	return true;
}


void KFCVWrapper::getRGB(Mat *rgb){

	_rgbImage->copyTo(*rgb);

}

void KFCVWrapper::getRawDepth(Mat *rawDepth){

	_rawDepth->copyTo(*rawDepth);

}

void KFCVWrapper::getCombo(Mat *combo){

	_comboImage->copyTo(*combo);
}

void KFCVWrapper::getDisplayDepth(Mat *displayDepth){

	_depthColor->copyTo(*displayDepth);
}


void KFCVWrapper::updateJoints(XnUserID userID){
	/*
	cv::Point3f neckPt = getJointPos(skel, XN_SKEL_NECK);
	cv::Point3f  shoulderPt = getJointPos(skel, XN_SKEL_RIGHT_SHOULDER);
	cv::Point3f  handPt = getJointPos(skel, XN_SKEL_RIGHT_HAND); XN_SKEL_HEAD;
	cv::Point3f  elbowPt = getJointPos(skel, XN_SKEL_RIGHT_ELBOW);
	*/
	// vmap<XnSkeletonJoint, XnSkeletonJointPosition> &skel = userSkels.at(userID);
	updateJoint(userID, XN_SKEL_HEAD);
	updateJoint(userID, XN_SKEL_NECK);
	updateJoint(userID, XN_SKEL_TORSO);

	updateJoint(userID, XN_SKEL_RIGHT_SHOULDER);
	updateJoint(userID, XN_SKEL_LEFT_SHOULDER);

	updateJoint(userID, XN_SKEL_RIGHT_HAND);
	updateJoint(userID, XN_SKEL_LEFT_HAND);

	updateJoint(userID, XN_SKEL_LEFT_HIP);
	updateJoint(userID, XN_SKEL_RIGHT_HIP);

	updateJoint(userID, XN_SKEL_RIGHT_KNEE);
	updateJoint(userID, XN_SKEL_RIGHT_FOOT);

	updateJoint(userID, XN_SKEL_LEFT_KNEE);
	updateJoint(userID, XN_SKEL_LEFT_FOOT);

	updateJoint(userID, XN_SKEL_LEFT_ELBOW);
	updateJoint(userID,XN_SKEL_RIGHT_ELBOW);

}
void KFCVWrapper::updateJoint(XnUserID userID, XnSkeletonJoint joint){
	//map<XnSkeletonJoint, XnSkeletonJointPosition> skel
	//if(skel.find(j) == skel.end()) 
	//os = skelCap.getSkeletonJointPosition(userID, joint);

	XnSkeletonJointPosition myjoint;
	XnPoint3D proj_point;
	mUserGenerator.GetSkeletonCap().GetSkeletonJointPosition(userID,joint,myjoint);
	g_depth.ConvertRealWorldToProjective(1,&myjoint.position,&proj_point);

	userSkelsReal.at(userID)[joint] =myjoint;

	//cout << "real position:" << myjoint.position.X <<"\n";
	//cout << "proj position:" << proj_point.X << "\n";
	XnSkeletonJointPosition joint_projective;
	joint_projective.fConfidence=0;
	joint_projective.position=proj_point;
	userSkels.at(userID)[joint] =joint_projective;
}


void KFCVWrapper::processUsers(){

	XnUInt16 nUsers = mUserGenerator.GetNumberOfUsers(); // Get User Number
	xn::SkeletonCapability p_mSC =mUserGenerator.GetSkeletonCap();
	if(nUsers > 0)
	{
		//cout<< "User detected love";
		xn::SkeletonCapability p_mSC =mUserGenerator.GetSkeletonCap();
		XnUserID *aUserID = new XnUserID;
		XnUInt16 nUsers = mUserGenerator.GetNumberOfUsers();
		mUserGenerator.GetUsers(aUserID,nUsers); //not nUsers


		for(int i = 0; i<nUsers;++i)
		{
			if(p_mSC.IsTracking(aUserID[i]))
			{
				map<XnSkeletonJoint, XnSkeletonJointPosition> newuser;
				userSkels[aUserID[i]] = newuser;
				userSkelsReal[aUserID[i]] = newuser;
				updateJoints(aUserID[i]);		
				if(focusNearestUser){
					//if(!userDetected){
					this->nearUserDetect();
					//}
					//	else{
					if(userDetected){
						m_gestureController->updateController(*this->currentUser);

						drawStickFigure();
					}
				}
				else{ //!focus on narest user

					if(!userDetected){
						this->nearUserDetect();
					}
					else{
						if(userDetected){
							m_gestureController->updateController(*this->currentUser);

							drawStickFigure();
						}
					}

				}

			}
		}
	}

}
	void KFCVWrapper::drawStickFigure()
	{
		XnSkeletonJointPosition& neckPos = userSkels[*this->currentUser].at(XN_SKEL_NECK);
		XnSkeletonJointPosition& headPos = userSkels[*this->currentUser].at(XN_SKEL_HEAD);

		XnSkeletonJointPosition& torsoPos = userSkels[*this->currentUser].at(XN_SKEL_TORSO);

		XnSkeletonJointPosition& leftShPos = userSkels[*this->currentUser].at(XN_SKEL_LEFT_SHOULDER);
		XnSkeletonJointPosition& rightShPos = userSkels[*this->currentUser].at(XN_SKEL_RIGHT_SHOULDER);

		XnSkeletonJointPosition& leftElbPos = userSkels[*this->currentUser].at(XN_SKEL_LEFT_ELBOW);
		XnSkeletonJointPosition& rightElbPos = userSkels[*this->currentUser].at(XN_SKEL_RIGHT_ELBOW);

		XnSkeletonJointPosition& leftHand = userSkels[*this->currentUser].at(XN_SKEL_LEFT_HAND);
		XnSkeletonJointPosition& rightHand = userSkels[*this->currentUser].at(XN_SKEL_RIGHT_HAND);

		XnSkeletonJointPosition& leftHipPos = userSkels[*this->currentUser].at(XN_SKEL_LEFT_HIP);
		XnSkeletonJointPosition& rightHipPos = userSkels[*this->currentUser].at(XN_SKEL_RIGHT_HIP);

		XnSkeletonJointPosition& leftKneePos = userSkels[*this->currentUser].at(XN_SKEL_LEFT_KNEE);
		XnSkeletonJointPosition& rightKneePos = userSkels[*this->currentUser].at(XN_SKEL_RIGHT_KNEE);

		XnSkeletonJointPosition& lFoot = userSkels[*this->currentUser].at(XN_SKEL_LEFT_FOOT);
		XnSkeletonJointPosition& rFoot = userSkels[*this->currentUser].at(XN_SKEL_RIGHT_FOOT);

		cv::line(*_depthColor,cv::Point(neckPos.position.X,neckPos.position.Y),cv::Point(headPos.position.X,headPos.position.Y),cv::Scalar(123,100,50),4);

		cv::line(*_depthColor,cv::Point(leftShPos.position.X,leftShPos.position.Y),cv::Point(torsoPos.position.X,torsoPos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(rightShPos.position.X,rightShPos.position.Y),cv::Point(torsoPos.position.X,torsoPos.position.Y),cv::Scalar(123,100,50),4);

		cv::line(*_depthColor,cv::Point(neckPos.position.X,neckPos.position.Y),cv::Point(leftShPos.position.X,leftShPos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(leftShPos.position.X,leftShPos.position.Y),cv::Point(leftElbPos.position.X,leftElbPos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(leftElbPos.position.X,leftElbPos.position.Y),cv::Point(leftHand.position.X,leftHand.position.Y),cv::Scalar(123,100,50),4);

		cv::line(*_depthColor,cv::Point(neckPos.position.X,neckPos.position.Y),cv::Point(rightShPos.position.X,rightShPos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(rightShPos.position.X,leftShPos.position.Y),cv::Point(rightElbPos.position.X,rightElbPos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(rightElbPos.position.X,rightElbPos.position.Y),cv::Point(rightHand.position.X,rightHand.position.Y),cv::Scalar(123,100,50),4);

		cv::line(*_depthColor,cv::Point(leftHipPos.position.X,leftHipPos.position.Y),cv::Point(leftKneePos.position.X,leftKneePos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(leftKneePos.position.X,leftKneePos.position.Y),cv::Point(lFoot.position.X,lFoot.position.Y),cv::Scalar(123,100,50),4);

		cv::line(*_depthColor,cv::Point(rightHipPos.position.X,rightHipPos.position.Y),cv::Point(rightKneePos.position.X,rightKneePos.position.Y),cv::Scalar(123,100,50),4);
		cv::line(*_depthColor,cv::Point(rightKneePos.position.X,rightKneePos.position.Y),cv::Point(rFoot.position.X,rFoot.position.Y),cv::Scalar(123,100,50),4);
	}


	void KFCVWrapper::nearUserDetect()
	{
		double zNearest=9999;
		//std::map<int, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> userSkels;
		std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>>::iterator iter;
		for(iter = userSkels.begin();iter!=userSkels.end();iter++)
		{
			//std::cout << "VERIFYING USER\n";
			XnSkeletonJointPosition neckPos = iter->second.at(XN_SKEL_NECK);
			cout << "position is " << neckPos.position.Z;
			if(neckPos.position.Z > 0){ // we have the position
				if(neckPos.position.Z < zNearest){
					zNearest= neckPos.position.Z;
					currentUser= &iter->first;
					//std::cout << "user id is :" <<iter->first;
					userDetected=true;
				}
			}
		}
	}
