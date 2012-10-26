#include "KinectFramework.h"


void keyShoot(){
	INPUT input[2];
	memset(input, 0, sizeof(input));
	input[0].type = INPUT_KEYBOARD;

	input[0].ki.wVk = VK_SPACE; // ASCI value of A	
	input[0].ki.dwFlags = 0;
	input[0].ki.time = 0;
	input[0].ki.dwExtraInfo = 0;

	input[1].ki.wVk = VK_SPACE; // ASCI value of A
	input[1].ki.dwFlags = KEYEVENTF_KEYUP;
	input[1].ki.time = 0;
	input[1].ki.dwExtraInfo = 0;
	SendInput(2,input,sizeof(INPUT));
}

void keyLeft(bool type){
	INPUT input;
	//memset(input, 0, sizeof(input));
	input.type = INPUT_KEYBOARD;
	if(type==true){
		//input.ki.wVk = 65; // ASCI value of A
		input.ki.wVk = VK_LEFT;
		input.ki.dwFlags = 0;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}else{
		input.ki.wVk = VK_LEFT; // ASCI value of A
		input.ki.dwFlags = KEYEVENTF_KEYUP;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}
	SendInput(1,&input,sizeof(INPUT));
}

void keyRight(bool type){
	INPUT input;
	//memset(input, 0, sizeof(input));
	input.type = INPUT_KEYBOARD;
	if(type==true){
		//input.ki.wVk = 65; // ASCI value of A
		input.ki.wVk = VK_RIGHT;
		input.ki.dwFlags = 0;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}else{
		input.ki.wVk = VK_RIGHT; // ASCI value of A
		input.ki.dwFlags = KEYEVENTF_KEYUP;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}
	SendInput(1,&input,sizeof(INPUT));
}

void KFController::keyFwd(bool type){
	INPUT input;
	//memset(input, 0, sizeof(input));
	input.type = INPUT_KEYBOARD;
	if(type==true){
		//input.ki.wVk = 65; // ASCI value of A
		input.ki.wVk = VK_UP;
		input.ki.dwFlags = 0;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}else{
		input.ki.wVk = VK_UP; // ASCI value of A
		input.ki.dwFlags = KEYEVENTF_KEYUP;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}
	SendInput(1,&input,sizeof(INPUT));
}

void KFController::keyBack(bool type){
	INPUT input;
	//memset(input, 0, sizeof(input));
	input.type = INPUT_KEYBOARD;
	if(type==true){
		//input.ki.wVk = 65; // ASCI value of A
		input.ki.wVk = VK_DOWN;
		input.ki.dwFlags = 0;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}else{
		input.ki.wVk = VK_DOWN; // ASCI value of A
		input.ki.dwFlags = KEYEVENTF_KEYUP;
		input.ki.time = 0;
		input.ki.dwExtraInfo = 0;
	}
	SendInput(1,&input,sizeof(INPUT));
}



//projective bodies
void KFController::setUserSkels(std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> &userSkeletons){
	userSkels= &userSkeletons;
}
//real bodies
void KFController::setUserSkelsReal(std::map<XnUserID, std::map<XnSkeletonJoint,XnSkeletonJointPosition>> &userSkeletons){
	userSkelsReal= &userSkeletons;
}

void KFController::calculateHumanLength(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel){
	torsoPt = getJointPos(skel, XN_SKEL_TORSO);
	cv::Point3d neckPt = getJointPos(skel, XN_SKEL_NECK);
	cv::Point3d  shoulderPt = getJointPos(skel, XN_SKEL_RIGHT_SHOULDER);
	cv::Point3d  handPt = getJointPos(skel, XN_SKEL_RIGHT_HAND); 
	cv::Point3d  elbowPt = getJointPos(skel, XN_SKEL_RIGHT_ELBOW);

	neckLength = distApart(neckPt, shoulderPt);    // neck to shoulder length
	armLength = distApart(handPt, shoulderPt);     // hand to shoulder length
	lowerArmLength = distApart(handPt, elbowPt);    // hand to elbow length
}
cv::Point3d KFController::getJointPos(std::map<XnSkeletonJoint, XnSkeletonJointPosition> skel, XnSkeletonJoint j){
	if(skel.find(j) == skel.end()) 
		return NULL;
	XnSkeletonJointPosition pos = skel.at(j);

	//if (pos.fConfidence== 0) WTF
	//return NULL; n
	return cv::Point3d(pos.position.X,pos.position.Y,pos.position.Z);
}



float KFController::distApart(cv::Point3d& p1, cv::Point3d& p2)
{
	float dist =sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
	return dist;
}

void KFController::updateController(XnUserID userID){
	currentSkelProj = userSkels->at(userID);
	currentSkelReal = userSkelsReal->at(userID);
	if(currentSkelProj.empty())
		return;
	calculateHumanLength(currentSkelProj);
	processUserGestures();
	drawController();

}

void KFController::leanForward(){
	cv::Point3d torsoPt = getJointPos(currentSkelProj, XN_SKEL_TORSO);
	cv::Point3d headPt = getJointPos(currentSkelProj, XN_SKEL_HEAD);
	cout << " arm length is: " << lowerArmLength <<"\n";
	//cout << "Torso z is : " <<torsoPt.z <<"\n";
	float zDiff = headPt.z - torsoPt.z;
	frontLean = zDiff*-1;
	if (zDiff< -115) {    // head is forward
		if (!isLeanFwd) {			
			isLeanFwd=true;
			//keyFwd(true);
		}
	}  else {   // not forward
		if (isLeanFwd) {
			//watcher.pose(userID, GestureName.LEAN_FWD, false);  // stopped
			isLeanFwd = false;
			//keyFwd(false);
			//	keyShoot(false);
		}
	}
	std::cout << "leaning forward" << isLeanFwd << "\n";
}
void KFController::leanBack(){
}

void KFController::turnLeft(){
	cv::Point3d rightHipPt= getJointPos(currentSkelProj, XN_SKEL_LEFT_HIP);
	cv::Point3d leftHipPt= getJointPos(currentSkelProj, XN_SKEL_RIGHT_HIP);
	float zDiff = leftHipPt.z - rightHipPt.z;
	cout << "TURN LEFT IS "<< zDiff <<"\n";
	if(zDiff > lowerArmLength){
		cout <<"TURNED LEFT";
	}
}

void KFController::turnRight(){
	cv::Point3d rightHipPt= getJointPos(currentSkelProj, XN_SKEL_LEFT_HIP);
	cv::Point3d leftHipPt= getJointPos(currentSkelProj, XN_SKEL_RIGHT_HIP);
	float zDiff = rightHipPt.z - leftHipPt.z;
	cout << "TURN RIGHT IS "<< zDiff <<"\n";
	if(zDiff > lowerArmLength){
		cout <<"TURNED RIGHT";
	}
}


void KFController::jump(){
	cv::Point3d torsoNow = getJointPos(currentSkelProj, XN_SKEL_TORSO);
	//if(abs(torsoNow.z - torsoOld.z) <5)
	//cout << abs(torsoNow.y - torsoOld.y) <<"\n";
	double diff =  abs(torsoNow.y - torsoOld.y);
	if( diff > 40.0){
		keyShoot();
		cout << "I AM JUMPING" <<"\n";
	}
	torsoOld = torsoNow;

}
void KFController::leanLeft(){
	cv::Point3d leftHipPt = getJointPos(currentSkelProj, XN_SKEL_LEFT_HIP);
	cv::Point3d headPt = getJointPos(currentSkelProj, XN_SKEL_HEAD);
	horizontalLean = headPt.x-leftHipPt.x;

	//float zDiff = headPt.z - torsoPt.z;
	if (headPt.x+10 <=leftHipPt.x) {    // head is forward
		
		
		
		if (!isLeanRight) {
			std::cout << "KEY LEFT\n";
			isLeanRight=true;
			keyLeft(true);
		}
	}  else {   // not forward
		if (isLeanRight) {
			//watcher.pose(userID, GestureName.LEAN_FWD, false);  // stopped
			isLeanRight = false;
			keyLeft(false);
		}
	}
	//std::cout << "leaning left" << isLeanLeft << "\n";
}
void KFController::leanRight(){
	cv::Point3d rightHipPt = getJointPos(currentSkelProj, XN_SKEL_RIGHT_HIP);
	cv::Point3d headPt = getJointPos(currentSkelProj, XN_SKEL_HEAD);
	horizontalLean = headPt.x-rightHipPt.x;
	
	
	if (rightHipPt.x <= headPt.x) {    // head is forward
		if (!isLeanLeft) {
			std::cout << "KEY RIGHT\n";
			isLeanLeft=true;
			keyRight(true);
		}
	}  else {   // not forward
		if (isLeanLeft) {
			isLeanLeft = false;
			keyRight(false);
		}
	}
}

void KFController::leftHandUp(){
	cv::Point3d headPt= getJointPos(currentSkelProj, XN_SKEL_HEAD);
	cv::Point3d leftHandPt= getJointPos(currentSkelProj, XN_SKEL_LEFT_HAND);
	if(leftHandPt.y <= headPt.y ){
		cout <<"LEFT HAND IS UP";
	}
}

void KFController::leftHandFwd(){
	cv::Point3d leftHandPt= getJointPos(currentSkelProj, XN_SKEL_LEFT_HAND);
	cv::Point3d shoulderPt= getJointPos(currentSkelProj, XN_SKEL_LEFT_SHOULDER);
	float zDiff = leftHandPt.z - shoulderPt.z; /// zDiff will be negative if hand is toward the camera
	if(zDiff < -1 *(armLength * 0.90)){
		cout <<"L HAND IS FORWARD";
	}
}

void KFController::rightHandUp(){
		cv::Point3d headPt= getJointPos(currentSkelProj, XN_SKEL_HEAD);
	cv::Point3d rightHandPt= getJointPos(currentSkelProj, XN_SKEL_RIGHT_HAND);
	if(rightHandPt.y <= headPt.y ){
		cout <<"RIGHT HAND IS UP";
	}
}

void KFController::rightHandFwd(){
	cv::Point3d rightHandPt= getJointPos(currentSkelProj, XN_SKEL_RIGHT_HAND);
	cv::Point3d shoulderPt= getJointPos(currentSkelProj, XN_SKEL_RIGHT_SHOULDER);
	float zDiff = rightHandPt.z - shoulderPt.z; /// zDiff will be negative if hand is toward the camera
	if(zDiff < -1 *(armLength * 0.90)){
		cout <<" R HAND IS FORWARD";
	}
}