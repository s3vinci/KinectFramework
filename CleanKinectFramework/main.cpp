#include "KinectFramework.h"

int main(int argc, char* argv[]){
	static KFCVWrapper& wrapper = KFCVWrapper::getInstance();
	KFController* joystick = new KFLeanController();
	
	wrapper.setKinectController(*joystick);	
	wrapper.focusNearestUser= true;
	Mat *rgbImage = new Mat(480, 640, CV_8UC3, Scalar::all(0));
	Mat *depthImage = new Mat(480, 640, CV_8UC1, Scalar::all(0));
	Mat *comboImage = new Mat(480, 640, CV_8UC3, Scalar::all(0));
	Mat *rawDepth = new Mat(480, 640, CV_16UC1, Scalar::all(0));

	namedWindow("Joystick");
	namedWindow( "Depth" ); //Ventana de la imagen
	if(!wrapper.init()) exit(50);
	int i =0;
	for(;;){
		if(wrapper.update()){
			wrapper.getDisplayDepth(depthImage);
			cv::imshow("Depth",*depthImage);
			cv::imshow("Joystick", wrapper.m_gestureController->joystickView);
		}
		int k = waitKey(25);
		if(k == 27 ) exit(0);
	}
}

