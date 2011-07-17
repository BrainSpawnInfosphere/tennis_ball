#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/CvBridge.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;

sensor_msgs::CvBridge bridge_;

//IplConvKernel *se21;
//IplConvKernel *se11;
//CvMat *mask;
//IplImage *hsv; // used in color space change
//IplImage *hough_in; // binary confidence of tennis balls
//IplImage *labelImg; // blobs drawn on image

//IplImage *img; // pointer from camera or JPEG image read
//IplImage *out; // pointer from process()

// Hue        - color [0,180] in opencv
// Saturation - greyness [0,255] dull to vivid in opencv
// Value      - brightness [0,255] black to very bright color in opencv
//
// Typical tennis ball color (RGB) is (154,205,50) ... opencv is BGR!! 
//CvScalar hsv_lower = cvScalar(0.11*255, 0.60*255, 0.20*255, 0); // H=28.05
//CvScalar hsv_upper = cvScalar(0.14*255, 1.00*255, 1.00*255, 0); // H=35.7
// yellow is H = 60 on a 0-360 scale
// work good
//CvScalar hsv_lower = cvScalar(43.0*179.0/255.0, 30, 60, 0);
//CvScalar hsv_upper = cvScalar(65.0*179.0/255.0, 1.00*255, 200, 0);

// colors to search for
CvScalar hsv_lower = cvScalar(20.0, 50, 50, 0); //20-30
CvScalar hsv_upper = cvScalar(35.0, 255, 255, 0); //25-35
// h = 32 +- 4
// s = 130 +- 20
// v = 100 +- 20
//CvScalar hsv_lower = cvScalar(28.0, 100, 120, 0); 
//CvScalar hsv_upper = cvScalar(36.0, 160, 180, 0);

///////////////////////////////////////////////////////


typedef unsigned int uint;

/**
 * Wrap class cv::KalmanFilter like this to handle it prettier.
 */
class Kalman
{
public:
	
	Kalman(uint m, uint n, float rate);
	~Kalman() { delete filter; }
	
	const cv::Mat& update(const cv::Point2f& p);
	const cv::Mat& update(void);
	
private:
	const uint N; //! dimension of transition matrix: NxN
	const uint M; //! length of measurement vector
	
	cv::KalmanFilter* filter;
	
	uint numActiveFrames;
	uint numInactiveFrames;
};





/**
 * There is a measurement, so update the predicted state, then correct the 
 * measurement with it.
 *
 * @param  DetectionRect&
 * @return statePost&
 */
const cv::Mat& Kalman::update(const cv::Point2f& p)
{
	++numActiveFrames;
	numInactiveFrames = 0;
	
	// Tracking 4 points
	cv::Mat measurement(M, 1, CV_32F);
	measurement.at<float>(0,0) = p.x;
	measurement.at<float>(1,0) = p.y;
	
	filter->predict();
	const cv::Mat& statePost = filter->correct(measurement);
	
	return statePost; // new estimate of state
}

/**
 * There is no measurement, return the predicted state
 *
 * @return statePre&
 */
const cv::Mat& Kalman::update(void)
{
	++numInactiveFrames;
	
	//    std::cout << "Tracking ... " << numInactiveFrames << std::endl;
	
	//if (++numInactiveFrames >= Tracker::NUM_MAX_INACTIVE_FRAMES || !isActive())
	//	return false;
	
	const cv::Mat& statePre = filter->predict();
	
	
	return statePre; // predicted state
}


/**
 * Kalman constructor: Parameters of the filter are set in here.
 * These parameters have a direct effect on the behaviour pf the filter.
 */
Kalman::Kalman(uint m, uint n, float rate) : M(m), N(n)
{
	
	numActiveFrames = 0;
	numInactiveFrames = 0;
	
	// setup kalman filter with a Model Matrix, a Measurement Matrix and no control vars
	filter = new cv::KalmanFilter(N, M, 0);
	
	// [1 0 dt 0]
	// [0 1 0 dt]
	// [0 0 1  0] = A
	// [0 0 0  1]
	
	// transitionMatrix is eye(n,n) by default
	filter->transitionMatrix.at<float>(0,2) = rate; // dt=0.04, stands for the time
	filter->transitionMatrix.at<float>(1,3) = rate; // betweeen two video frames in secs.
	filter->transitionMatrix.at<float>(0,0) = 1.0f;
	filter->transitionMatrix.at<float>(1,1) = 1.0f;
	filter->transitionMatrix.at<float>(2,2) = 1.0f;
	filter->transitionMatrix.at<float>(3,3) = 1.0f;
	
	// measurementMatrix is zeros(n,p) by default
	filter->measurementMatrix.at<float>(0,0) = 1.0f;
	filter->measurementMatrix.at<float>(1,1) = 1.0f;
	//filter->measurementMatrix.at<float>(2,2) = 1.0f;
	//filter->measurementMatrix.at<float>(3,3) = 1.0f;
	
	using cv::Scalar;
	
	// assign a small value to diagonal coeffs of processNoiseCov
	cv::setIdentity(filter->processNoiseCov, Scalar::all(1e-2)); // 1e-2
	
	// Measurement noise is important, it defines how much can we trust to the
	// measurement and has direct effect on the smoothness of tracking window
	// - increase this tracking gets smoother
	// - decrease this and tracking window becomes almost same with detection window
	cv::setIdentity(filter->measurementNoiseCov, Scalar::all(10)); // 1e-1
	cv::setIdentity(filter->errorCovPost, Scalar::all(1));
	
	// we are tracking 4 points, thus having 4 states: corners of rectangle
	filter->statePost.at<float>(0,0) = 0.0f;
	filter->statePost.at<float>(1,0) = 0.0f;
	//filter->statePost.at<float>(2,0) = initRect.x2;
	//filter->statePost.at<float>(3,0) = initRect.y2;
}

Kalman kalman(4,2,1.0f/30.0f);

////////////////////////////////////////////////////////

void stdColor(IplImage *s){
	Mat src = cvarrToMat(s);
	Scalar mean,std;
	meanStdDev(src, mean, std);
	printf("============================\n");
	printf("Hue mean/std: %f %f\n", mean[0], std[0]);
	printf("Sat mean/std: %f %f\n", mean[1], std[1]);
	printf("Val mean/std: %f %f\n", mean[2], std[2]);
}

void printstd(IplImage *s, IplImage *m){
	Mat src = cvarrToMat(s);
	Mat mat = cvarrToMat(m);
	
	Scalar mean,std;
	meanStdDev(src, mean, std, mat);
	
	printf("============================\n");
	printf("Hue mean/std: %f %f\n", mean[0], std[0]);
	printf("Sat mean/std: %f %f\n", mean[1], std[1]);
	printf("Val mean/std: %f %f\n", mean[2], std[2]);
}

// do image proccessing to find color blob
IplImage *process3(IplImage *img)
{
	CvSize size = cvGetSize(img);	
	IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
	CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
	IplImage *hough_in = cvCreateImage(size, 8, 1);
	
	//IplImage *img = _img; //?
	
	// Convert to HSV
	cvCvtColor(img, hsv, CV_BGR2HSV);
	
	IplImage* h = cvCreateImage( cvGetSize(hsv), IPL_DEPTH_8U, 1 );
	cvSplit(hsv,h,NULL,NULL,NULL);
	
	//stdColor(hsv);
	
	// Generate binary mask 
	// \todo make hsv colors inputs [20110326]
	cvInRangeS(hsv, hsv_lower,hsv_upper, mask);
	//cvInRangeS(h, cvScalarAll(29), cvScalarAll(31), mask);
	
	// Perform morphological ops 
	//cvClose(mask, mask, se21);
	//cvOpen(mask, mask, se11);
	
	//cvErode(mask,mask,0, 3);
	//cvDilate( mask,mask,0,5);
	//cvSmooth(mask, mask, CV_GAUSSIAN, 5, 5, 0, 0);
	
	// rename hough_in - why have it? mask is a matrix ...
	cvCopy(mask, hough_in, NULL); // copy mask to image
	
	//printstd(hsv,hough_in);
	
	// add much?
	//cvThreshold(hough_in, hough_in, 5, 255, CV_THRESH_BINARY);
	
	CvMoments mom;
	cvMoments(mask, &mom, 1); // find blobs and treat binary
	double area = cvGetCentralMoment(&mom,0,0);
	
	if(area > 0){ // found tenis ball
		
		double xx = cvGetSpatialMoment(&mom,1,0);
		double yy = cvGetSpatialMoment(&mom,0,1);
		
		float x = xx/area;
		float y = yy/area;
		
		//printf("Ball: %f %f\n",x,y);
		cvCircle( img, cvPoint(x,y),3, CV_RGB(0,255,0), -1, 8, 0 );
		
		Point2f p(x,y);
		
		const cv::Mat& m = kalman.update(p);
		
		p.x = m.at<float>(0,0);
		p.y = m.at<float>(1,0);
		
		cvCircle( img, p,3, CV_RGB(255,0,0), -1, 8, 0 );
	}
	else { // no tennis ball found
		const cv::Mat& m = kalman.update();
		Point2f p;
		p.x = m.at<float>(0,0);
		p.y = m.at<float>(1,0);
		
		cvCircle( img, p,3, CV_RGB(0,0,255), -1, 8, 0 );
	}
	
	
	return hough_in;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		//ROS_INFO("image ...");
		
		// Grab image from bridge to cv_image
		IplImage *cv_image = NULL;
		
		try
		{
			cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& error)
		{
			ROS_ERROR("Can't get image from CvBridge");
			return;
		}
		
		// find tennis ball
		//img = process(cv_image);
		//img = process2(cv_image);
		IplImage *img = process3(cv_image);
		//img = processHough(cv_image);
		
		// debug - display images
		cvShowImage("Image window",cv_image);
		cvShowImage("hsv",img);
		cvWaitKey(10);
		
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tennis_ball");
	ros::NodeHandle n;
	ros::Rate r(30);
	
	cvNamedWindow("Image window");
	cvNamedWindow("hsv");
	
	image_transport::ImageTransport it_(n);
	
	//image_sub_ = it_.subscribe("axis_camera", 1, &ImageConverter::imageCallback, this, image_transport::TransportHints("compressed"));
	image_transport::Subscriber image_sub_ = it_.subscribe("camera", 1, imageCallback, image_transport::TransportHints("compressed"));
		
		
	//CvSize size = cvGetSize(cv_image);		
	//setup(size);
	ROS_INFO("Camera Node setup()");
	
	
	
	// Main Loop -- go until ^C terminates
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	
	cvDestroyWindow("Image window");
	cvDestroyWindow("hsv");
	//cvReleaseImage(&hsv);
	
	return 0;
}
