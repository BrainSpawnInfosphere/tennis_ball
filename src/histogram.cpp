#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/CvBridge.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;

sensor_msgs::CvBridge bridge_;

// colors to search for
CvScalar hsv_lower = cvScalar(20.0, 50, 50, 0); //20-30
CvScalar hsv_upper = cvScalar(35.0, 255, 255, 0); //25-35
// h = 32 +- 4
// s = 130 +- 20
// v = 100 +- 20

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

class Histogram1D {
private:
	int histSize[1]; // number of bins
	float hranges[2]; // min and max pixel values
	const float* ranges[1];
	int channels[1]; // only 1 channel
	
public:
	Histogram1D(){
		// Prepare arguments for 1D histogram
		histSize[0] = 256; // grey scale
		hranges[0] = 0.0; // min
		hranges[1] = 255.0; // max
		ranges[0] = hranges; // why redo this?
		channels[0] = 0; // by default we look at channel 0 ... not sure this 
							  // makes sense
	}
	
	// Computes the 1 D histogram
	cv::MatND getHistogram(const cv::Mat &image){
		cv::MatND hist;
		cv::calcHist(&image,
						 1,		// histogram for 1 image only
						 channels,	// the channel used
						 cv::Mat(),		//no mask is used
						 hist,			// the resulting hist
						 1,				// it is a 1D histogram
						 histSize,		// number of bins
						 ranges			// pixel value range
						 );
		return hist;
	}
	
};

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


// using overloaded operators
void colorReduce13(cv::Mat &image, int div=64) {
	
	int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	// mask used to round the pixel value
	uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
	
	// perform color reduction
	image=(image&cv::Scalar(mask,mask,mask))+cv::Scalar(div/2,div/2,div/2);
}

// do image proccessing to find color blob
void process( cv::Mat& in, cv::Mat& out)
{	
	cv::Mat hsv(in.cols,in.rows,CV_8UC3);
	cv::Mat mask(in.cols,in.rows,CV_8UC1);
	cv::Mat h(in.cols,in.rows,CV_8UC1);
	
	// Convert to HSV
	cvtColor(in, hsv, CV_BGR2HSV);
    
	//split(hsv,h,NULL,NULL,NULL);
	
	
	// Generate binary mask 
	// \todo make hsv colors inputs [20110326]
	inRange(hsv, hsv_lower,hsv_upper, mask);
	
	// Perform morphological ops 
	erode(mask,mask, cv::Mat()); // get rid of "noise"
	dilate(mask,mask,cv::Mat()); // fill in holes
	//cvSmooth(mask, mask, CV_GAUSSIAN, 5, 5, 0, 0);
	
	// add much?
	//cvThreshold(hough_in, hough_in, 5, 255, CV_THRESH_BINARY);
	
	Moments mom = moments(mask,true);
	double area = mom.m00; //cvGetCentralMoment(&mom,0,0);
	
	if(area > 0){ // found something
		
		double xx = mom.m10; //cvGetSpatialMoment(&mom,1,0);
		double yy = mom.m01; //cvGetSpatialMoment(&mom,0,1);
		
		float x = xx/area;
		float y = yy/area;
		
		//printf("Ball: %f %f\n",x,y);
		out = mask;
		circle( in, cvPoint(x,y),3, CV_RGB(0,255,0), -1, 8, 0 );
		
		Point2f p(x,y);
		
		const cv::Mat& m = kalman.update(p);
		
		p.x = m.at<float>(0,0);
		p.y = m.at<float>(1,0);
		
		circle( in, p,3, CV_RGB(255,0,0), -1, 8, 0 );
	}
	else { // no tennis ball found
		const cv::Mat& m = kalman.update();
		Point2f p;
		p.x = m.at<float>(0,0);
		p.y = m.at<float>(1,0);
		
		out = mask;
		circle( in, p,3, CV_RGB(0,0,255), -1, 8, 0 );
	}
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		//ROS_INFO("image ...");
		
		// Grab image from bridge to cv_image
		IplImage *ipl = NULL;
		
		try
		{
			ipl = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& error)
		{
			ROS_ERROR("Can't get image from CvBridge");
			return;
		}
		
		// find tennis ball
		cv::Mat image(ipl);
		cv::Mat cv_image(ipl);
		cv::Mat result(image.rows,image.cols,CV_8UC1);
		//cv_image = process(cv_image);
		
		colorReduce13(image,16);
		//colorReduce13(cv_image);
		
		process(cv_image,result);
		
		// debug - display images
        cv::Mat gray(image.rows,image.cols,CV_8UC1);
        cv::cvtColor(image, gray, CV_BGR2GRAY);
        cv::Mat edges(gray);
        
        //cv::Canny(gray,edges,125,255,cv::THRESH_BINARY_INV);
        cv::Canny(gray,edges,125,255);
        
		imshow("Image window",image);
		imshow("result - mask",result);
		waitKey(10);
		
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tennis_ball");
	ros::NodeHandle n;
	ros::Rate r(30);
	
	namedWindow("Image window",1);
	namedWindow("hsv",1);
	
	image_transport::ImageTransport it_(n);
	
	image_transport::Subscriber image_sub_ = it_.subscribe("camera", 1, imageCallback, image_transport::TransportHints("compressed"));
		
		
	//CvSize size = cvGetSize(cv_image);		
	//setup(size);
	//ROS_INFO("Camera Node setup()");
	
	// Main Loop -- go until ^C terminates
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
