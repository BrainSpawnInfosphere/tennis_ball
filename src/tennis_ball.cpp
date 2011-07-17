#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h" // handles raw or compressed images
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "cvBlob/cvblob.h"

using namespace cv;
using namespace cvb;

IplConvKernel *se21;
IplConvKernel *se11;
CvMat *mask;
IplImage *hsv; // used in color space change
IplImage *hough_in; // binary confidence of tennis balls
IplImage *labelImg; // blobs drawn on image

IplImage *img; // pointer from camera or JPEG image read
IplImage *out; // pointer from process()

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
CvScalar hsv_lower = cvScalar(20.0, 100, 100, 0); //20-30
CvScalar hsv_upper = cvScalar(35.0, 255, 255, 0); //25-35
// h = 32 +- 4
// s = 130 +- 20
// v = 100 +- 20
//CvScalar hsv_lower = cvScalar(28.0, 100, 120, 0); 
//CvScalar hsv_upper = cvScalar(36.0, 160, 180, 0);

CvTracks tracks;
CvBlobs blobs;

void cvOpen(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
	cvErode (src, dst, element, 1);
	cvDilate(src, dst, element, 1);
}

void cvClose(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
	cvDilate(src, dst, element, 1);
	cvErode (src, dst, element, 1);
}

/**
 * Sets up the window and mask sizes
 */
void setup(CvSize &size){
	hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
	mask = cvCreateMat(size.height, size.width, CV_8UC1);
	se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
	se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  CV_SHAPE_RECT, NULL);
	labelImg = cvCreateImage(size, IPL_DEPTH_LABEL, 1);
	hough_in = cvCreateImage(size, 8, 1);
}

// do image proccessing to find color blob
IplImage *processHough(IplImage *img)
{
	
	// Convert to HSV
	cvCvtColor(img, hsv, CV_BGR2HSV);
	
	// Generate binary mask 
	// \todo make hsv colors inputs [20110326]
	cvInRangeS(hsv, hsv_lower,hsv_upper, mask);
	
	// Memory for hough circles
	CvMemStorage* storage = cvCreateMemStorage(0);
	// hough detector works better with some smoothing of the image
	//cvSmooth( mask, mask, CV_GAUSSIAN, 9, 9 );
	cvSmooth( mask, mask, CV_GAUSSIAN, 15, 15 );
	CvSeq* circles = cvHoughCircles(mask, storage, CV_HOUGH_GRADIENT, 2,
											  mask->height/4, 100, 50, 10, 400);
	
	for (int i = 0; i < circles->total; i++)
	{
		float* p = (float*)cvGetSeqElem( circles, i );
		printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
		cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])),
					3, CV_RGB(0,255,0), -1, 8, 0 );
		cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])),
					cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
	}
	
	//cvShowImage( "Camera", frame ); // Original stream with detected ball overlay
	//cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
	//cvShowImage( "After Color Filtering", thresholded ); // The stream after color filtering
	
	cvReleaseMemStorage(&storage);
	
	cvCopy(mask, hough_in, NULL); // copy mask to image
	return hough_in;
}

// do image proccessing to find color blob
IplImage *process(IplImage *img)
{
	//IplImage *img = _img; //?
	
	// Convert to HSV
	cvCvtColor(img, hsv, CV_BGR2HSV);
	
	// Generate binary mask 
	// \todo make hsv colors inputs [20110326]
	cvInRangeS(hsv, hsv_lower,hsv_upper, mask);
	
	// Perform morphological ops 
	cvClose(mask, mask, se21);
	cvOpen(mask, mask, se11);
	
	// rename hough_in - why have it? mask is a matrix ...
	cvCopy(mask, hough_in, NULL); // copy mask to image
	cvSmooth(hough_in, hough_in, CV_GAUSSIAN, 15, 15, 0, 0);
	
	
	// add much?
	cvThreshold(hough_in, hough_in, 5, 255, CV_THRESH_BINARY);
	
	// find and label blobs? result is a big number!
	/*unsigned int result = */cvLabel(hough_in, labelImg, blobs);
	
	
	// remove blobs smaller than 500 px and larger than 100000 px
	//cvFilterByArea(blobs, 500, 1000000);
	
	// not sure what this is doing
	//cvRenderBlobs(labelImg, blobs, img, img, CV_BLOB_RENDER_BOUNDING_BOX);
	
	// ?
	cvUpdateTracks(blobs, tracks, 200., 5);
	
	/*
	for (CvTracks::const_iterator it=tracks.begin(); it!=tracks.end(); ++it)
	{
		cout << "Track " << it->second->id << endl;
		if (it->second->inactive)
			cout << " - Inactive for " << it->second->inactive << " frames" << endl;
		else
			cout << " - Associated with blobs " << it->second->label << endl;
		cout << " - Lifetime " << it->second->lifetime << endl;
		cout << " - Active " << it->second->active << endl;
		cout << " - Bounding box: (" << it->second->minx << ", " << it->second->miny << ") - (" << it->second->maxx << ", " << it->second->maxy << ")" << endl;
		cout << " - Centroid: (" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
		cout << endl;
	}
	*/
	
	// draw bounding box and number on image; also prints info to screen
	//cvRenderTracks(tracks, img, img, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX|CV_TRACK_RENDER_TO_STD);
	
	return hough_in;
}


// do image proccessing to find color blob
IplImage *process2(IplImage *img)
{
	//IplImage *img = _img; //?
	
	// Convert to HSV
	cvCvtColor(img, hsv, CV_BGR2HSV);
	
	//cvSetImageCOI( hsv, 1); // ignore all but hue (color)
	
	// Generate binary mask 
	// \todo make hsv colors inputs [20110326]
	cvInRangeS(hsv, hsv_lower,hsv_upper, mask);
	//cvInRangeS(hsv, cvScalarAll(30), cvScalarAll(35), mask);
	
	// Perform morphological ops 
	//cvClose(mask, mask, se21);
	//cvOpen(mask, mask, se11);
	
	cvErode(mask,mask,NULL, 3);
	cvSmooth(mask, mask, CV_GAUSSIAN, 15, 15, 0, 0);
	
	// rename hough_in - why have it? mask is a matrix ...
	cvCopy(mask, hough_in, NULL); // copy mask to image
	
	
	// add much?
	//cvThreshold(hough_in, hough_in, 5, 255, CV_THRESH_BINARY);
	
	CvMoments mom;
	cvMoments(mask, &mom, 1); // find blobs and treat binary
	double area = cvGetCentralMoment(&mom,0,0);
	
	if(area > 1000){
		
		double xx = cvGetSpatialMoment(&mom,1,0);
		double yy = cvGetSpatialMoment(&mom,0,1);
		
		float x = xx/area;
		float y = yy/area;
		
		printf("Ball: %f %f\n",x,y);
		cvCircle( img, cvPoint(x,y),3, CV_RGB(0,255,0), -1, 8, 0 );
	}
	
	
	return hough_in;
}

void hist(IplImage *s){
	
	Mat src = cvarrToMat(s);
	
	// let's quantize the hue to 30 levels
	// and the saturation to 32 levels
	int hbins = 30; //, sbins = 32;
	int histSize[] = {hbins /*, sbins*/};
	// hue varies from 0 to 179, see cvtColor
	float hranges[] = { 0, 180 };
	// saturation varies from 0 (black-gray-white) to
	// 255 (pure spectrum color)
	//float sranges[] = { 0, 256 };
	const float* ranges[] = { hranges /*, sranges */};
	MatND hist;
	// we compute the histogram from the 0-th and 1-st channels
	int channels[] = {0 /*, 1*/};
	
	try{
		calcHist( &src, 1, channels, Mat(), // do not use mask
				hist, 1 /*2*/, histSize, ranges,
				true, // the histogram is uniform
				false );
		
		//double maxVal=0;
		//double minVal=0;
		Scalar mean,std;
		//meanStdDev(src, &minVal, &maxVal);
		meanStdDev(src, mean, std);
		
		//printf("min/max: %f %f\n", minVal, maxVal);
		printf("mean/std: %f %f\n", mean[0], std[0]);
		
		//std::cout<<"mean/std: "<<mean<<" "<<std<<endl;
	}
	catch(...){
		printf("Histogram error\n");
		imshow( "H-S Histogram", src );
	}
		
	
	/*
	int scale = 10;
	Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);
	
	for( int h = 0; h < hbins; h++ )
		//for( int s = 0; s < sbins; s++ )
		{
			float binVal = hist.at<float>(h, s);
			int intensity = cvRound(binVal*255/maxVal);
			rectangle( histImg, Point(h*scale, s*scale),
						 Point( (h+1)*scale - 1, (s+1)*scale - 1),
						 Scalar::all(intensity),
						 CV_FILLED );
		}
	
	
	imshow( "H-S Histogram", histImg );
	 */
}

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
	
	cvErode(mask,mask,0, 3);
	cvDilate( mask,mask,0,5);
	//cvSmooth(mask, mask, CV_GAUSSIAN, 5, 5, 0, 0);
	
	// rename hough_in - why have it? mask is a matrix ...
	cvCopy(mask, hough_in, NULL); // copy mask to image
	
	printstd(hsv,hough_in);
	
	// add much?
	//cvThreshold(hough_in, hough_in, 5, 255, CV_THRESH_BINARY);
	
	CvMoments mom;
	cvMoments(mask, &mom, 1); // find blobs and treat binary
	double area = cvGetCentralMoment(&mom,0,0);
	
	if(area > 0){
		
		double xx = cvGetSpatialMoment(&mom,1,0);
		double yy = cvGetSpatialMoment(&mom,0,1);
		
		float x = xx/area;
		float y = yy/area;
		
		printf("Ball: %f %f\n",x,y);
		cvCircle( img, cvPoint(x,y),3, CV_RGB(0,255,0), -1, 8, 0 );
	}
	
	
	return hough_in;
}


class ImageConverter {
	
public:
	
	/**
	 * Constructer: sets up the windows, subscribers, and publishers
	 * \publish image
	 * \subscribe axis_camera
	 * \window Image window
	 * \window hsv
	 */
	ImageConverter(ros::NodeHandle &n) :
	n_(n), it_(n_)
	{
		image_pub_ = it_.advertise("image_out",1);
		
		cvNamedWindow("Image window");
		cvNamedWindow("hsv");
		cvNamedWindow( "H-S Histogram");
		
		//image_sub_ = it_.subscribe("axis_camera", 1, &ImageConverter::imageCallback, this, image_transport::TransportHints("compressed"));
		image_sub_ = it_.subscribe("camera", 1, &ImageConverter::imageCallback, this, image_transport::TransportHints("compressed"));
	
		needSetup = true;
	}
	
	/**
	 * Destructor: closes windows
	 */
	~ImageConverter()
	{
		cvDestroyWindow("Image window");
		cvDestroyWindow("hsv");
		cvReleaseImage(&hsv);

	}
	
	//void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr,
	//						 const sensor_msgs::CameraInfoConstPtr& info_msg)
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
		
		// setup - maybe change this to reset, incase camera changes resolutins?
		if(needSetup){
			CvSize size = cvGetSize(cv_image);		
			setup(size);
			needSetup = false;
			ROS_INFO("Camera Node setup()");
		}
		
		// find tennis ball
		//img = process(cv_image);
		//img = process2(cv_image);
		img = process3(cv_image);
		//img = processHough(cv_image);
		
		// debug - display images
		cvShowImage("Image window",cv_image);
		cvShowImage("hsv",img);
		cvWaitKey(10);
		
		// publish cv_image to bridge
		try
		{
			image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("Can't publish to CvBridge");
		}
		
	}
	
protected:
	
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	image_transport::Publisher image_pub_;
	
	bool needSetup;
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tennis_ball");
	ros::NodeHandle n;
	ImageConverter ic(n);
	ros::spin();
	return 0;
}
