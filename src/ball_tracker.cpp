/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 8/21/2011
 *********************************************************************
 *
 * Tracks a colored ball using OpenCV 2.2.
 *
 * rosrun tennis_ball tracker 
 *
 * Change Log:
 * 21 Aug 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/CvBridge.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "Kalman.h"
#include "Histogram.h"
#include "HistogramFinder.h"

using namespace cv;

sensor_msgs::CvBridge bridge;

//---- Setup finder and color histogram --
HistogramFinder finder;
//ColorHistogram hc;
//HistogramHS<cv::MatND> hc;
HistogramHS<cv::SparseMat> hc;

// 32 works good
const int num_colors = 32;

// colors to search for
//CvScalar hsv_lower = cvScalar(20.0, 50, 50, 0); //20-30
//CvScalar hsv_upper = cvScalar(35.0, 255, 255, 0); //25-35
// h = 32 +- 4
// s = 130 +- 20
// v = 100 +- 20

///////////////////////////////////////////////////////

Kalman kalman(4,2,1.0f/30.0f);

////////////////////////////////////////////////////////

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
void process( cv::Mat& in, cv::Mat& color_image)
{	
	
   cv::Mat mask = in;
	
	// Perform morphological ops 
	erode(mask,mask, cv::Mat()); // get rid of "noise"
	dilate(mask,mask,cv::Mat()); // fill in holes
	
	Moments mom = moments(mask,true);
	double area = mom.m00; //cvGetCentralMoment(&mom,0,0);
	
	if(area > 0){ // found something
		
		double xx = mom.m10; //cvGetSpatialMoment(&mom,1,0);
		double yy = mom.m01; //cvGetSpatialMoment(&mom,0,1);
		
		float x = xx/area;
		float y = yy/area;
		
		//printf("Ball: %f %f\n",x,y);
      //ROS_INFO("Ball: %f %f\n",x,y);
		//out = mask;
		circle( color_image, cvPoint(x,y),3, CV_RGB(0,255,0), -1, 8, 0 );
		
		Point2f p(x,y);
		
		const cv::Mat& m = kalman.update(p);
		
		p.x = m.at<float>(0,0);
		p.y = m.at<float>(1,0);
		
		circle( color_image, p,6, CV_RGB(255,0,0), 2, 8, 0 );
	}
	else { // no tennis ball found
		const cv::Mat& m = kalman.update();
		Point2f p;
		p.x = m.at<float>(0,0);
		p.y = m.at<float>(1,0);
      
      //ROS_INFO(" ... ");
		
		//out = mask;
		circle( color_image, p,3, CV_RGB(0,0,255), -1, 8, 0 );
	}
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		//ROS_INFO("image ...");
		
		// Grab image from bridge to cv_image
		IplImage *ipl = NULL;
		
		try
		{
			ipl = bridge.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& error)
		{
			ROS_ERROR("Can't get image from CvBridge");
			return;
		}
		
		// grab image and make temps
		cv::Mat image(ipl);
		
      // reduce color space
      cv::Mat image_reduced = hc.colorReduce(image,num_colors);
      
      // Convert to HSV space
      cv::Mat hsv;
      cv::cvtColor(image_reduced, hsv, CV_BGR2HSV);
      cv::Mat result = finder.findHS(hsv);
      
      
      process(result,image);
      
		imshow("Image window",image);
		//imshow("Image window",image_reduced);
		//imshow("result - mask",result);
		waitKey(10);
		
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tennis_ball");
	ros::NodeHandle n;
	ros::Rate r(30);
   
   //---- Setup Histogram ---------
	cv::Mat ball = cv::imread("/Users/kevin/ros_sandbox/camera_node/red_ref_1.jpg",1);
	if (!ball.data){
      ROS_ERROR("Couldn't read histogram data file");
      return 1;
   }
   
   //---- build histogram and setup finder --
   cv::Mat ball_reduced = hc.colorReduce(ball,num_colors);
	//cv::MatND colorhist = hc.getHueSaturationHistogram(ball_reduced);
	//cv::MatND colorhist = hc.getHistogram(ball_reduced);
  cv::SparseMat colorhist = hc.getHistogram(ball_reduced); 
	finder.setHistogram(colorhist);
	finder.setThreshold(0.03f);
	
	namedWindow("Image window",1);
	namedWindow("result - mask",1);
	
	image_transport::ImageTransport transport(n);
	image_transport::Subscriber image_sub = transport.subscribe("opencv_cam/camera", 1, imageCallback, image_transport::TransportHints("compressed"));
		
   
	
	// Main Loop -- go until ^C terminates
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
