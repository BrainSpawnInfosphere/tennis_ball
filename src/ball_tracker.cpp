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
 * Tracks a colored ball using OpenCV
 *
 * rosrun tennis_ball tracker -h | --help
 *
 * Also takes rosparams ball [0|1|2] debug [true|false]
 *
 * Change Log:
 * 21 Aug 2011 Created
 *  8 Jun 2012 updated for Fuerte and OSX Lion - seems to work fine
 *  9 Jun 2012 added sample launch file - works with kinect rgb camera
 *
 **********************************************************************
 *
 * 
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Kalman.h"
#include "Histogram.h"
#include "HistogramFinder.h"
#include "kevin.h"

// 32 works good
const int num_colors = 32;

// colors to search for
//CvScalar hsv_lower = cvScalar(20.0, 50, 50, 0); //20-30
//CvScalar hsv_upper = cvScalar(35.0, 255, 255, 0); //25-35
// h = 32 +- 4
// s = 130 +- 20
// v = 100 +- 20

///////////////////////////////////////////////////////

////////////////////////////////////////////////////////


/**
 * Finds a ball in an image or region of interest (roi) in an image. Key
 * functions are to set the roi (setROI()) and find the ball in the image
 * (findBall()).
 *
 * This class uses a histogram matching routine from OpenCV also reduces 
 * the number of colors in an image to reduce the size of the histogram
 * and speed the matching process. Key function is creating a histogram
 * (createHistogram()).
 */
class BallFinder {
public:
    BallFinder(int w, int h){
        roi.x = 0;
        roi.y = 0;
        roi.width = w;
        roi.height = h;
    }
    
    // bunch of get and set functions
    const cv::Mat& getResult(){ return result; }
    const cv::MatND& getHistogram(){ return colorhist; }
    
    const unsigned int getSize(){ return target_size; }
    const cv::Point2f& getCM(){ return target_cm; }
    const cv::Rect& getBB(){ return target_bb; }
    
    void setROI(cv::Rect& p){ roi = p; }
    void setNumColors(int i){ cr.set(i); }
    
    // need to figure out a better findBall() function!
    bool findBall(cv::Mat& image){
        // Convert to HSV space
        cv::Mat hsv;    
        cv::Mat ROI = image(roi);
        preprocess(ROI,hsv); // only grab the roi
        
        // find ball in HSV roi image   
        finder.findHS(hsv, result);
        
        // Perform morphological ops 
        cv::erode(result,result, cv::Mat()); // get rid of "noise"
        cv::dilate(result,result,cv::Mat()); // fill in holes
        
        // Get size, CM and BB of the largest blob
        target_size = getBounds(result,target_cm,target_bb);
        
        // fix offsets from ROI
        target_cm.x += roi.x;
        target_cm.y += roi.y;
        target_bb.x += roi.x;
        target_bb.y += roi.y;
        
        // determine if ball was found based on target_size
        return bool(target_size);
    }
    
	// Draws the rect in the original image and show it
    void drawBoundingBox(cv::Mat& image){
        cv::circle(image, target_cm,3, CV_RGB(0,255,0), -1, 8, 0 );
        cv::rectangle(image, target_bb, CV_RGB(255,0,0), 1);
    }

    /**
     * Creates a histogram from a serial of images
     * \input pics a vector of file names
     * \input num_colors is how many colors to reduce the image too
     */
    bool createHistogram(std::vector<std::string>& pics,int num_colors){
        // get images
        const unsigned int size = pics.size();
        cv::Mat img;
        cv::Mat *hsv = new cv::Mat[size];
        
        for(unsigned int i=0;i<pics.size();i++){
            img = cv::imread(pics[i].c_str(),1);
            if(!img.data) return false;
            preprocess(img,hsv[i]);
        }
        
        // make histogram
        HistogramHS<cv::MatND> hc;
        colorhist = hc.getHistogramHSV(hsv,size,num_colors,num_colors); 
        //hc.print(colorhist);
        //HistogramHS<cv::SparseMat> hc;
        //cv::SparseMat colorhist = hc.getHistogram(imgs,2); 
        finder.setHistogram(colorhist);
        finder.setThreshold(0.03f);
        
        return true;
    }      
    
protected:    
	/**
	 * Prepares image for operations:
	 *   - reduces number of colors
	 *   - smooths image
	 *   - converts to HSV
	 */
    void preprocess(cv::Mat& in, cv::Mat& out){
        // reduce color space
        cv::Mat image_reduced = in.clone(); // fixme!!
        cv::Mat image_blur = in.clone(); // fixme!!
        cr.reduce(image_reduced);
        
        // smooth image
        cv::blur(image_reduced,image_blur,cv::Size(9,9));
        
        // Convert to HSV space
        cv::cvtColor(image_blur, out, CV_BGR2HSV);
    }
     
    /**
     * Finds the bounding box and cm of the largest blob in the image (or roi).
     *
     * \return target_size is the size of the largest blob
     * \input color_image BGR image to draw boundry on
     */
    unsigned int getBounds(const cv::Mat& in, cv::Point2f& target_cm, cv::Rect& target_bb){
        std::vector<std::vector<cv::Point> > v; // list of points 
        
        cv::Mat image = in.clone();
        
        cv::findContours(image,v,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
        
        //ROS_INFO("getBounds found %d blobs",(int) v.size());
        
        // no target ball in image
        if(v.size() == 0) return 0;
        
        // Finds the contour with the largest area
        unsigned int area = 0;
        unsigned int idx;
        for(unsigned int i=0; i<v.size();i++) {
        	// the more points, the larger the area
            if(area < v[i].size()){
                idx = i; 
                area = v[i].size();
            }
        }
        
        // Calculates the bounding rect of the largest area contour
        target_bb = cv::boundingRect(v[idx]);
        target_cm.x = target_bb.x+target_bb.width/2.0;
        target_cm.y = target_bb.y+target_bb.height/2.0;
        
        return area;
    }
    
    //--- members -------------------
    HistogramFinder finder;   // finds pixels which match histogram
    ColorReduction cr;        // reduces color space
    cv::MatND colorhist;      // target histogram
    cv::Rect roi;             // region of interest in camera image
    cv::Mat result;           // binary blob image
    cv::Point2f target_cm;    // target center of mass
    cv::Rect target_bb;       // target bounding box
    float target_size;        // target size in pixels
    
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////



BallFinder bf(640,480);
bool debug = false;
Kalman kalman(4,4,1.0f/30.0f);


float distance(cv::Point2f& a, cv::Point2f& b){
	cv::Point2f c = a-b;
	float dist2 = c.x*c.x+c.y*c.y;
	return dist2;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
  
  // Grab image from bridge to cv_image
  cv_bridge::CvImagePtr cv_msg;
  
  try
  {
  	cv_msg = cv_bridge::toCvCopy(msg_ptr, "bgr8"); // try toCvShare
  }
  catch (...)
  {
    ROS_ERROR("Can't get image from CvBridge");
    return;
  }
    /*
    // use bounding box from kalman if it is valid
    cv::Mat roi;
    if(bf.getSize() > 40){ // fix size 40
    	cv::Rect bb = bf.getBB();
    	roi = cv_msg->image(bb);
    }
    else { // take whole image
    	roi = cv_msg->image;
    }
    */
    
    // find ball in image using histogram
    // only look at bottom half of image -- ball should be on ground
    cv::Rect bb(0,240,640,240);
    bf.setROI(bb);
    bool found_ball = bf.findBall(cv_msg->image);
    //float diff = distance( bf.getCM(), kalman.getCM() );
    //const float pixel_dist = 10*10;
    //ROS_INFO("size: %d", bf.getSize());
    
    if(found_ball && bf.getSize() > 50 /*&& diff < pixel_dist*/){ 
    	//ROS_INFO("ball found; dist: %g",diff);
    	bf.drawBoundingBox(cv_msg->image);
    	kalman.update( bf.getCM() );
    	kalman.draw(cv_msg->image);
    }
    else {
    	//ROS_INFO("no ball found; dist: %g",diff);
    	kalman.update();
    	kalman.draw(cv_msg->image);
    }
    
    if(debug){
        cv::imshow("Image window",cv_msg->image);
        cv::imshow("result - mask",bf.getResult());
        cv::waitKey(1);
    }
}

void printHelp(){
	printf("tracker -h --help\n");
	printf(" rosparams: debug [true|false]\n");
	printf(" rosparams: ball [0 red | 1 blue | 2 tennis]\n");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n("~");
	ros::Rate r(30);
	int ball = 0;
	
	// command line args - only does help
	if(argc == 2){
		std::string arg = argv[1];
		
		if(!arg.compare("--help") or !arg.compare("-h")){
			printHelp();
			return 0;
		}
	}
	else if( argc > 2 ){
		ROS_ERROR("Too many commandline args, try --help");
		return 1;
	}
	
	// grab parameters
	n.param<bool>("debug",debug, true);		n.deleteParam("debug");
	n.param<int>("ball",ball, 0);		n.deleteParam("ball");
  
    // which sample images do we use?
    std::vector<std::string> imgs;
    switch(ball){
    	case 0: // red ball
    		imgs.push_back("/Users/kevin/ros_sandbox/images/red_ref_0.jpg");
    		imgs.push_back("/Users/kevin/ros_sandbox/images/red_ref_1.jpg");
    		break;
    	case 1: // blue ball
    		imgs.push_back("/Users/kevin/ros_sandbox/images/blue_ref_0.jpg");
    		imgs.push_back("/Users/kevin/ros_sandbox/images/blue_ref_1.jpg");
    		imgs.push_back("/Users/kevin/ros_sandbox/images/blue_ref_2.jpg");
    		break;
    	case 2: // tennis ball
    		imgs.push_back("/Users/kevin/ros_sandbox/images/ref_0.jpg");
    		imgs.push_back("/Users/kevin/ros_sandbox/images/ref_1.jpg");
    }
    
    // create histogram map of the colored balls from above
    bf.createHistogram(imgs,num_colors);
    
    cv::namedWindow("Histogram");
    cv::imshow("Histogram",bf.getHistogram());

    ROS_INFO("histogram set");
    
    if(debug){
        cv::namedWindow("Image window",1);cvMoveWindow("Image window",0,0);
        cv::namedWindow("result - mask",1);cvMoveWindow("result - mask",0,500);
    }
    
    image_transport::ImageTransport transport(n);
    image_transport::Subscriber image_sub = transport.subscribe("/image_in", 1, imageCallback);
    
    // Main Loop -- go until ^C terminates
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
