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
 * rosrun tennis_ball tracker _debug:=true/false _histogram:=file_path
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

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "Kalman.h"
#include "Histogram.h"
#include "HistogramFinder.h"
#include "kevin.h"

using namespace cv;

sensor_msgs::CvBridge bridge;

//---- Setup finder and color histogram --
HistogramFinder finder;
//HistogramHS<cv::SparseMat> hc;

std::string defaultimage = "/Users/kevin/ros_sandbox/images/red_ref_1.jpg";

// 32 works good
const int num_colors = 128;

bool debug = false;

// colors to search for
//CvScalar hsv_lower = cvScalar(20.0, 50, 50, 0); //20-30
//CvScalar hsv_upper = cvScalar(35.0, 255, 255, 0); //25-35
// h = 32 +- 4
// s = 130 +- 20
// v = 100 +- 20

///////////////////////////////////////////////////////

Kalman kalman(4,2,1.0f/30.0f);

////////////////////////////////////////////////////////



class BallFinder {
public:
    BallFinder(){
        ;
    }
    
    void setNumColors(int i){
        cr.set(i);
    }
    
    const cv::Mat& getResult(){
        return result;
    }
    
    const cv::MatND& getHistogram(){
        return colorhist;
    }

    // need to figure out a better findBall() function!
    cv::Rect findBall(cv::Mat& image){
        // Convert to HSV space
        cv::Mat hsv;    
        preprocess(image,hsv);
        
        // find ball    
#if 1    
        finder.findHS(hsv, result);
#else
        // blue = 240 -> 120
        cv::inRange(hsv,Scalar(110,120,0),Scalar(130,255,255),result);
        //cv::inRange(hsv,Scalar(0,0,0),Scalar(255,255,255),result);
#endif
        // Perform morphological ops 
        erode(result,result, cv::Mat()); // get rid of "noise"
        dilate(result,result,cv::Mat()); // fill in holes
        
        // Get location
        cv::Rect rect = getBounds(result,image);
        cv::circle(image, cvPoint(rect.x+rect.width/2.0,rect.y+rect.height/2.0),3, CV_RGB(0,255,0), -1, 8, 0 );
    
        return rect;
    }

    bool createHistogram(std::vector<std::string>& pics,int num_colors){
        // get images
        unsigned int size = pics.size();
        cv::Mat img, hsv[size];
        
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
    }
       
    
protected:    

    void preprocess(cv::Mat& in, cv::Mat& out){
        // reduce color space
        cv::Mat image_reduced = in.clone(); // fixme!!
        cv::Mat image_blur = in.clone(); // fixme!!
        cr.reduce(image_reduced);
        
        // smooth image
        cv::blur(image_reduced,image_blur,Size(9,9));
        
        // Convert to HSV space
        cv::cvtColor(image_blur, out, CV_BGR2HSV);
    }
     
    /**
     * \input image 8b single channel image
     * \input color_image BGR image to draw boundry on
     */
    Rect getBounds(const cv::Mat& in, cv::Mat& color_image){
        //ROS_INFO("getBounds");
        
        vector<vector<Point> > v;
        cv::Rect rect;
        
        cv::Mat image = in.clone();
        
        findContours(image,v,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
        
        ROS_INFO("getBounds found %d blobs",v.size());
        
        // no target ball in image
        if(v.size() == 0) return rect;
        
        // Finds the contour with the largest area
        int area = 0;
        unsigned int idx;
        for(unsigned int i=0; i<v.size();i++) {
            if(area < v[i].size()){
                idx = i; 
                area = v[i].size();
            }
        }
        // Calculates the bounding rect of the largest area contour
        rect = boundingRect(v[idx]);
        Point pt1, pt2;
        pt1.x = rect.x;
        pt1.y = rect.y;
        pt2.x = rect.x + rect.width;
        pt2.y = rect.y + rect.height;
        
        // Draws the rect in the original image and show it
        rectangle(color_image, pt1, pt2, CV_RGB(255,0,0), 1);
        
        return rect;
    }
    
    HistogramFinder finder;
    ColorReduction cr;
    cv::MatND colorhist;
    cv::Rect ball_rect; 
    cv::Mat result;
};

////////////////////////////////////////////////////////

BallFinder bf;


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
    
    //ROS_INFO("grabbed image transport");
    
    // grab image and make temps
    cv::Mat image(ipl);
    
    bf.findBall(image);
    //if(ok) cv::Rect rect = bf.getBounds(); ???
    
    /*
     class Target {
        cv::Rect bounds;
        cv::Point cm;
        
        void draw(cv::Mat& image);
     }
     */
    
    if(debug){
        imshow("Image window",image);
        //imshow("Image window",image_reduced);
        imshow("result - mask",bf.getResult());
        waitKey(10);
    }
  
  //ROS_INFO("Hi");
  
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n; //("~");
	ros::Rate r(15);
	
	n.param<bool>("debug",debug, true);		n.deleteParam("debug");
  
    //---- Setup Histogram ---------
    //std::string histogram;
    //n.param<std::string>("histogram",histogram, defaultimage);		n.deleteParam("histogram");
 
    std::vector<std::string> imgs;
    imgs.push_back("/Users/kevin/ros_sandbox/images/blue_ref_0.jpg");
    imgs.push_back("/Users/kevin/ros_sandbox/images/blue_ref_1.jpg");
    imgs.push_back("/Users/kevin/ros_sandbox/images/blue_ref_2.jpg");
    bf.createHistogram(imgs,num_colors);
    
    namedWindow("Histogram");
    imshow("Histogram",bf.getHistogram());
    //waitKey(0);
    //exit(1);

    ROS_INFO("histogram set");
    
    if(debug){
        namedWindow("Image window",1);cvMoveWindow("Image window",0,0);
        namedWindow("result - mask",1);cvMoveWindow("result - mask",0,300);
    }
    
    image_transport::ImageTransport transport(n);
    image_transport::Subscriber image_sub = transport.subscribe("/opencv_cam/camera", 1, imageCallback /*, image_transport::TransportHints("compressed")*/);
    
    // Main Loop -- go until ^C terminates
    while (ros::ok())
    {
        //ROS_INFO("go");
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
