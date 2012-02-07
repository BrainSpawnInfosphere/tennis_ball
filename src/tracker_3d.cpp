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
 * Takes the ball location from 2 cameras (stereo) and converts it 
 * into a 3D location
 *
 * rosrun tennis_ball tracker3d _debug:=true/false _config:=file_path
 *
 * Change Log:
 *  1 Sept 2011 Created
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
//#include "Histogram.h"
//#include "HistogramFinder.h"

using namespace cv;

sensor_msgs::CvBridge bridge;

//---- Setup finder and color histogram --
HistogramFinder finder;
//ColorHistogram hc;
//HistogramHS<cv::MatND> hc;
HistogramHS<cv::SparseMat> hc;


bool debug = false;


///////////////////////////////////////////////////////

//Kalman kalman(4,2,1.0f/30.0f);

////////////////////////////////////////////////////////

typedef struct msg_t {
	int i;
} msg;

// do image proccessing to find color blob
void process( const msg& left, const msg& right)
{	
	
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker3D");
	ros::NodeHandle n; //("~");
	ros::Rate r(15);
  
  n.param<bool>("debug",debug, true);		n.deleteParam("debug");
  
  //---- Setup Histogram ---------
  std::string config;
  n.param<std::string>("config",config, " ");		n.deleteParam("config");
	
	
	if (0)
  {
    ROS_ERROR("Couldn't read config data file: %s", config.c_str());
    return 1;
  }
  
	// Main Loop -- go until ^C terminates
	while (ros::ok())
	{
    //ROS_INFO("go");
		
		process(left,right);
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
