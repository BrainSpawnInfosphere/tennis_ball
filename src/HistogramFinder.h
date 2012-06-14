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
 * Finds objects in an image based on histogram information
 *
 * Change Log:
 * 21 Aug 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */

#ifndef __HISTOGRAM_FINDER_H__
#define __HISTOGRAM_FINDER_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

#include "kevin.h"

// where T is for cv::MatND or cv:SparseMat
//template <class T>
class HistogramFinder {
   
private:
   
   const unsigned int num_colors;
	float hranges[2];
   const float* ranges[3]; // 3 ranges BGR, HSV, etc
   int channels[3];
   
	float threshold;
	cv::MatND histogram;
	cv::SparseMat shistogram;
	bool isSparse;
   cv::Point2f target;
   
   
public:
   
	HistogramFinder(unsigned int colors=32) : threshold(0.1f), isSparse(false), num_colors(colors) {
	    
        // just set them all to the same for now
		ranges[0]= hranges; // all channels have the same range 
		ranges[1]= hranges; 
		ranges[2]= hranges; 
	}
   
	// Sets the threshold on histogram values [0,1]
	void setThreshold(float t) {
	    // bound threshold to [0,1]
        t = (t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t));
		threshold= t;
	}
   
	// Gets the threshold
	float getThreshold() {
      
		return threshold;
	}
   
	// Sets the reference histogram
	void setHistogram(const cv::MatND& h) {
      
		isSparse= false;
		histogram= h;
		cv::normalize(histogram,histogram,1.0);
	}
   
	// Sets the reference histogram from sparse matrix
	void setHistogram(const cv::SparseMat& h) {
      
		isSparse= true;
		shistogram= h;
		cv::normalize(shistogram,shistogram,1.0,cv::NORM_L2);
	}
    
   /**
    * Finds the colored object - all in one funciton call
    * 
    * don't like how this turned out ...
    * 
    * param cv::Mat color BGR image, CM is drawn on image
    * param cv::Point location of found object
    * param double min size of blob
    * return bool if object blob is found
    */
    
	bool find(cv::Mat& hsv, cv::Point2f& p, const double minSize = 0) {
      bool ret = false;
      
      // Match histogram
      cv::Mat result_image;
      findHS(hsv,result_image);
      
      // Perform morphological ops 
      cv::erode(result_image,result_image, cv::Mat()); // get rid of "noise"
      cv::dilate(result_image,result_image,cv::Mat()); // fill in holes
      
      cv::Moments mom = moments(result_image,true);
      double area = mom.m00; 
      
      if(area > minSize){ // found something
         
         double xx = mom.m10; 
         double yy = mom.m01; 
         
         float x = xx/area;
         float y = yy/area;
         
         //printf("Ball: %f %f\n",x,y);
         //ROS_INFO("Ball: %f %f\n",x,y);
         //out = mask;
         //cv::circle( color_image, cvPoint(x,y),3, CV_RGB(0,255,0), -1, 8, 0 );
         
         //Point2f p(x,y);
         p.x = x;
         p.y = y;
         
         ret = true;
      }
      
		return ret;
	}
	
   
   /**
    * Finds the Hue & Saturation of the previously provided histogram in the 
    * image passed to it.
    *
    * params cv::Mat image input HSV image
    * return cv::Mat result binary (single channel) image of found blobs
    */
	cv::Mat& findHS(const cv::Mat& image, cv::Mat& result) {
      
		hranges[0]= 0.0;
		hranges[1]= 180.0;
      
      //printImage(image,"image findHS");
      
      float sranges[] = {0.0, 255.0};
      const float *ranges[] = {hranges,sranges};
      int channels[2]={0,1};
      
      // result of calcBackProject is always a single channel image
		if (isSparse) { // call the right function based on histogram type
         
         cv::calcBackProject(&image,
                             1,            // we only use one image at a time
                             channels,     // vector specifying what histogram dimensions belong to what image channels
                             shistogram,   // the histogram we are using
                             result,       // the resulting back projection image
                             ranges,       // the range of values, for each dimension
                             255.0         // the scaling factor is chosen such that a histogram value of 1 maps to 255
                             );
         
		} else {
         
         cv::calcBackProject(&image,
                             1,            // we only use one image at a time
                             channels,     // vector specifying what histogram dimensions belong to what image channels
                             histogram,    // the histogram we are using
                             result,       // the resulting back projection image
                             ranges,       // the range of values, for each dimension
                             255.0         // the scaling factor is chosen such that a histogram value of 1 maps to 255
                             );
		}
      
          //printImage(result,"result backproj");

      // Threshold back projection to obtain a binary image
		//if (threshold>0.0)
         cv::threshold(result, result, 255.0*threshold, 255, cv::THRESH_BINARY);
      
		return result;
	}
   
};


#endif
