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
 * Misc functions
 *
 * Change Log:
 * 21 Jan 2012 Created
 *
 **********************************************************************
 *
 * 
 *
 */


#ifndef __KEVIN_H__
#define __KEVIN_H__

#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
//#include <iostream>


class ColorReduction {
public:
    ColorReduction(int num=64){
        set(num);
    }
    
    void set(int num=64){
        div = num;
      int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
      // mask used to round the pixel value
      mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
    }
    
    cv::Mat& reduce(cv::Mat& image){
      
      cv::Mat_<cv::Vec3b>::const_iterator it= image.begin<cv::Vec3b>();
      cv::Mat_<cv::Vec3b>::const_iterator itend= image.end<cv::Vec3b>();
      
      // Set output image (always 1-channel)
      //cv::Mat result(image.rows,image.cols,image.type());
      //cv::Mat_<cv::Vec3b>::iterator itr= result.begin<cv::Vec3b>();
      
      for ( ; it!= itend; ++it) {
         (*it)[0]= ((*it)[0]&mask) + div/2;
         (*it)[1]= ((*it)[1]&mask) + div/2;
         (*it)[2]= ((*it)[2]&mask) + div/2;
      }
      
      return image;
    }
      
    /*  
    cv::Mat& reduceCopy(const cv::Mat& image){
      // create a copy
      cv::Mat result = image.clone();
      reduce(result);
      return result;
   }
   */
   
protected:
    uchar mask;
    int div;
};


void stdDev(cv::Mat& src, char a, char b, char c){
	cv::Scalar mean,std;
	cv::meanStdDev(src, mean, std);
	
	ROS_INFO("Mean/Std: %c[%.1f %.1f] %c[%.1f %.1f] %c[%.1f %.1f]",
	    a,
	    mean[0], std[0],
	    b,
	    mean[1], std[1],
	    c,
	    mean[2], std[2]);
}

void stdDevHSV(cv::Mat& src){
    stdDev(src,'H','S','V');
} 


void stdDevBGR(cv::Mat& src){
    stdDev(src,'B','G','R');
} 

void stdDevHSV(IplImage *s){
	cv::Mat src = cv::cvarrToMat(s);
	stdDevHSV(src);
}
/*
void printstd(IplImage *s, IplImage *m){
	cv::Mat src = cv::cvarrToMat(s);
	cv::Mat mat = cv::cvarrToMat(m);
	
	cv::Scalar mean,std;
	cv::meanStdDev(src, mean, std, mat);
	
	ROS_INFO("============================\n");
	ROS_INFO("Hue mean/std: %f %f\n", mean[0], std[0]);
	ROS_INFO("Sat mean/std: %f %f\n", mean[1], std[1]);
	ROS_INFO("Val mean/std: %f %f\n", mean[2], std[2]);
}
*/
void printImage(const cv::Mat& i, std::string s="Image"){
    ROS_INFO("%s[%dx%dx%d] 0x%x",s.c_str(),i.rows,i.cols,i.channels(),i.data);
}


#endif
