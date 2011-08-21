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
 * Simple kalman filter using OpenCV 2.2 to track objects.
 *
 * Change Log:
 * 21 Aug 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */


#ifndef __KALMAN_FILTER_OPENCV_H__
#define __KALMAN_FILTER_OPENCV_H__

//#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>


///////////////////////////////////////////////////////


/**
 * Wrap class cv::KalmanFilter like this to handle it prettier.
 */
class Kalman
{
public:
	
	Kalman(unsigned int m, unsigned int n, float rate);
	~Kalman() { delete filter; }
	
	const cv::Mat& update(const cv::Point2f& p);
	const cv::Mat& update(void);
	
private:
	const unsigned int N; //! dimension of transition matrix: NxN
	const unsigned int M; //! length of measurement vector
	
	cv::KalmanFilter* filter; // why pointer?
	
	unsigned int numActiveFrames; // ?
	unsigned int numInactiveFrames;
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
Kalman::Kalman(unsigned int m, unsigned int n, float rate) : M(m), N(n)
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

#endif
