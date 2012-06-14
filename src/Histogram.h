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
 * Handles histogram information
 *
 * Change Log:
 * 21 Aug 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */


#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

/**
 * Handles a generic nD histogram creation and color reduction. This class
 * really only creates the histogram but doesn't do anything with it. 
 * Histograms can be either cv::MatND or cv::SparseMat. 
 */
template <class T>
class Histogram {
public:
   
	Histogram() {;}
   
   /* Need to add something like this ...
    
	// Computes the 1D histogram and returns an image of it.
	cv::Mat getHistogramImage(const cv::Mat &image){
      
		// Compute histogram first
		cv::MatND hist= getHistogram(image);
      
		// Get min and max bin values
		double maxVal=0;
		double minVal=0;
		cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
      
		// Image on which to display histogram
		cv::Mat histImg(histSize[0], histSize[0], CV_8U,cv::Scalar(255));
      
		// set highest point at 90% of nbins
		int hpt = static_cast<int>(0.9*histSize[0]);
      
		// Draw vertical line for each bin 
		for( int h = 0; h < histSize[0]; h++ ) {
         
			float binVal = hist.at<float>(h);
			int intensity = static_cast<int>(binVal*hpt/maxVal);
			cv::line(histImg,cv::Point(h,histSize[0]),cv::Point(h,histSize[0]-intensity),cv::Scalar::all(0));
		}
      
		return histImg;
	}
   */
   /**
    * Helper funtion
    */
   inline T getHistogram2D(const cv::Mat &image,
                           const float *ranges[2],
                           int histSize[],
                           int channels[]){
      return getHistogram(image,ranges,histSize,channels,2);
   }
   
   /**
    * Helper funtion
    */
   inline T getHistogram2D(const cv::Mat *image,
                            int num,
                           const float *ranges[2],
                           int histSize[],
                           int channels[]){
      return getHistogram(image,num,ranges,histSize,channels,2);
   }
   
   /**
    * Helper funtion
    */
   inline T getHistogram3D(const cv::Mat &image,
                           const float *ranges[2],
                           int histSize[],
                           int channels[]){
      return getHistogram(image,ranges,histSize,channels,3);
   }
   
	/** 
    * Computes the nD Hue histogram with out a mask. The assumption is the
    * images passed are already cropped and the whole image is what is of
    * interest.
    */
	T getHistogram(const cv::Mat &image,
                  const float *ranges[2],
                  int histSize[],
                  int channels[],
                  int dim) {
      
		T hist;
      
		// Compute histogram
		cv::calcHist(&image, 
                   1,			    // histogram of 1 image only
                   channels,	// the channel used
                   cv::Mat(),	// no mask is used
                   hist,		  // the resulting histogram
                   dim,			    // it is a 3D histogram
                   histSize,	// number of bins
                   ranges		// pixel value range
                   );    // do not accumulate histogram
      
		return hist;
	}  
   
	T getHistogram(const cv::Mat *image,
	                int num,
                  const float *ranges[2],
                  int histSize[],
                  int channels[],
                  int dim) {
      
		T hist;
      
		// Compute histogram
		cv::calcHist(image, 
                   num,			    // histogram of 1 image only
                   channels,	// the channel used
                   cv::Mat(),	// no mask is used
                   hist,		  // the resulting histogram
                   dim,			    // it is a 3D histogram
                   histSize,	// number of bins
                   ranges,		// pixel value range
                   true,     // uniform histogram
                   true);      // accumulate histograms
      
		return hist;
	}  
   
   
   /**
    * Print the histogram bins for a 1D histogram
    *
    * \todo need to modify so it works for nD histograms
    */
   void print(const cv::MatND& hist){
      for (int i=0; i<256; i++) 
         std::cout << "Value " << i << " = " << hist.at<float>(i) << std::endl; 
   }
   
   /**
    * Color reduction function from ...
    * Should move it out into its own class ...
    *//*
	cv::Mat colorReduce(const cv::Mat &image, int div=64) {
      
      int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
      
      // I don't totally understand how this mask works
      // mask used to round the pixel value
      uchar mask= 0xFF<<n; // e.g. for div=16, mask= 0xF0
      
      cv::Mat_<cv::Vec3b>::const_iterator it= image.begin<cv::Vec3b>();
      cv::Mat_<cv::Vec3b>::const_iterator itend= image.end<cv::Vec3b>();
      
      // Setup output image
      cv::Mat result(image.rows,image.cols,image.type());
      cv::Mat_<cv::Vec3b>::iterator itr= result.begin<cv::Vec3b>();
      
      for ( ; it!= itend; ++it, ++itr) {
         
         (*itr)[0]= ((*it)[0]&mask) + div/2;
         (*itr)[1]= ((*it)[1]&mask) + div/2;
         (*itr)[2]= ((*it)[2]&mask) + div/2;
      }
      
      return result;
   }
   */
};

/**
 * Histogram class specifically designed to handle hue and saturation (2D)
 * histograms. Since the base class is Histogram<T>, it can be either cv::MatND
 * or cv::SparseMat. The sparse matrix is probably the best one to use, since 
 * only a small region of colors are really present in the learning image set.
 */
template <class T>
class HistogramHS : public Histogram<T> {
public:
   HistogramHS(){;}
   
   /**
    * Create a histogram for hue & saturation from 1 image
    */
    T getHistogram(const cv::Mat &image, int hbins=256, int sbins=256) {
        
        T hist;
        
        // Convert to HSV color space
        cv::Mat hsv;
        cv::cvtColor(image, hsv, CV_BGR2HSV);
        
        // Prepare arguments for a 2D hue histogram
        float hranges[]= {0.0, 180.0};
        float sranges[] = {0.0, 255.0};
        const float *ranges[] = {hranges,sranges};
        int histSize[] = {hbins, sbins};
        int channels[] = {0, 1}; // hue & saturation channel
        
        // Compute histogram from base class
        return hist = Histogram<T>::getHistogram2D(hsv,ranges,histSize,channels);
	}  
   
   /**
    * Create a histogram for hue & saturation from many images.
    * \input image array in BGR format 
    */
    T getHistogramBGR(cv::Mat *image, const int num, int hbins=256, int sbins=256) {
        
        T hist;
        
        // Convert to HSV color space
        cv::Mat *hsv = new cv::Mat[num];
        for(int i=0;i<num;i++) cv::cvtColor(image[i], hsv[i], CV_BGR2HSV);
        
        // Prepare arguments for a 2D hue histogram
        float hranges[]= {0.0, 180.0};
        float sranges[] = {0.0, 255.0};
        const float *ranges[] = {hranges,sranges};
        int histSize[] = {hbins, sbins};
        int channels[] = {0, 1}; // hue & saturation channel
        
        //return hist;
        // Compute histogram from base class
        return hist = Histogram<T>::getHistogram2D(hsv,num,ranges,histSize,channels);
	}
	
   /**
    * Create a histogram for hue & saturation from many images.
    * \input hsv image array already converted to HSV
    */
    T getHistogramHSV(cv::Mat *hsv, const int num, int hbins=256, int sbins=256) {
        
        T hist;
        
        // Prepare arguments for a 2D hue histogram
        float hranges[]= {0.0, 180.0};
        float sranges[] = {0.0, 255.0};
        const float *ranges[] = {hranges,sranges};
        int histSize[] = {hbins, sbins};
        int channels[] = {0, 1}; // hue & saturation channel
        
        //return hist;
        // Compute histogram from base class
        return hist = Histogram<T>::getHistogram2D(hsv,num,ranges,histSize,channels);
	}
   
};



#endif
