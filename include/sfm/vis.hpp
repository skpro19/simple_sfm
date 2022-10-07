#ifndef VIS_H
#define VIS_H


//  ===========================================================================
//  =============   Visualization related functions  ==========================
//  ===========================================================================

#include "ds.hpp"
#include <opencv2/viz/types.hpp>



namespace simple_sfm {

    class Vis {

        public: 

            Vis();

            static void drawKeyPoints(cv::Mat &mat_, const Points2D &kp_a_, const Points2D &kp_b_);
            static void drawKeyPoints(cv::Mat &mat_, const Points2D &kp_a_);
            static void displayFrame(const cv::Mat &frame_);


            static void updateGroundPose(const cv::Matx34d &pose_); 
            static void updatePredictedPose(const cv::Mat &pose_);
            static void updateBAPose(const cv::Mat &pose_);


        private:

            static inline cv::Mat gt_mat_ = cv::Mat::zeros(1000, 1000, CV_8UC3);
            static inline cv::Mat pred_mat_ = cv::Mat::zeros(1000, 1000, CV_8UC3);
            
           
    };


};

#endif