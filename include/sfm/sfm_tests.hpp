#ifndef UNIT_TESTS_H
#define UNIT_TESTS_H

#include <opencv2/viz/types.hpp>
#include <opencv2/opencv.hpp>

#include "frame.hpp"
#include "sfm_util.hpp"
#include "vis.hpp"

namespace simple_sfm{


    class SfmTest{

        public: 

            static void showPCLPointsForFrameIdx(const int &frame_idx_, 
                                                const std::vector<CloudPoint3d> &pointcloud_, 
                                                const std::vector<cv::String> &mFrames_);


            static void projectPCLOnFrameIdx(const int frame_idx_, 
                                                const std::vector<cv::Matx34f> &mCameraPoses_,
                                                const std::vector<cv::String> mFrames_, 
                                                const std::vector<CloudPoint3d> &pointcloud_,
                                                const cv::Matx33f &K_);
        private:



    };


};



#endif