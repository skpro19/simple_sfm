#ifndef UNIT_TESTS_H
#define UNIT_TESTS_H

#include <opencv2/viz/types.hpp>
#include <opencv2/opencv.hpp>

#include "ds.hpp"
#include "frame.hpp"

namespace simple_sfm{


    class SfmTest{

        public: 

            static void showPCLPointsForFrameIdx(const int &frame_idx_, const std::vector<CloudPoint3d> &pointcloud_, const std::vector<cv::String> &mFrames_);
        
        private:



    };


};



#endif