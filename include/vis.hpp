#ifndef VIS_H
#define VIS_H

#include "visual_odom.hpp"

namespace simple_sfm{

    class Vis {

        public:

            Vis();

            void viusalizeCameraPose(const cv::Matx44d &pose_);

        private:


            cv::Mat predictions_mat_ = cv::Mat::zeros(800, 500, CV_8UC3);
        




    };





};


#endif