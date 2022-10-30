#ifndef SFM_HELPERS_H
#define SFM_HELPERS_H


#include <opencv2/opencv.hpp>

#include "ds.hpp"
#include "frame.hpp"
#include "sfm_util.hpp"
#include "vis.hpp"

namespace simple_sfm{


    class SfmHelper{

        public: 

            static bool findCameraMatrices(cv::Matx34f &P1_,
                                            cv::Matx34f &P2_,
                                            const Features &f1_,
                                            const Features &f2_,
                                            const Matches &matches_,
                                            const cv::Matx33f &K_,
                                            Matches &pruned_matches_);
            
            static bool triangulateViews(const cv::Mat &img_a_, 
                                        const cv::Mat &img_b_, 
                                        const cv::Matx34d &P1_, 
                                        const cv::Matx34d &P2_, 
                                        const cv::Matx33f &K_,
                                        std::vector<CloudPoint3d> &pointcloud_,
                                        const Matches &pruned_matches_);


            static void visualizeCloudPointProjections(const cv::Matx34f &P1_, 
                                                const cv::Matx34f &P2_, 
                                                const std::vector<CloudPoint3d> &cloudpoints_,
                                                const cv::Matx33f &K_,
                                                const cv::Mat &base_img_ = cv::Mat::zeros(376, 1241, CV_8UC3));




        private:



    };


};



#endif