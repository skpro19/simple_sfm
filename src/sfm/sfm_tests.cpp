#include "../../include/sfm/sfm_tests.hpp"

//** visualizes all the features corresponding to a particular frame in the pointcloud
void simple_sfm::SfmTest::showPCLPointsForFrameIdx(const int &frame_idx_, const std::vector<CloudPoint3d> &pointcloud_, const std::vector<cv::String> &mFrames_){

    cv::Mat base_img_ = cv::imread(mFrames_[frame_idx_].c_str());

    const Features &f_ = Frame::extractFeaturesAndDescriptors(base_img_);

    for(const auto &t: f_.points) {

        cv::circle(base_img_, t, 1, cv::viz::Color::cherry(), 1);

    }
    
    const Points2d &points_ = f_.points;

    for(const auto &t: pointcloud_){

        if(t.view_idx_ == frame_idx_){
            
            cv::Point2d p_ = points_[t.feature_idx_];
            cv::circle(base_img_, p_, 2, cv::viz::Color::yellow(), 1);
        
        }

    }

    cv::imshow("x", base_img_);
    cv::waitKey(0);
}