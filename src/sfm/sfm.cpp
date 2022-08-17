#include "../../include/sfm/sfm.hpp"


simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) {

    io_ = std::make_shared<SFM_IO>(base_folder_);

    updateIOParams();

    //Frame::orb_ = cv::ORB::create(5000);

    
}

void simple_sfm::SimpleSFM::updateIOParams() {

    io_->getImageFileNames(image_file_list_);
    io_->getGTPoses(gt_poses_);
    
    P0_ = io_->getP0();
    K_  = io_->getK(); 
    t0_ = io_->gett0();
    R0_ = io_->getR0();

    F0_ = image_file_list_[0]; 
    F1_ = image_file_list_[1];

}

void simple_sfm::SimpleSFM::InitializeSFMPipeline() {

    std::cout << "[sfm] InitializeSFMPipeline function!" << std::endl;

    bool initialized_ = false; 

    cv::Mat E_, E_mask_;
    cv:: Mat R, t;
    
    Points2D pts_curr_, pts_last_;

    int last_idx_ = 0 ;

    int curr_idx_ = 1;

    int inlier_cnt_ = 0 ;

    double scale_; 

    while(!initialized_) {
        
        std::cout << "curr_idx_: " << curr_idx_ << std::endl;

        pts_curr_.resize(0);
        pts_last_.resize(0);

        F0_ = image_file_list_[last_idx_];
        F1_ = image_file_list_[curr_idx_];

        Frame::Points2DFromFrames(F0_, F1_, pts_last_, pts_curr_);

        
        E_ = cv::findEssentialMat(pts_curr_, pts_last_, K_, cv::RANSAC, 0.999, 1.0, E_mask_);

        
        inlier_cnt_ = cv::recoverPose(E_, pts_curr_, pts_last_, K_, R, t, E_mask_);

        
        //using ground truth for scale estimation
        std::cout << "[sfm] gt_poses_.size(): " << (int)gt_poses_.size() << std::endl;

        scale_ = Frame::GetAbsoluteScale(gt_poses_[last_idx_], gt_poses_[curr_idx_]);

        std::cout << "scale_: " << scale_ << std::endl;

        curr_idx_++; 

        if(inlier_cnt_ > 25 && scale_ > 0.1) {

            std::cout << "[sfm.cpp] Valid initial points pair found for sfm pipeline!" << std::endl;
            std::cout << "last_idx_: " << last_idx_  << " curr_idx_: " << curr_idx_ << std::endl;
            std::cout << "R: " << R << " t: " << t << std::endl;
            initialized_ = true;
        }

        else {

            std::cout << "[sfm.cpp] Invalid initial points pair -- search for next point!" << std::endl;

        }

        
    }



}

