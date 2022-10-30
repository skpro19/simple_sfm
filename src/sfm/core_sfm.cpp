#include "../../include/sfm/core_sfm.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/viz/types.hpp>



simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
   
    updateIOParams();

    initializeSFM();

}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(mFrames_);
    io_->getGTPoses(gt_poses_);
    
    K_  = io_->getK(); 

}


void simple_sfm::SimpleSFM::initializeSFM(){

    cv::Matx34f C1_, C2_; 
    
    assert(mFrames_.size() >= 2);

    const cv::Mat &img_a_ = cv::imread(mFrames_[0].c_str());
    const cv::Mat &img_b_ = cv::imread(mFrames_[1].c_str());
    
    std::cout << "img_a.size(): " << img_a_.size() << std::endl;

    cv::imshow("img_a", img_a_);
    cv::waitKey(0);

    Features f1_ = Frame::extractFeaturesAndDescriptors(img_a_);
    Features f2_ = Frame::extractFeaturesAndDescriptors(img_b_);
    
    Matches matches_ = Frame::getMatches(f1_, f2_);

    Matches pruned_matches_;
    
    bool flag_ = SfmHelper::findCameraMatrices(C1_, C2_,f1_, f2_, matches_, K_, pruned_matches_);
    
    std::cout << "pruned_matches_.size(): " << pruned_matches_.size() << std::endl;

    assert(flag_);

    flag_ = SfmHelper::triangulateViews(img_a_, img_b_, C1_, C2_, K_, mPointCloud_, pruned_matches_);

    assert(flag_);

    std::cout << "mPointcloud.size(): " << mPointCloud_.size() << std::endl;
    
    SfmHelper::visualizeCloudPointProjections(C1_, C2_, mPointCloud_, K_, img_a_);


}