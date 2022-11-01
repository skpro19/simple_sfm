#include "../../include/sfm/core_sfm.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/viz/types.hpp>



simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
   
    updateIOParams();
    mCameraPoses_.resize(mFrames_.size(), cv::Matx34f());

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

    //cv::imshow("img_a", img_a_);
    // /cv::waitKey(0);

    Features f1_ = Frame::extractFeaturesAndDescriptors(img_a_);
    Features f2_ = Frame::extractFeaturesAndDescriptors(img_b_);
    
    Matches matches_ = Frame::getMatches(f1_, f2_);

    Matches pruned_matches_;
    
    bool flag_ = SfmHelper::findCameraPoseMatrices(C1_, C2_,f1_, f2_, matches_, K_, pruned_matches_);

    mCameraPoses_[0] = C1_; 
    mCameraPoses_[1] = C2_;

    std::cout << "pruned_matches_.size(): " << pruned_matches_.size() << std::endl;

    assert(flag_);

    flag_ = SfmHelper::triangulateViews(f1_, f2_, 1, C1_, C2_, K_, lastPointCloud_, pruned_matches_);

    assert(flag_);

    std::cout << "mPointcloud.size(): " << lastPointCloud_.size() << std::endl;
    
//    SfmHelper::projectPCLOnFrameIdx2(1, mFrames_, mCameraPoses_, lastPointCloud_, K_, img_a_);
    
    //SfmHelper::visualizeCloudPointProjections(C1_, C2_, lastPointCloud_, K_, img_a_);
    //SfmTest::projectPCLOnFrameIdx(1, mCameraPoses_, mFrames_, lastPointCloud_, K_);
    SfmTest::showPCLPointsForFrameIdx(1, lastPointCloud_, mFrames_);

}



void simple_sfm::SimpleSFM::addView(const int frame_idx_){

    assert(frame_idx_ > 1);
    
    const cv::Mat &img_a_ = cv::imread(mFrames_[frame_idx_ - 1].c_str());
    const cv::Mat &img_b_ = cv::imread(mFrames_[frame_idx_].c_str());
    
    const Features &f1_ = Frame::extractFeaturesAndDescriptors(img_a_);
    const Features &f2_ = Frame::extractFeaturesAndDescriptors(img_b_);

    const Matches &matches_ = Frame::getMatches(f1_, f2_);
    
    Matches pruned_matches_;

    cv::Matx34f C1_, C2_; 
    bool flag_ = SfmHelper::findCameraPoseMatrices(C1_, C2_,f1_, f2_, matches_, K_, pruned_matches_);
    
    std::cout << "pruned_matches_.size(): " << pruned_matches_.size() << std::endl;



    
    //SfmHelper::get2D3DMatches(frame_idx_, mPointCloud_, f1_, f2_, pruned_matches_);



    //Match2D3D match2d3d_ = sf


    

}