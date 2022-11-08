#include "../../include/sfm/core_sfm.hpp"
#include "../../include/sfm/sfm_util.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/viz/types.hpp>



simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
   
    updateIOParams();
    mCameraPoses_.resize(mFrames_.size(), cv::Matx34f());

    initializeSFM();

   //addView(2);
    //addView(3);
    //addView(4);
    //addView(5);
    //addView(6);

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

    flag_ = SfmHelper::triangulateViews(f1_, f2_, 1, C1_, C2_, K_, lastPCL_, pruned_matches_);

    assert(flag_);

    globalPCL_ = lastPCL_;

    std::cout << "mPointcloud.size(): " << globalPCL_.size() << std::endl;
    
    //SfmHelper::visualizeCloudPointProjections(C1_, C2_, lastPointCloud_, K_, img_a_);
    SfmTest::projectPCLOnFrameIdx(1, mCameraPoses_, mFrames_, lastPCL_, K_);
    //SfmTest::showPCLPointsForFrameIdx(1, lastPointCloud_, mFrames_);

}




/*void simple_sfm::SimpleSFM::addView(const int frame_idx_){

    SfmUtil::startSeparator("addView");
    std::cout << "FRAME_IDX ===> " << frame_idx_ << std::endl;

    assert(frame_idx_ > 1);
    
    const cv::Mat &img_a_ = cv::imread(mFrames_[frame_idx_ - 1].c_str());
    const cv::Mat &img_b_ = cv::imread(mFrames_[frame_idx_].c_str());
    
    const Features &f1_ = Frame::extractFeaturesAndDescriptors(img_a_);
    const Features &f2_ = Frame::extractFeaturesAndDescriptors(img_b_);

    std::cout << "f1_.size(): " << f1_.points.size() <<  " f2_.size(): " << f2_.points.size() << std::endl;

    const Matches &matches_ = Frame::getMatches(f1_, f2_);

    std::cout << "matches_.size(): " << matches_.size() << std::endl;
    
    Matches pruned_matches_;

    cv::Matx34f C1_, C2_; 
    bool flag_ = SfmHelper::findCameraPoseMatrices(C1_, C2_,f1_, f2_, matches_, K_, pruned_matches_);
    
    std::cout << "pruned_matches_.size(): " << pruned_matches_.size() << std::endl;
    
    Match2D3D match2d3d_ =  SfmHelper::get2D3DMatches(frame_idx_, lastPCL_, f1_, f2_, pruned_matches_);

    //std::cout << "match2d3d_.size(): " << match2d3d_.pts_2d_.size() << std::endl;
    
    cv::Mat camera_pose_;
    flag_ = SfmHelper::updateCameraPoseFrom2D3DMatch(camera_pose_, match2d3d_, K_);
 
    //std::cout << "camera_pose_: " << camera_pose_ << std::endl;

    mCameraPoses_[frame_idx_] = cv::Matx34f(camera_pose_);

    //SfmTest::showPCLPointsForFrameIdx(frame_idx_, lastPCL_, mFrames_);
    SfmTest::projectPCLOnFrameIdx(frame_idx_, mCameraPoses_, mFrames_, lastPCL_, K_);

    //std::cout << "updateCameraPoseFrom2D3DMatch FLAG ---> " << flag_ << std::endl;
    
    //std::cout << "mCameraPoses_[frame_idx]: " << mCameraPoses_[frame_idx_] << std::endl;

    assert(flag_);
    lastPCL_.resize(0);

    //TODO --> replace frame_idx_ - 1 with last_camera_pose_
    flag_ = SfmHelper::triangulateViews(f1_, f2_, 
                                        frame_idx_, 
                                        mCameraPoses_[frame_idx_ - 1], mCameraPoses_[frame_idx_], 
                                        K_, lastPCL_, pruned_matches_);

    std::cout << "lastPCL_.size() after triangulation ---> " << lastPCL_.size() << std::endl;

    assert(flag_);

    std::cout << "before updating ---> globalPCL.size(): " << globalPCL_.size() << std::endl;

    SfmHelper::updateGlobalPCL(lastPCL_, globalPCL_, f1_, f2_, pruned_matches_);
    
    std::cout << "after updating ---> globalPCL.size(): " << globalPCL_.size() << std::endl;
    
    SfmUtil::endSeparator("addView");

    
}*/