#include "../../include/sfm/core_sfm.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/viz/types.hpp>



simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
   
    updateIOParams();

    initializeSFM();

}





void simple_sfm::SimpleSFM::initializeSFM(){

    cv::Matx34f P1_, P2_; 
    
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

    bool flag_ = findCameraMatrices(P1_, P2_,f1_, f2_, matches_, pruned_matches_);
    
    std::cout << "pruned_matches_.size(): " << pruned_matches_.size() << std::endl;

    assert(flag_);

    //std::cout << "P1_: " << P1_ << std::endl;
    //std::cout << "P2_: " << P2_ << std::endl;
    cv::Mat A_, R_, t_hom_, t_;
    
    cv::decomposeProjectionMatrix(P2_, K_, R_, t_hom_);
    std::cout << "t_hom_: " << t_hom_ << std::endl;
    
    cv::convertPointsFromHomogeneous(t_hom_.t(), t_);

    std::cout << "t_: " << t_ << std::endl;

    
    flag_ = triangulateViews(img_a_, img_b_, P1_, P2_, mPointCloud_);

    std::cout << "mPointcloud.size(): " << mPointCloud_.size() << std::endl;

    visualizeCloudPointProjections(P1_, P2_, mPointCloud_);


}