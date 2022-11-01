#include "../../include/sfm/sfm_tests.hpp"

//** visualizes all the features corresponding to a particular frame in the pointcloud
void simple_sfm::SfmTest::showPCLPointsForFrameIdx(const int &frame_idx_, const std::vector<CloudPoint3d> &pointcloud_, const std::vector<cv::String> &mFrames_){

    cv::Mat base_img_ = cv::imread(mFrames_[frame_idx_].c_str());

    const Features &f_ = Frame::extractFeaturesAndDescriptors(base_img_);

    for(const auto &t: f_.points) {

        cv::circle(base_img_, t, 3, cv::viz::Color::yellow(), 1);

    }
    
    int pcl_size_ = (int)pointcloud_.size();

    int frame_pts_cnt_ = 0;

    const Points2d &points_ = f_.points;

    for(const auto &t: pointcloud_){

        if(t.view_idx_ == frame_idx_){
            
            cv::Point2d p_ = points_[t.feature_idx_];
            cv::circle(base_img_, p_, 1, cv::viz::Color::red(), 2);
            frame_pts_cnt_++;
        }

    }

    std::cout << "pcl.size(): " << pcl_size_ << " frame_pts_cnt_: " << frame_pts_cnt_ << std::endl;

    cv::imshow("x", base_img_);
    cv::waitKey(0);
}

void simple_sfm::SfmTest::projectPCLOnFrameIdx(const int frame_idx_, 
                                                const std::vector<cv::Matx34f> &mCameraPoses_,
                                                const std::vector<cv::String> mFrames_, 
                                                const std::vector<CloudPoint3d> &pointcloud_,
                                                const cv::Matx33f &K_){

    //cv::String s_ = "/home/skpro19/simple_sfm/data/00/image_0/000005.png";
    cv::Mat base_img_ = cv::imread(mFrames_[frame_idx_]);
                            
    std::cout << "==================== START OF projectPCLOnFrameIdx =============================" << std::endl << std::endl;

    std::vector<cv::Point3f> v_; 
    for(const auto &t: pointcloud_) v_.push_back(t.point_);

    
    cv::Mat mat_4d_, mat_3d_ = cv::Mat(v_).reshape(1).t();
    SfmUtil::convertToHomogeneous(mat_3d_, mat_4d_);
    
    cv::Matx34f C2_ = mCameraPoses_[frame_idx_];

    cv::Mat x_,  x_hom_ = cv::Mat(K_) * cv::Mat(C2_) * mat_4d_.t();
    SfmUtil::convertFromHomogeneous(x_hom_ , x_);   // x_ ===> (2516 * 2) , x_hom_ ==> (3 * 2516)
    /*const cv::Mat x_copy_ = x_;

    std::cout << "$$$$$$$$$$$$$$  BEFORE PASSING $$$$$$$$$$$$$$" << std::endl;
        */
    for(int i = 0 ;i  < 10; i++) std::cout << x_.row(i) << std::endl;

    /*
    //cv::Mat a_ = cv::imread(mFrames_[0].c_str());
    
    //Vis::draw2DPoints(cv::imread(mFrames_[frame_idx_].c_str()), x_);
    
    std::cout << "$$$$$$$$$$$$$$  AFTER PASSING $$$$$$$$$$$$$$" << std::endl;
    for(int i = 0 ;i  < 10; i++) std::cout << x_copy_.row(i) << std::endl;
    //for(int i = 0 ;i  < 10; i++) std::cout << x_.at<float>(i,0) <<"," << x_.at<float>(i,1) << std::endl;
    */

   // cv::imshow("base_img_", base_img_);
    std::cout << "HEELO!" << std::endl;
    Vis::draw2DPoints(base_img_, x_);

    std::cout << "==================== END OF projectPCLOnFrameIdx =============================" << std::endl << std::endl;


}