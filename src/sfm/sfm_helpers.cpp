#include "../../include/sfm/sfm_helpers.hpp"





//TODO --> check alignFeaturesUsingMatches for bug ; add inlier ratio check ; tune inlier ratio condition
bool simple_sfm::SfmHelper::findCameraMatrices(cv::Matx34f &C1_ , 
                                                cv::Matx34f &C2_, 
                                                const Features &f1_, 
                                                const Features &f2_, 
                                                const Matches &matches_,
                                                const cv::Matx33f &K_, 
                                                Matches &pruned_matches_){


    //std::cout << "########################################################################" << std::endl;
     std::cout << "==================== START OF findCameraMatrices =============================" << std::endl;

    
    cv::Mat E_, R, t, inlier_mask_;

    
    Features f1_mat_, f2_mat_; 
    std::vector<int> ref_f1_, ref_f2_;

    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_, ref_f1_, ref_f2_, matches_);
    
    const Points2d &pts1_ = f1_mat_.points;
    const Points2d &pts2_ = f2_mat_.points;


    std::cout << "pts1_.size(): " << pts1_.size() << " pts2_.size(): " << pts2_.size() << std::endl;
    
    E_ = cv::findEssentialMat(pts1_, pts2_, K_,cv::RANSAC, 0.999, 1.0, inlier_mask_);
    
    std::cout << "E_: " << E_ << std::endl;

    const int inlier_cnt_ = cv::recoverPose(E_, pts1_, pts2_, K_, R, t, inlier_mask_);

    std::cout << "R_: " << R << " t: " << t << std::endl;

    std::cout << "inlier_cnt_: " << inlier_cnt_ << std::endl;

    if(inlier_cnt_ < POSE_INLIERS_MINIMAL_COUNT) { return false;}

    C1_ = cv::Matx34f::eye();

    std::cout << "C1_: " << C1_ << std::endl;

    C2_ = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));


    std::cout << "C2_: " << C2_ << std::endl;

    pruned_matches_.resize(0);

    for(int i = 0 ; i < inlier_mask_.rows; i++) {

        if(inlier_mask_.at<uchar>(i)){

            pruned_matches_.push_back(matches_[i]);

        }
    }   

    std::cout << "==================== END OF findCameraMatrices =============================" << std::endl << std::endl;

    return true;
   
}

bool simple_sfm::SfmHelper::triangulateViews(const cv::Mat &img_a_, 
                                            const cv::Mat &img_b_,
                                            const cv::Matx34d &C1_, 
                                            const cv::Matx34d &C2_,
                                            const cv::Matx33f &K_, 
                                            std::vector<CloudPoint3d> &pointcloud_,
                                            const Matches &pruned_matches_){


    const Features &f1_ = Frame::extractFeaturesAndDescriptors(img_a_); 
    const Features &f2_ = Frame::extractFeaturesAndDescriptors(img_b_); 
    
    Features f1_mat_, f2_mat_;
    std::vector<int> ref_f1_, ref_f2_;
    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_,ref_f1_, ref_f2_,  pruned_matches_);

    cv::Mat normalized_pts_1_, normalized_pts_2_;
    
    cv::undistortPoints(f1_mat_.points, normalized_pts_1_, K_, cv::Mat());
    cv::undistortPoints(f2_mat_.points, normalized_pts_2_, K_, cv::Mat());
    
    cv::Mat pts_4d_;
    cv::triangulatePoints(C1_, C2_, normalized_pts_1_, normalized_pts_2_, pts_4d_);
    
    cv::Mat pts_3d_;
    cv::convertPointsFromHomogeneous(pts_4d_.t(), pts_3d_);
    
    cv::Mat rvec1_;
    cv::Rodrigues(C1_.get_minor<3, 3>(0,0), rvec1_);
    cv::Mat tvec1_(C1_.get_minor<3,1>(0,3).t());
    
    cv::Mat rvec2_;
    cv::Rodrigues(C2_.get_minor<3, 3>(0,0), rvec2_);
    cv::Mat tvec2_(C2_.get_minor<3,1>(0,3).t());
    
    Points2d projected_pts_1_, projected_pts_2_;
    cv::projectPoints(pts_3d_, rvec1_, tvec1_, K_, cv::Mat(), projected_pts_1_);
    cv::projectPoints(pts_3d_, rvec2_, tvec2_, K_, cv::Mat(), projected_pts_2_);

    int rejected_3d_pts_cnt_ = 0 ;
    for(int k = 0; k < pts_3d_.rows; k++){

        double d1_ = cv::norm(projected_pts_1_[k] - f1_mat_.points[k]);
        double d2_ = cv::norm(projected_pts_2_[k] - f2_mat_.points[k]);

        if(d1_ > MIN_REPROJECTION_ERROR or d2_ > MIN_REPROJECTION_ERROR){
            
            rejected_3d_pts_cnt_++;
            continue;

        }

        CloudPoint3d cloud_pt_;
        cloud_pt_.point_ = cv::Point3d{pts_3d_.at<double>(k,0), pts_3d_.at<double>(k,1), pts_3d_.at<double>(k,2)};

        pointcloud_.push_back(cloud_pt_);
        
    }

    std::cout << "pointcloud_.size(): " << pointcloud_.size() << std::endl;
    std::cout << "rejected_pts_.size(): " << rejected_3d_pts_cnt_ << std::endl;

    assert((int)pointcloud_.size() + rejected_3d_pts_cnt_ == pruned_matches_.size());    

    std::cout << "==================== END OF triangulateViews =============================" << std::endl << std::endl;

    return true;
    
}


void simple_sfm::SfmHelper::visualizeCloudPointProjections(const cv::Matx34f &P1_, 
                                                            const cv::Matx34f &P2_, 
                                                            const std::vector<CloudPoint3d> &pointcloud_,
                                                            const cv::Matx33f &K_,
                                                            const cv::Mat &base_img_){


                            
    std::cout << "==================== START OF visualizeCloudPointProjections =============================" << std::endl << std::endl;

    std::vector<cv::Point3f> v_; 
    for(const auto &t: pointcloud_) v_.push_back(t.point_);

    
    cv::Mat mat_4d_, mat_3d_ = cv::Mat(v_).reshape(1).t();
    SfmUtil::convertToHomogeneous(mat_3d_, mat_4d_);
    
    cv::Mat x_,  x_hom_ = cv::Mat(K_) * cv::Mat(P2_) * mat_4d_.t();
    SfmUtil::convertFromHomogeneous(x_hom_ , x_);   // x_ ===> (2516 * 2) , x_hom_ ==> (3 * 2516)

   
    Vis::draw2DPoints(base_img_, x_);

    std::cout << "==================== END OF visualizeCloudPointProjections =============================" << std::endl << std::endl;


}
