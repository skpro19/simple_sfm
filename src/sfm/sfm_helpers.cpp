#include "../../include/sfm/sfm_helpers.hpp"


void simple_sfm::SfmHelper::updateGlobalPCL(const std::vector<CloudPoint3d> &lastPCL, 
                                            std::vector<CloudPoint3d>&globalPCL,
                                            const Features &f1_,
                                            const Features &f2_,
                                            const Matches &pruned_matches_){
    
    int merge_cnt_ = 0 ;
    int new_pts_cnt_  = 0 ;

    std::vector<CloudPoint3d> pointsToAdd_;

    for(const CloudPoint3d &new_pt_ : lastPCL){

       for(auto &existing_pt_: globalPCL){

            const double norm_ = cv::norm(new_pt_.point_ - existing_pt_.point_);

            if(norm_ < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {

                int train_idx_ = new_pt_.feature_idx_;
                int query_idx_ = existing_pt_.feature_idx_;

                bool match_flag_ = false;

                for(const auto &match_ : pruned_matches_){

                    match_flag_ = (match_.queryIdx == query_idx_ && match_.trainIdx == train_idx_);

                    if(match_flag_) {

                        merge_cnt_++;
                        break;

                    }
                }

                if(match_flag_){

                    existing_pt_.view_idx_ = new_pt_.view_idx_;
                    existing_pt_.feature_idx_ = new_pt_.feature_idx_;

                }
                else{

                    pointsToAdd_.push_back(new_pt_);

                }           
            }
       }
    }

    for(const auto &t: pointsToAdd_) globalPCL.push_back(t);

    ///assert(merge_cnt_ == pointsToAdd_.size());

    std::cout << "merge_cnt_: " << merge_cnt_ << std::endl;
    std::cout << "new_pts_added: " << pointsToAdd_.size() << std::endl;

    assert(pointsToAdd_.size() + merge_cnt_ == lastPCL.size()); 

}

bool simple_sfm::SfmHelper::updateCameraPoseFrom2D3DMatch(cv::Matx34f &camera_pose_, const Match2D3D &match2d3d_, const cv::Matx33f &K_){
    
    //std::cout << "Inside getBestViewIndexToMerge!" << std::endl;
    
    //std::cout << "n_: " << n_ << std::endl;

    int sz_= (int)match2d3d_.pts_2d_.size(); 

    bool success_ = false;

    cv::Mat rvec_, tvec_; 
    cv::Mat inliers_;

    //std::cout << "match2d3d.size(): " << match2d3d_.pts_2d_.size() << std::endl;

    success_ = cv::solvePnPRansac(match2d3d_.pts_3d_, match2d3d_.pts_2d_, K_, cv::Mat(), rvec_, tvec_,false,100,8.0, 0.99, inliers_);

    //std::cout << "match2d3d_pts_.size(): " << match2d3d_.pts_3d_.size() << std::endl;
    //std::cout << "pts_2d.size(): " << match2d3d_.pts_2d_.size() << std::endl;
    //std::cout << "inliers_cnt_: " << cv::countNonZero(inliers_) << std::endl;
    //std::cout << "success_: " << success_ << std::endl;

    if(!success_) {return success_; }

   // assert(inliers_.rows != match2d3d_.pts_2d_.size());

    //const double inlier_ratio_ = (1.0) * ((double)inliers_.rows/ (double)match2d3d_.pts_2d_.size());
    
    //const double inlier_ratio_ = (1.0) * ((double)inliers_.rows/ (double)match2d3d_.pts_2d_.size());
    const double inlier_ratio_ = (1.0) * ((double)cv::countNonZero(inliers_)/ (double)match2d3d_.pts_2d_.size());
    
    std::cout << "inliers_ratio_: " << inlier_ratio_ << std::endl;

    if(inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO) {

        //std::cout << "inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO" << std::endl;;
        return false;

    }

    //std::cout << "inlier_ratio_test_: "  << (inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO)  << std::endl;

    cv::Mat R_;
    cv::Rodrigues(rvec_, R_);

    const cv::Rect &ROT_    =       cv::Rect(0, 0, 3, 3);
    const cv::Rect &TRA_    =       cv::Rect(3, 0, 1, 3);

    
    R_.copyTo(cv::Mat(3, 4, CV_64FC1, camera_pose_.val)(ROT_));
    tvec_.copyTo(cv::Mat(3, 4, CV_64FC1, camera_pose_.val)(TRA_));

    //Vis::updatePredictedPose(cv::Mat(camera_pose_));
    
    return true;
    
}

Match2D3D simple_sfm::SfmHelper::get2D3DMatches(const int curr_view_idx_, 
                                                const std::vector<CloudPoint3d> &pointcloud_, 
                                                const Features &f1_, 
                                                const Features &f2_,
                                                const Matches &pruned_matches_){

    std::cout << "Inside get2D3d matches! " << std::endl;

    std::cout << "mPointcloud.size(): " << pointcloud_.size() << std::endl;
    
    Features f1_mat_, f2_mat_; 
    std::vector<int> ref_f1_, ref_f2_;
    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_, ref_f1_, ref_f2_, pruned_matches_);
    
    Match2D3D match_2d3d_;

    int match_cnt_ = 0 ;

    for(const auto &cloud_pt_: pointcloud_){

        int frame_idx_ = cloud_pt_.view_idx_;

        if(frame_idx_ != curr_view_idx_ - 1) {continue;}

        int feat_idx_ = cloud_pt_.feature_idx_;


        for(const auto &match_: pruned_matches_){

            int flag_ = (match_.queryIdx == feat_idx_);

            if(flag_) {
                
                match_cnt_++;
                match_2d3d_.pts_2d_.push_back(f2_.points[match_.trainIdx]);
                match_2d3d_.pts_3d_.push_back(cloud_pt_.point_);
                break;
            }
        }
    }

    std::cout << "pcl_.size(): " << pointcloud_.size() << std::endl;
    std::cout << "match_cnt_: " << match_cnt_ << std::endl;

    return match_2d3d_;  

}



//TODO --> check alignFeaturesUsingMatches for bug ; add inlier ratio check ; tune inlier ratio condition
bool simple_sfm::SfmHelper::findCameraPoseMatrices(cv::Matx34f &C1_ , 
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

bool simple_sfm::SfmHelper::triangulateViews(const Features &f1_, 
                                            const  Features &f2_,
                                            const int &frame_idx_,
                                            const cv::Matx34d &C1_, 
                                            const cv::Matx34d &C2_,
                                            const cv::Matx33f &K_, 
                                            std::vector<CloudPoint3d> &pointcloud_,
                                            const Matches &pruned_matches_){


    //const Features &f1_ = Frame::extractFeaturesAndDescriptors(img_a_); 
    //const Features &f2_ = Frame::extractFeaturesAndDescriptors(img_b_); 
    
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
        cloud_pt_.view_idx_ = frame_idx_;
        cloud_pt_.feature_idx_ = ref_f2_[k];
        
        
        pointcloud_.push_back(cloud_pt_);
        
    }

    std::cout << "pointcloud_.size(): " << pointcloud_.size() << std::endl;
    std::cout << "rejected_pts_.size(): " << rejected_3d_pts_cnt_ << std::endl;

    assert((int)pointcloud_.size() + rejected_3d_pts_cnt_ == pruned_matches_.size());    

    std::cout << "==================== END OF triangulateViews =============================" << std::endl << std::endl;

    return true;
    
}

void simple_sfm::SfmHelper::visualizeCloudPointProjections(const cv::Matx34f &C1_, 
                                                            const cv::Matx34f &C2_, 
                                                            const std::vector<CloudPoint3d> &pointcloud_,
                                                            const cv::Matx33f &K_,
                                                            const cv::Mat &base_img_){


                            
    std::cout << "==================== START OF visualizeCloudPointProjections =============================" << std::endl << std::endl;

    std::vector<cv::Point3f> v_; 
    for(const auto &t: pointcloud_) v_.push_back(t.point_);

    
    cv::Mat mat_4d_, mat_3d_ = cv::Mat(v_).reshape(1).t();
    SfmUtil::convertToHomogeneous(mat_3d_, mat_4d_);
    
    cv::Mat x_,  x_hom_ = cv::Mat(K_) * cv::Mat(C2_) * mat_4d_.t();
    SfmUtil::convertFromHomogeneous(x_hom_ , x_);   // x_ ===> (2516 * 2) , x_hom_ ==> (3 * 2516)

    for(int i = 0 ;i  < 10; i++) std::cout << x_.row(i)  << std::endl;
   
    Vis::draw2DPoints(base_img_, x_);

    std::cout << "==================== END OF visualizeCloudPointProjections =============================" << std::endl << std::endl;


}

