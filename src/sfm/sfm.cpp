#include "../../include/sfm/sfm.hpp"

#include <opencv2/calib3d.hpp>

#include <opencv2/viz/types.hpp>

simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
   
    updateIOParams();

    //ceres::Problem::Options problem_options_;
    //ceres::Problem problem(problem_options_);

}


void simple_sfm::SimpleSFM::createFeatureMatrix(){

    int num_frames_ = mFrames_.size();

    mFeatures_.resize(num_frames_);


    for(int i = 0 ;i < num_frames_; i++) {

        cv::Mat img_ = cv::imread(mFrames_[i].c_str());
        
        Features features_;
        Frame::extractFeaturesAndDescriptors(img_, features_);
        Frame::keypointsToPoints(features_);        
        mFeatures_[i] = features_;
    
    }

}

void simple_sfm::SimpleSFM::createFeatureMatchMatrix(){

    int n_ = mFrames_.size(); 

    mMFeatureMatches_.resize(n_);

    for(int i = 0 ; i < n_ ; i++) {

        for(int j = i + 1; i < n_ ; i++) {

            Matches matches_ = Frame::getMatches(mFeatures_[i], mFeatures_[j]);

            mMFeatureMatches_[i].push_back(matches_);

        }
    }
}



std::map<float, ImagePair> simple_sfm::SimpleSFM::sortViewsByHomography(){

    std::map<float, ImagePair> homography_ratio_map_;

    const int n_ = (int)mFrames_.size(); 

    for(int i = 0 ; i < n_ ; i++) {

        for(int j = i + 1; j < n_; j++) {

            int match_sz_ = (int)mMFeatureMatches_[i][j].size();

            if(match_sz_ < MIN_POINT_COUNT_FOR_HOMOGRAPHY) {

                homography_ratio_map_[1.0] = ImagePair{i,j};
                continue;
            }
            
            
            int inliers_cnt_ = Frame::getHomographyInliersCount(mFeatures_[i], mFeatures_[j], mMFeatureMatches_[i][j]);

            double inlier_ratio_ = 1.0 * (double)inliers_cnt_/ (double)mMFeatureMatches_[i][j].size(); 

            homography_ratio_map_[inlier_ratio_] = ImagePair{i,j};

        }

    }

    return homography_ratio_map_;

}

bool simple_sfm::SimpleSFM::findCameraMatrices(cv::Matx34d &P1_ , cv::Matx34d &P2_, const ImagePair &img_pair_, Matches &pruned_matches_){

    assert(img_pair_.first < img_pair_.second);

    cv::Mat E_, R, t, inlier_mask_;

    const KeyPoints &kp1_ = mFeatures_[img_pair_.first].keypoints;
    const KeyPoints &kp2_ = mFeatures_[img_pair_.second].keypoints;
    
    const Points2d &pts1_ = mFeatures_[img_pair_.first].points;
    const Points2d &pts2_ = mFeatures_[img_pair_.second].points;
    

    E_ = cv::findEssentialMat(pts1_, pts2_, K_,cv::RANSAC, 0.999, 1.0, inlier_mask_);


    const int inlier_cnt_ = cv::recoverPose(E_, pts1_, pts2_, K_, R, t, inlier_mask_);

    if(inlier_cnt_ < POSE_INLIERS_MINIMAL_COUNT) { return false;}

    P1_ = cv::Matx34d::eye();

    P2_ = cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    pruned_matches_.resize(0);

    const Matches &matches_ = mMFeatureMatches_[img_pair_.first][img_pair_.second];
        
    for(int i = 0 ; i < inlier_mask_.rows; i++) {

        if(inlier_mask_.at<uchar>(i)){

            pruned_matches_.push_back(matches_[i]);

        }
    }

    return true;
   
}


bool simple_sfm::SimpleSFM::triangulateViews(const cv::Matx34d &P1_, const cv::Matx34d &P2_, const ImagePair &img_pair_, std::vector<CloudPoint3d> &pointcloud_){

    const int i = img_pair_.first ;
    const int j = img_pair_.second; 

    assert(i < j);

    const Features &f1_ = mFeatures_[i]; 
    const Features &f2_ = mFeatures_[j]; 
    const Matches &matches_ = mMFeatureMatches_[i][j];

    Features f1_mat_, f2_mat_;
    std::vector<int> ref_f1_, ref_f2_;
    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_,ref_f1_, ref_f2_,  matches_);

    cv::Mat normalized_pts_1_, normalized_pts_2_;

    cv::undistortPoints(f1_mat_.points, normalized_pts_1_, K_, cv::Mat());
    cv::undistortPoints(f2_mat_.points, normalized_pts_2_, K_, cv::Mat());
    
    cv::Mat pts_4d_;
    cv::triangulatePoints(P1_, P2_, normalized_pts_1_, normalized_pts_2_, pts_4d_);

    cv::Mat pts_3d_;
    cv::convertPointsFromHomogeneous(pts_4d_.t(), pts_3d_);
    
    cv::Mat rvec1_;
    cv::Rodrigues(P1_.get_minor<3, 3>(0,0), rvec1_);
    cv::Mat tvec1_(P1_.get_minor<3,1>(0,3).t());
    

    cv::Mat rvec2_;
    cv::Rodrigues(P2_.get_minor<3, 3>(0,0), rvec2_);
    cv::Mat tvec2_(P2_.get_minor<3,1>(0,3).t());
    
    Points2d projected_pts_1_, projected_pts_2_;
    
    cv::projectPoints(pts_4d_, rvec1_, tvec1_, K_, cv::Mat(), projected_pts_1_);
    cv::projectPoints(pts_4d_, rvec2_, tvec2_, K_, cv::Mat(), projected_pts_2_);

    for(int k = 0; k < pts_3d_.rows; k++){

        double d1_ = cv::norm(projected_pts_1_[k] - f1_mat_.points[k]);
        double d2_ = cv::norm(projected_pts_2_[k] - f2_mat_.points[k]);

        if(d1_ > MIN_REPROJECTION_ERROR or d2_ > MIN_REPROJECTION_ERROR){

            continue;

        }

        CloudPoint3d cloud_pt_;
        cloud_pt_.point_ = cv::Point3d{pts_3d_.at<double>(k,0), pts_3d_.at<double>(k,1), pts_3d_.at<double>(k,2)};

        cloud_pt_.viewMap[i] = ref_f1_[k];
        cloud_pt_.viewMap[j] = ref_f2_[k];
        
        pointcloud_.push_back(cloud_pt_);
        

    }
    
    return true;
    
}

void simple_sfm::SimpleSFM::initializeBaselineSFM(){


    const std::map<float, ImagePair> homography_ratio_map_  = sortViewsByHomography();

    for(const auto &elem_: homography_ratio_map_){

        const float ratio_ = elem_.first;
        const ImagePair &img_pair_  = elem_.second;

        assert(img_pair_.first < img_pair_.second);

        cv::Matx34d P1_, P2_;

        Matches pruned_matches_;

        bool success_ = findCameraMatrices(P1_, P2_, img_pair_, pruned_matches_);
        
        if(not success_) {continue;}

        
        const double inlier_ratio_ = 1.0 * (double)pruned_matches_.size() / (double)mMFeatureMatches_[img_pair_.first][img_pair_.second].size();

        if (inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO) {

            success_ = false;
            continue;
        }


        mMFeatureMatches_[img_pair_.first][img_pair_.second] = pruned_matches_;


    }


    /*

        - triangulatePoints()

    */




}

void simple_sfm::SimpleSFM::runSFMPipeline() {

    createFeatureMatrix();

    createFeatureMatchMatrix();
    


    

    /*


        initializeBaselineSFM();
        addViewToSFM();    


    */
    


}



void simple_sfm::SimpleSFM::runBundleAdjust(int se_, int en_){

    std::cout << "Inside runBundleAdjustment function!" << std::endl;

    std::cout << "se: " << se_ << " en_: " << en_ << std::endl;

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
        
    //int n_ = (int)views_.size(); 

    //n_ = 20;

    int unused_cnt_ = 0 ;

    
    for(int i = 1 ; i <= en_; i++){
        
        if(views_[i] == nullptr){

            continue;

        }
        
        std::shared_ptr<View> view_(views_[i]);
        
        assert(view_->pts_2d_->size() == view_->indices_3d_pts_->size());

        int m_ = (int)view_->pts_2d_->size();

        Mat3d intrinsics_ = view_->cam_intrinsics_;
        Vec6d &extrinsics_ = view_->cam_extrinsics_;

        //std::cout << "index: " << i << std::endl;
        //std::cout << "extrinsics_: " << extrinsics_ << std::endl;
        
        for(int j = 0 ; j < m_; j++) {
            
            float x_= view_->pts_2d_->at(j)[0];
            float y_= view_->pts_2d_->at(j)[1];

            Vec3d &pt_ = View::point_cloud_[view_->indices_3d_pts_->at(j)];
            
            ceres::CostFunction* cost_function = ReprojectionError::create(x_, y_, intrinsics_);
            
            problem.AddResidualBlock(cost_function, NULL, &extrinsics_(0), &pt_(0));

        }
    }

    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.use_inner_iterations = true;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    //std::cout << "After BA" << std::endl;

    for(int i = se_; i <= en_; i++) {

        if(views_[i] == nullptr) {continue;}
        std::shared_ptr<View> view_(views_[i]);

        cv::Matx44d ext_mat_ = unpackCameraExtrinsics(view_->cam_extrinsics_);
        Vis::updateBAPose(cv::Mat(ext_mat_));
        
        
    }
}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(mFrames_);
    io_->getGTPoses(gt_poses_);
    
    P_prev_ = io_->getP0();
    K_  = io_->getK(); 

    cv::Matx41d t_ = io_->gett0();
    t_prev_ = cv::Matx31d(t_(0,0), t_(1, 0), t_(2,0));
    
    R_prev_ = io_->getR0();

}
