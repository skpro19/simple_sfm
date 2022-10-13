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

    //std::cout << "Inside createFeatureMatrix!" << std::endl;

    int num_frames_ = mFrames_.size();

    mFeatures_.resize(num_frames_);


    for(int i = 0 ;i < num_frames_; i++) {

        cv::Mat img_ = cv::imread(mFrames_[i].c_str());
        
        Features features_;
        Frame::extractFeaturesAndDescriptors(img_, features_);
        Frame::keypointsToPoints(features_);        
        mFeatures_[i] = features_;
        
        //std::cout << "mFeatures_[i].keypoints.size(): " << mFeatures_[i].keypoints.size() << std::endl;
        
    }

    //for(auto t : mFeatures_) std::cout << t.keypoints.size() << std::endl;

    //std::cout << "DONE!" << std::endl;


}   

void simple_sfm::SimpleSFM::createFeatureMatchMatrix(){

    //std::cout << "Inside createFeatureMatchMatrix!" << std::endl;
    int n_ = mFrames_.size(); 

    mMFeatureMatches_.resize(n_);

    for(int i = 0 ; i < n_ ; i++) {
        
        std::vector<Matches> v_(n_);

        for(int j = i + 1; j < n_ ; j++) {

            Matches matches_ = Frame::getMatches(mFeatures_[i], mFeatures_[j]);

            std::cout << "(" << i << "," << j << ") ---> " << matches_.size() << std::endl;

            //v_.push_back(matches_);
            v_[j] = matches_;

        }

        //mMFeatureMatches_.push_back(v_);
        mMFeatureMatches_[i] = v_;
    
    }
    
    for(auto &vec: mMFeatureMatches_){

        for(auto mat: vec) {
            
            //std::cout << mat << " ";
            std::cout << mat.size() << " ";
        }

        std::cout << std::endl;

    }
    
    std::cout << "DONE!" << std::endl;

}

std::map<float, ImagePair> simple_sfm::SimpleSFM::sortViewsByHomography(){

    //std::cout << "Inside sortViewsByHomography!" << std::endl;

    std::map<float, ImagePair> homography_ratio_map_;
    const int n_ = (int)mFrames_.size(); 


    for(int i = 0 ; i < n_ ; i++) {

        for(int j = i + 1; j < n_; j++) {
            
            //std::cout << "(i,j): (" << i << "," << j << ")" << std::endl;

            int match_sz_ = (int)mMFeatureMatches_[i][j].size();
            
            //std::cout << "match_sz_: " << match_sz_  << std::endl;

            if(match_sz_ < MIN_POINT_COUNT_FOR_HOMOGRAPHY) {

                homography_ratio_map_[1.0] = ImagePair{i,j};
                continue;
            }

            //std::cout << "HI" << std::endl;
            
            
            const int inliers_cnt_ = Frame::getHomographyInliersCount(mFeatures_[i], mFeatures_[j], mMFeatureMatches_[i][j]);

            //std::cout << "inliers_cnt_: " << inliers_cnt_ << std::endl;

            const double inlier_ratio_ = 1.0 * (double)inliers_cnt_/ (double)mMFeatureMatches_[i][j].size(); 

            //std::cout << "inlier_ratio_: " << inlier_ratio_ << std::endl;

            homography_ratio_map_[inlier_ratio_] = ImagePair{i,j};

        }

    }
    
    return homography_ratio_map_;

}

bool simple_sfm::SimpleSFM::findCameraMatrices(cv::Matx34d &P1_ , cv::Matx34d &P2_, const ImagePair &img_pair_, Matches &pruned_matches_){

    //std::cout << "Inside findCameraMatrices! " << std::endl;

    assert(img_pair_.first < img_pair_.second);

    cv::Mat E_, R, t, inlier_mask_;

    const Features &f1_ = mFeatures_[img_pair_.first]; 
    const Features &f2_ = mFeatures_[img_pair_.second]; 
    const Matches &matches_ = mMFeatureMatches_[img_pair_.first][img_pair_.second];

    Features f1_mat_, f2_mat_; 
    std::vector<int> ref_f1_, ref_f2_;

    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_, ref_f1_, ref_f2_, matches_);
    
    const Points2d &pts1_ = f1_mat_.points;
    const Points2d &pts2_ = f2_mat_.points;


    //std::cout << "pts1_.size(): " << pts1_.size() << " pts2_.size(): " << pts2_.size() << std::endl;
    
    E_ = cv::findEssentialMat(pts1_, pts2_, K_,cv::RANSAC, 0.999, 1.0, inlier_mask_);
    
    //std::cout << "E_: " << E_ << std::endl;

    const int inlier_cnt_ = cv::recoverPose(E_, pts1_, pts2_, K_, R, t, inlier_mask_);

    //std::cout << "inlier_cnt_: " << inlier_cnt_ << std::endl;

    if(inlier_cnt_ < POSE_INLIERS_MINIMAL_COUNT) { return false;}

    P1_ = cv::Matx34d::eye();

    P2_ = cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    pruned_matches_.resize(0);

    //const Matches &matches_ = mMFeatureMatches_[img_pair_.first][img_pair_.second];
        
    for(int i = 0 ; i < inlier_mask_.rows; i++) {

        if(inlier_mask_.at<uchar>(i)){

            pruned_matches_.push_back(matches_[i]);

        }
    }

    return true;
   
}

bool simple_sfm::SimpleSFM::getBestViewIndexToMerge(int &idx_){

    const int n_ = (int)mFrames_.size(); 

    std::cout << "Inside getBestViewIndexToMerge!" << std::endl;
    
    std::cout << "n_: " << n_ << std::endl;

    int best_frame_idx_ = -1; 
    int mx_match_cnt_ = -1;
    
    for(int i = 0 ; i < n_; i++) {
        
        if(mDoneViews_.find(i) != mDoneViews_.end()) {continue;}

        const int curr_view_idx_ = i; 

        int match_cnt_ = 0 ; 
    
        for(const auto &cloud_pt_: mPointCloud_){

            bool matching_feature_found_ = false;

            const cv::Point3d &pt_3d_ = cloud_pt_.point_;
            const std::map<int, int> &view_map_ = cloud_pt_.viewMap; 

            for(const auto &item_: view_map_){
                
                const int &pt_view_idx_ = item_.first; 
                const int &feature_idx_ = item_.second;

                const int &left_idx_ = (pt_view_idx_ > curr_view_idx_ ? curr_view_idx_: pt_view_idx_);
                const int &right_idx_ = (pt_view_idx_ > curr_view_idx_ ? pt_view_idx_: curr_view_idx_);
                
                assert(left_idx_ < right_idx_);

                const Matches &matches_ = mMFeatureMatches_[left_idx_][right_idx_];

                for(const auto &match: matches_){
                    
                    //matching_feature_found_ = ((curr_view_idx_ > pt_view_idx_)  ? (match.trainIdx == pt_view_idx_) : (match.queryIdx == pt_view_idx_));
                    matching_feature_found_ = ((curr_view_idx_ > pt_view_idx_)  ? (match.trainIdx == feature_idx_) : (match.queryIdx == feature_idx_));

                    if(matching_feature_found_) {
                        
                        //std::cout << "match found!" << std::endl;
                        match_cnt_++;
                        break;

                    }
                }

                //if(matching_feature_found_) {break;}
            
            }
        }

        if(match_cnt_ > mx_match_cnt_) {

            mx_match_cnt_ = match_cnt_; 
            best_frame_idx_ = curr_view_idx_;

        }

        std::cout << "i: " << i << " match_cnt_: " << match_cnt_ << std::endl;
    
    }



    if(best_frame_idx_ != -1) {

        idx_ = best_frame_idx_ ;
        return true;
    }

    return false;
}

Match2D3D simple_sfm::SimpleSFM::get2D3DMatches(const int view_idx_){

    std::cout << "Inside get2D3DMatches function!" << std::endl;

    Match2D3D match_2d3d_;

    const int curr_view_idx_ = view_idx_;

    for(const auto &cloud_pt_: mPointCloud_){

        bool matching_feature_found_ = false;

        const cv::Point3d &pt_3d_ = cloud_pt_.point_;
        const std::map<int, int> &view_map_ = cloud_pt_.viewMap; 

        for(const auto &item_: view_map_){
            
            const int &pt_view_idx_ = item_.first; 
            const int &feature_idx_ = item_.second;

            const int &left_idx_ = (pt_view_idx_ > curr_view_idx_ ? curr_view_idx_: pt_view_idx_);
            const int &right_idx_ = (pt_view_idx_ > curr_view_idx_ ? pt_view_idx_: curr_view_idx_);
            
            assert(left_idx_ < right_idx_);

            const Matches &matches_ = mMFeatureMatches_[left_idx_][right_idx_];

            for(const auto &match: matches_){
                
                matching_feature_found_ = ((curr_view_idx_ > pt_view_idx_)  ? (match.trainIdx == feature_idx_) : (match.queryIdx == feature_idx_));

                if(matching_feature_found_) {
                    
                    int match_idx_ = ((curr_view_idx_ > pt_view_idx_) ? match.queryIdx : match.trainIdx);
                    match_2d3d_.pts_2d_.push_back(mFeatures_[view_idx_].points[match_idx_]);
                    match_2d3d_.pts_3d_.push_back(cloud_pt_.point_);
                    break;
                
                }
            }

            if(matching_feature_found_) {break;}
        
        }
    }

    std::cout << "match_2d3d.size(): " << match_2d3d_.pts_2d_.size() << std::endl;

    return match_2d3d_;
}

bool simple_sfm::SimpleSFM::updateCameraPoseFrom2D3DMatch(cv::Matx34d &camera_pose_, const Match2D3D &match2d3d_){

    bool success_ = false;

    cv::Mat rvec_, tvec_; 
    cv::Mat inliers_;
    
    success_ = cv::solvePnPRansac(match2d3d_.pts_3d_, match2d3d_.pts_2d_, K_, cv::Mat(), rvec_, tvec_,false,100,8.0, 0.99, inliers_);

    if(!success_) {return success_; }

    assert(inliers_.rows != match2d3d_.pts_2d_.size());

    const double inlier_ratio_ = (1.0) * ((double)inliers_.rows/ (double)match2d3d_.pts_2d_.size());

    if(inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO) {

        //std::cout << "inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO" << std::endl;;
        return false;

    }

    cv::Mat R_;
    cv::Rodrigues(rvec_, R_);

    const cv::Rect &ROT_    =       cv::Rect(0, 0, 3, 3);
    const cv::Rect &TRA_    =       cv::Rect(3, 0, 1, 3);

    
    R_.copyTo(cv::Mat(3, 4, CV_64FC1, camera_pose_.val)(ROT_));
    tvec_.copyTo(cv::Mat(3, 4, CV_64FC1, camera_pose_.val)(TRA_));
    
    return true;
    
}

void simple_sfm::SimpleSFM::mergeNewPointCloud(std::vector<CloudPoint3d> &pointcloud_){

    int merge_cnt_ = 0 ;
    int new_pts_cnt_  = 0 ;

    for(const CloudPoint3d &new_pt_ : pointcloud_){

        bool matching_3d_pt_found_ = false;
        bool any_matching_view_pt_found_ = false;

        for(auto &existing_pt_: mPointCloud_){

            const double norm_ = cv::norm(new_pt_.point_ - existing_pt_.point_);

            if(norm_ < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {

                matching_3d_pt_found_ = true;

                for(const auto &new_view_: new_pt_.viewMap){
                    
                    const int new_view_idx_ = new_view_.first; 
                    const int new_feature_idx_ = new_view_.second;
                    
                    for(auto &existing_view_: existing_pt_.viewMap){
                        
                        bool matching_view_pt_found_ = false;

                        const int existing_view_idx_ = existing_view_.first; 
                        const int existing_feature_idx_=  existing_view_.second;

                        const ImagePair image_pair_ = ((existing_view_idx_ > new_view_idx_) ? ImagePair{new_view_idx_, existing_view_idx_}: ImagePair{existing_view_idx_, new_view_idx_});

                        const Matches &matches_ =  mMFeatureMatches_[image_pair_.first][image_pair_.second];

                        for(const auto &match_ : matches_) {

                            if( match_.trainIdx == image_pair_.first && 
                                match_.queryIdx == image_pair_.second && 
                                match_.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE) {
                                
                                matching_view_pt_found_ = true; 
                                break;
                                
                            }
                        }

                        if(matching_view_pt_found_) {
                            
                            any_matching_view_pt_found_ = true;
                            existing_pt_.viewMap[new_view_idx_] = new_feature_idx_;
                        
                        }
                    }
                }


                

            }

            if(any_matching_view_pt_found_){

                merge_cnt_++;
                break;

            }
        }

        if(not matching_3d_pt_found_ and not any_matching_view_pt_found_) {

            mPointCloud_.push_back(new_pt_);
            new_pts_cnt_++;

        }
    }
    
}

void simple_sfm::SimpleSFM::addMoreViewsToReconstruction(){

    std::cout << "Inside addMoreViewsToReconstruction!" << std::endl;

    std::cout << "mFrames.size(): " << mFrames_.size() << std::endl;
    std::cout << "mDoneViews_.size(): " << mDoneViews_.size() << std::endl;

    for(int i = 0; i < mFrames_.size(); i++ ) {

        if(mDoneViews_.size() == mFrames_.size()) {

            std::cout << "mDoneViews.size() == mFrames.size()!" << std::endl;
            break;

        }

        int best_view_idx_;
        bool success_ = getBestViewIndexToMerge(best_view_idx_);
        
        std::cout << "success_: " << success_ << std::endl;
        
        mDoneViews_.insert(best_view_idx_);

        if(!success_) {

            continue;
        }
        
        std::cout << "best_view_idx_: " << best_view_idx_ << std::endl;

        Match2D3D match2d3d_ = get2D3DMatches(best_view_idx_);

        std::cout << "match2d3d_pts_2d_.size(): " << match2d3d_.pts_2d_.size() << std::endl;
        std::cout << "match2d3d_pts_3d_.size(): " << match2d3d_.pts_3d_.size() << std::endl;
        
        assert(match2d3d_.pts_2d_.size() == match2d3d_.pts_3d_.size());

        assert(match2d3d_.pts_2d_.size() > MIN_POINT_COUNT_FOR_2D3DMATCH);

        cv::Matx34d camera_pose_;

        success_  = updateCameraPoseFrom2D3DMatch(camera_pose_, match2d3d_);  

        if(!success_) {

            continue;

        }

        mCameraPoses_[best_view_idx_] = camera_pose_;

        for(const int &good_view_ : mGoodViews_){

            CloudPoint3d cloud_pt_ = mPointCloud_[good_view_];

            for(const auto &view_: cloud_pt_.viewMap){

                int view_idx_ = view_.first; 
                //int feature_idx_ = view_.second;

                cv::Matx34d P1_, P2_;

                const ImagePair img_pair_ = ( (view_idx_ > best_view_idx_) ? ImagePair{best_view_idx_, view_idx_} : ImagePair{view_idx_, best_view_idx_} ); 
                
                Matches pruned_matches_;

                bool flag_ = findCameraMatrices(P1_, P2_, img_pair_, pruned_matches_); 

                if(!flag_) {

                    continue;

                }

                mMFeatureMatches_[img_pair_.first][img_pair_.second] = pruned_matches_;

                
                std::vector<CloudPoint3d> pointcloud_;

                flag_ = triangulateViews(P1_, P2_, img_pair_, pointcloud_);

                if(!flag_) {

                    continue;

                }

                mergeNewPointCloud(pointcloud_);

            }

        }

    }

}


bool simple_sfm::SimpleSFM::triangulateViews(const cv::Matx34d &P1_, const cv::Matx34d &P2_, const ImagePair &img_pair_, std::vector<CloudPoint3d> &pointcloud_){

    //std::cout << "Inside triangulateViews!" << std::endl;

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
    
    cv::projectPoints(pts_3d_, rvec1_, tvec1_, K_, cv::Mat(), projected_pts_1_);
    cv::projectPoints(pts_3d_, rvec2_, tvec2_, K_, cv::Mat(), projected_pts_2_);

    //cv::projectPoints(pts_4d_, rvec1_, cv::Mat()_, K_, cv::Mat(), projected_pts_1_);
    //cv::projectPoints(pts_4d_, rvec2_, cv::Mat(), K_, cv::Mat(), projected_pts_2_);

    
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


    //std::cout << "Inside InitializeBaselineSFM!" << std::endl;

    const std::map<float, ImagePair> homography_ratio_map_  = sortViewsByHomography();
    
    //std::cout << "sortViewsByHomography ---- DONE!" << std::endl;

    cv::Matx34d P1_, P2_;
    std::vector<CloudPoint3d> pointcloud_;

    for(const auto &elem_: homography_ratio_map_){

        const float ratio_ = elem_.first;
        const ImagePair &img_pair_  = elem_.second;

        assert(img_pair_.first < img_pair_.second);

        const int i = img_pair_.first; 
        const int j = img_pair_.second;
        
        Matches pruned_matches_;

        bool success_ = findCameraMatrices(P1_, P2_, img_pair_, pruned_matches_);

        //std::cout << "success_: " << success_ << std::endl;

        if(not success_) {continue;}
        
        const double inlier_ratio_ = 1.0 * (double)pruned_matches_.size() / (double)mMFeatureMatches_[img_pair_.first][img_pair_.second].size();

        //std::cout << "inlier_ratio_: " << inlier_ratio_ << std::endl;

        if (inlier_ratio_ < POSE_INLIERS_MINIMAL_RATIO) {

            success_ = false;
            continue;
        }

        mMFeatureMatches_[img_pair_.first][img_pair_.second] = pruned_matches_;

        success_ = triangulateViews(P1_, P2_, img_pair_, pointcloud_);

        //std::cout << "success_ after triangulateViews: " << success_ << std::endl;

        if(not success_) {continue;}

        mCameraPoses_.resize(mFrames_.size());
        
        mCameraPoses_[i] = P1_; 
        mCameraPoses_[j] = P2_;
        



        mDoneViews_.insert(i);        
        std::cout << "mDoneViews.size(): " << (int)mDoneViews_.size() << std::endl;
        
        mDoneViews_.insert(j);
        std::cout << "mDoneViews.size(): "<< (int)mDoneViews_.size() << std::endl;
        
        mGoodViews_.insert(i);
        mGoodViews_.insert(j);

        mPointCloud_ = pointcloud_;

        break;

    }

    std::cout << "DONE!" << std::endl;
}

void simple_sfm::SimpleSFM::runSFMPipeline() {

    //std::cout << "Inside runSFMPipeline!" << std::endl;

    createFeatureMatrix();

    createFeatureMatchMatrix();
    
    initializeBaselineSFM();
        
    addMoreViewsToReconstruction();

}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(mFrames_);

    while(mFrames_.size() > 5) {

        mFrames_.pop_back();

    }

    std::cout << "mFrames.size(): " << mFrames_.size() << std::endl;
    
    io_->getGTPoses(gt_poses_);
    
    P_prev_ = io_->getP0();
    K_  = io_->getK(); 

    cv::Matx41d t_ = io_->gett0();
    t_prev_ = cv::Matx31d(t_(0,0), t_(1, 0), t_(2,0));
    
    R_prev_ = io_->getR0();

}
