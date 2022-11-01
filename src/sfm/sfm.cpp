/*

Match2D3D simple_sfm::SimpleSFM::get2D3DMatches(const int view_idx_){

    std::cout << "Inside get2D3d matches! " << std::endl;

    std::cout << "mPointcloud.size(): " << mPointCloud_.size() << std::endl;

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
                
                matching_feature_found_ = ((curr_view_idx_ > pt_view_idx_)  ? (match.queryIdx == feature_idx_) : (match.trainIdx == feature_idx_));

                if(matching_feature_found_) {
                    
                    //int match_idx_ = ((curr_view_idx_ > pt_view_idx_) ? match.trainIdx : match.queryIdx);
                    int match_idx_ = ((curr_view_idx_ > pt_view_idx_) ? match.trainIdx : match.queryIdx);
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
    
    //sstd::cout << "inliers_ratio_: " << inlier_ratio_ << std::endl;

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

    

    //std::cout << "Trying to visualize camera_pose!" << std::endl;
    
    
    //std::cout << "camera_pose_: " << camera_pose_ << std::endl;
    camera_cnt_++;

    std::cout << "camera_cnt_: " << camera_cnt_ << std::endl;
    
    Vis::updatePredictedPose(cv::Mat(camera_pose_));
    
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
                
                //std::cout << "norm_ < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE" << std::endl; 

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

                            if( match_.queryIdx == image_pair_.first && 
                                match_.trainIdx == image_pair_.second ) {
                                
                                std::cout << "POINTCLOUD MATCH FOUND!" << std::endl;
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

    std::cout << "merge_cnt_: " << merge_cnt_ << std::endl;
    std::cout << "new_pts_added_cnt_: " << new_pts_cnt_ << std::endl;


    
}

void simple_sfm::SimpleSFM::addMoreViewsToReconstruction(){

    std::cout << "Inside addMoreViewsToReconstruction!" << std::endl;

    //std::cout << "mFrames.size(): " << mFrames_.size() << std::endl;
    //std::cout << "mDoneViews_.size(): " << mDoneViews_.size() << std::endl;

    for(int i = 0; i < mFrames_.size(); i++ ) {
        
        std::cout << "i: " << i << std::endl;
        if(mDoneViews_.size() == mFrames_.size()) {

            //std::cout << "mDoneViews.size() == mFrames.size()!" << std::endl;
            break;

        }

        int best_view_idx_;
        bool success_ = getBestViewIndexToMerge(best_view_idx_);
        
        //std::cout << "success_: " << success_ << std::endl;
        
        mDoneViews_.insert(best_view_idx_);

        if(!success_) {

            continue;
        }
        

        Match2D3D match2d3d_ = get2D3DMatches(best_view_idx_);
        
        

        assert(match2d3d_.pts_2d_.size() == match2d3d_.pts_3d_.size());

        //std::cout << "match2d3d_.pts_2d_.size: " << match2d3d_.pts_2d_.size() << std::endl;

        cv::Matx34d camera_pose_;
        
       //std::cout << "calling updateCameraPoseFrom2d3dMatch function!" << std::endl << std::endl;
        
        success_  = updateCameraPoseFrom2D3DMatch(camera_pose_, match2d3d_);  
        
        
        if(!success_) {

            continue;

        }

        std::cout << std::endl;
        //std::cout << "after updateCameraPoseFrom2d3dMatch function!" << std::endl;   

        mCameraPoses_[best_view_idx_] = camera_pose_;

        std::cout << "camera pose for " << best_view_idx_ << " frame: " << camera_pose_ << std::endl;

        std::cout << "mGoodViews_.size(): " << mGoodViews_.size() << std::endl; 

        for(const int &good_view_ : mGoodViews_){

            //CloudPoint3d cloud_pt_ = mPointCloud_[good_view_];

            //for(const auto &view_: cloud_pt_.viewMap){

                //int view_idx_ = view_.first; 
                int view_idx_ = good_view_; 
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

            //}

        }


        mGoodViews_.insert(best_view_idx_);
    }

}






void simple_sfm::SimpleSFM::runSFMPipeline() {

    //std::cout << "Inside runSFMPipeline!" << std::endl;
    mCameraPoses_.resize(mFrames_.size());
    
    createFeatureMatrix();

    //createFeatureMatchMatrix();
    createFeatureMatchMatrixThreaded();

    initializeBaselineSFM();
        
    addMoreViewsToReconstruction();

}



*/