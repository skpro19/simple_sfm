#include "../../include/sfm/sfm.hpp"

#include <opencv2/calib3d.hpp>


simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
    bkp_    =   std::make_shared<BookKeeping>();
  
    updateIOParams();
    
}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(frame_list_);
    io_->getGTPoses(gt_poses_);
    
    P_prev_ = io_->getP0();
    K_  = io_->getK(); 

    cv::Matx41d t_ = io_->gett0();
    t_prev_ = cv::Matx31d(t_(0,0), t_(1, 0), t_(2,0));
    
    R_prev_ = io_->getR0();

}



/*void simple_sfm::SimpleSFM::extract_features(const cv::Mat &img_1, const cv::Mat &img_2){

    cv::Mat image_one, image_two;

    cv::cvtColor(img_1, image_one, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_2, image_two, cv::COLOR_BGR2GRAY);

    std::vector< cv::Point2f > corners_one, corners_two;
    
    int maxCorners = 2000;

    double qualityLevel = 0.01;

    double minDistance = 1.0;

    cv::Mat mask = cv::Mat();
    
    int blockSize = 1;

    bool useHarrisDetector = false;

    double k = 0.04;

    
    //*** keypoints extraction
    cv::goodFeaturesToTrack( image_one, corners_one, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

    cv::goodFeaturesToTrack( image_two, corners_two, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

    //std::cout << "corners_one.size(): " << (int)corners_one.size() << std::endl;
    //std::cout << "corners_two.size(): " << (int)corners_two.size() << std::endl;
    

    cv::KeyPoint::convert(kp_1, corners_one, std::vector<int>());
    cv::KeyPoint::convert(kp_2, corners_two, std::vector<int>());

    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
    //std::cout << "kp_2.size(): " << (int)kp_2.size() << std::endl;
    

}

void simple_sfm::SimpleSFM::match_features(const cv::Mat &img_1, const cv::Mat &img_2){
    
    kp_1_matched.clear(); 
    kp_2_matched.clear();

    cv::Mat image_one, image_two;
    cv::cvtColor(img_1, image_one, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_2, image_two, cv::COLOR_BGR2GRAY);
    
    cv::Mat mask = cv::Mat();
    cv::Mat des_1, des_2;
    
    cv::Ptr<cv::ORB>orb_ = cv::ORB::create(5000);


    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
    
    //*** extracting descriptors from keypoints
    orb_->detectAndCompute(image_one, mask, kp_1, des_1);
    orb_->detectAndCompute(image_two, mask, kp_2, des_2);
    
    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
        
    des_1.convertTo(des_1, 0);
    des_2.convertTo(des_2, 0);
    
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::vector<cv::DMatch> brute_hamming_matches;
    matcher->match(des_1, des_2, brute_hamming_matches);

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < des_1. rows; i++ )
    {
        double dist = brute_hamming_matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    std::vector<cv::DMatch> good_matches;
    
    for ( int i = 0; i < des_1.rows; i++ )
    {
        if ( brute_hamming_matches[i].distance <= std::max( 2*min_dist, 20.0 ) )
        {
            good_matches.push_back (brute_hamming_matches[i]);
        }
    }

    for (auto match : good_matches) {
        
        kp_1_matched.push_back(kp_1[match.queryIdx]);
        kp_2_matched.push_back(kp_2[match.trainIdx]);

    }
    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
    //std::cout << "kp_1_matched.size(): " << (int)kp_1_matched.size() << std::endl;
        
}

double simple_sfm::SimpleSFM::getScale(int curr_idx_, int prev_idx_) {

    cv::Mat prev_poses_ = cv::Mat(gt_poses_[prev_idx_]);
    cv::Mat curr_poses_ = cv::Mat(gt_poses_[curr_idx_]); 

    cv::Point3d prev_point_ = {prev_poses_.at<double>(0,3), prev_poses_.at<double>(1,3), prev_poses_.at<double>(2,3)};
    cv::Point3d curr_point_ = {curr_poses_.at<double>(0,3), curr_poses_.at<double>(1,3), curr_poses_.at<double>(2,3)};
    cv::Point3d diff_ = (curr_point_ - prev_point_);

    double scale_ = cv::norm(diff_);
    
    return scale_;

}*/

void simple_sfm::SimpleSFM::run_vo_pipeline(){

    cv::Mat E_, R, t;
    cv::Mat R_f, t_f;

    cv::Mat C_k_, C_k_minus_1_;
    cv::Mat T_k_;

    R_f.convertTo(R_f, CV_64F);
    t_f.convertTo(t_f, CV_64F);

    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);    

    C_k_.convertTo(C_k_, CV_64F);
    C_k_minus_1_.convertTo(C_k_minus_1_, CV_64F);
    
    cv::Mat NO_ROT_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat NO_T_ = cv::Mat::zeros(3, 1, CV_64F);
    
    cv::Mat TEMP_ = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1); 
    TEMP_.convertTo(TEMP_, CV_64F);
    
    
    //** Initializing C_k_minus_1 with 0 translation and 0 rotation w.r.t. the 'some' initial co-ordinate frame
    cv::hconcat(NO_ROT_, NO_T_, C_k_minus_1_);
    cv::vconcat(C_k_minus_1_, TEMP_, C_k_minus_1_);

    int last_idx_ = 0 ;

    int sz_ = frame_list_.size(); 

    for(int i = 1 ; i < sz_; i++) {

        //std::cout 
    
        Frame::kp_1.resize(0); 
        Frame::kp_2.resize(0); 

        Frame::kp_1_matched.resize(0); 
        Frame::kp_2_matched.resize(0);
    
        cv::Mat img_1 = cv::imread(frame_list_[last_idx_].c_str());
        cv::Mat img_2 = cv::imread(frame_list_[i].c_str());

        //match_features(img_1, img_2);
        Frame::extractAndMatchFeatures(img_1, img_2);
        
        assert((int)Frame::kp_1_matched.size() == (int)Frame::kp_2_matched.size());

        std::vector<cv::Point2f> kp_1f, kp_2f; //array of keypoint co-ordinates

        std::cout << "i: " << i << " kp_1_matched.size(): " << (int)Frame::kp_1_matched.size() << std::endl;

        for(int k = 0; k < (int)Frame::kp_1_matched.size(); k++) {

            cv::Point2f p1_ = Frame::kp_1_matched[k].pt, p2_ = Frame::kp_2_matched[k].pt;
            
            kp_1f.push_back(p1_); 
            kp_2f.push_back(p2_);

            cv::circle( img_1, p1_, 2, cv::viz::Color::yellow(), -1 );
            cv::line(img_1, p1_, p2_, cv::viz::Color::pink());
            cv::circle( img_1, p2_, 2, cv::viz::Color::orange_red(), -1 );
        }
        
        
        cv::Mat E_mask_;
        E_mask_.convertTo(E_mask_, CV_64F);

        E_ = cv::findEssentialMat(kp_2f, kp_1f, K_,cv::RANSAC, 0.999, 1.0, E_mask_);

        int inlier_cnt_ =0 ; 



        cv::imshow("Road facing camera", img_1);

        inlier_cnt_ = cv::recoverPose(E_, kp_2f, kp_1f, K_, R, t, E_mask_);

        std::cout << "inlier_cnt_: " << inlier_cnt_ << std::endl;

        double scale_;

        double del_z_ = std::fabs(t.at<double>(2,0)); 
        double del_y_ = std::fabs(t.at<double>(1,0));
        double del_x_ = std::fabs(t.at<double>(0,0));
        
        
        if(inlier_cnt_ < 25) {continue;}

        scale_ = Frame::getScale(cv::Mat(gt_poses_[i]), cv::Mat(gt_poses_[last_idx_])); 

        bool flag_ = ((scale_ > 0.1) &&  (del_z_ > del_x_) && (del_z_ > del_y_))  ; 
        
        std::cout << std::endl;

        if(!flag_) {continue; ;}

        last_idx_ = i;

        
        cv::Mat temp_ = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1); 
        temp_.convertTo(temp_, CV_64F);

        T_k_.convertTo(T_k_, CV_64F);

        cv::hconcat(R, t, T_k_);
        cv::vconcat(T_k_, temp_, T_k_);

        
        C_k_ =  C_k_minus_1_ * T_k_;
        C_k_minus_1_ = C_k_;

        std::cout <<  i << "--->[x y z]: " << "(" <<C_k_.at<double>(0, 3) << "," << C_k_.at<double>(1, 3) << "," << C_k_.at<double>(2,3) << ")" << std::endl; 
        //draw_trajectory_windows(C_k_, i);
        
        cv::waitKey(10);
    }
}

