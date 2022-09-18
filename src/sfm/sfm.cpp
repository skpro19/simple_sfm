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

void simple_sfm::SimpleSFM::runVOPipeline(){

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



        //cv::imshow("Road facing camera", img_1);

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
        
        cv::imshow("img_1" , img_1);
        Vis::drawKeyPoints(img_1, kp_1f, kp_2f);

        Vis::updateGroundPose(cv::Mat(gt_poses_[i]));
        Vis::updatePredictedPose(C_k_);

        cv::waitKey(10);
    }
}

