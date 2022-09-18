#include "../../include/sfm/sfm.hpp"



bool simple_sfm::SimpleSFM::checkForDuplicates(const std::vector<cv::Point2f> &a_  , const std::vector<cv::Point2f> &b_){

    std::set<std::pair<float, float> > sa_, sb_; 

    assert((int)a_.size() == (int)b_.size());
    
    int n_ = (int)a_.size() ;

    for(int i = 0 ;i < n_;  i++) {

        std::pair<float, float> pa_, pb_; 
        
        //pa_ = {a_[i].pt.x, a_[i].pt.y},  pb_ = {b_[i].pt.x, b_[i].pt.y}; 
        pa_ = {a_[i].x, a_[i].y},  pb_ = {b_[i].x, b_[i].y}; 
        
        //if(sa_.count(pa_) > 0 || sb_.count(pb_) > 0) {continue;}

        sa_.insert(pa_);
        sb_.insert(pb_);
    
    }

    return( ( (int)sa_.size() == a_.size() ) && ( (int)sb_.size() == (int)b_.size()));

    //std::cout << "a_.size(): " << a_.size() << " sa_.size(): " << (int)sa_.size() << std::endl;
    //std::cout << "b_.size(): " << b_.size() << " sb_.size(): " << (int)sb_.size() << std::endl;

}



/*void simple_sfm::SimpleSFM::runVOPipeline(){

    std::cout << "[sfm] runVOPipeline" << std::endl;

    std::cout << "[sfm] R0_: " << R0_ << std::endl;
    std::cout << "[sfm] t0_: " << t0_ << std::endl;

    int sz_ = (int)image_file_list_.size(); 

    double scale_;

    int curr_idx_ = 1, last_idx_= 0 ; 

    cv::Mat E_, E_mask_; 
    cv::Matx33d R;
    cv::Matx31d t; 
    
    cv::Mat T_k_;   //transform between c_k and c_k_minus_1
    cv::Mat C0_, C1_;



    
    //cv::Matx13d a_ = {0 ,0 , 0};
    cv::Matx14d b_ = {0, 0 , 0, 1};

    cv::Mat predictions_mat_ = cv::Mat::zeros(800, 500, CV_8UC3);
    cv::Mat gt_mat_ = cv::Mat::zeros(800, 500, CV_8UC3);

    //cv::vconcat(R0_, a_, C0_);
    //cv::hconcat(C0_, t0_, C0_);

    
    Points2D pts_last_, pts_curr_;

    int inlier_cnt_ = 0 ;

    C0_ = (cv::Mat_<double>(4, 4) << 1, 0 , 0 ,0 , 0 , 1, 0 , 0 , 0 , 0 ,1, 0, 0 , 0 , 0 ,1);
    std::cout << "[sfm] C0_: " << C0_ << std::endl;


    while(curr_idx_ < sz_ - 1) {

        F0_ = image_file_list_[last_idx_]; 
        F1_ = image_file_list_[curr_idx_];

        pts_last_.resize(0);
        pts_curr_.resize(0);

        Frame::Points2DFromFrames(F0_, F1_, pts_last_, pts_curr_);    
        
        //std::cout << "[sfm] H1" << std::endl;
    
        E_ = cv::findEssentialMat(pts_curr_, pts_last_, K_, cv::RANSAC, 0.999, 1.0, E_mask_);

        //std::cout << "[sfm] H2" << std::endl;

        inlier_cnt_ = cv::recoverPose(E_, pts_curr_, pts_last_, K_, R, t, E_mask_);
        
        scale_ = Frame::GetAbsoluteScale(gt_poses_[last_idx_], gt_poses_[curr_idx_]);

        double del_z_ = std::fabs(t(2,0)); 
        double del_y_ = std::fabs(t(1,0));
        double del_x_ = std::fabs(t(0,0));


        // checks for valid t        
        bool flag_ = ((scale_ > 0.1) &&  (del_z_ > del_x_) && (del_z_ > del_y_) && inlier_cnt_ > 25)  ; 


        
        std::cout << "curr_idx_: " << curr_idx_ << " flag_: " << flag_ << std::endl;

        if(!flag_) {

            curr_idx_++;    
            continue; ;
            
        }


        cv::hconcat(R, t, T_k_);
        cv::vconcat(T_k_, b_, T_k_);


        C1_ = C0_ * T_k_;
        C0_ = C1_;

        double px_ = C1_.at<double>(0, 3) + 200; 
        double py_ = C1_.at<double>(2, 3) + 100;
    
        double gx_ = gt_poses_[curr_idx_](0, 3) + 200; 
        double gy_ = gt_poses_[curr_idx_](2, 3) + 100;


        //std::cout << "[sfm] (px, py) -->(" << px_ << "," << py_ << ")" << std::endl;
        //std::cout << "[sfm] (gx, gy) -->(" << gx_ << "," << gy_ << ")" << std::endl;
        
        cv::circle(predictions_mat_,cv::Point(px_, py_), 1, CV_RGB(255, 0, 0), 2);
        cv::circle(gt_mat_,cv::Point(gx_, gy_), 1, CV_RGB(255, 0, 0), 2);
        
        cv::imshow("Trajectory", predictions_mat_);
        //cv::imshow("GT", gt_mat_);
        
        cv::waitKey(10);
        
        last_idx_ = curr_idx_;
        curr_idx_++; 

    }

}*/
