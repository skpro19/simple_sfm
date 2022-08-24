#include "../../include/sfm/sfm.hpp"


simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_ = std::make_shared<SFM_IO>(base_folder_);
   /// bkp_ = std::make_shared<BookKeeping>();


    updateIOParams();
    
}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(image_file_list_);
    io_->getGTPoses(gt_poses_);
    
    P0_ = io_->getP0();
    K_  = io_->getK(); 
    t0_ = io_->gett0();
    R0_ = io_->getR0();

}

void simple_sfm::SimpleSFM::runVOPipeline(){

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

}


void simple_sfm::SimpleSFM::InitializeSFMPipeline() 
{
    //TODO- Make intialization more robust and generic
    
    std::cout << "[sfm] InitializeSFMPipeline function!" << std::endl;

    bool initialized_ = false; 

    cv::Mat E_, E_mask_;
    cv:: Mat R, t;
    
    Points2D pts_curr_, pts_last_;

    //cv::Mat predictions_mat_ = cv::Mat::zeros(800, 500, CV_64FC3);

    int last_idx_ = 0 ;
    int curr_idx_ = 1;
    int inlier_cnt_ = 0 ;
    
    double scale_; 

    while(!initialized_) {
        
        std::cout << "curr_idx_: " << curr_idx_ << std::endl;

        pts_curr_.resize(0);
        pts_last_.resize(0);

        F0_ = image_file_list_[last_idx_];
        F1_ = image_file_list_[curr_idx_];

        Frame::Points2DFromFrames(F0_, F1_, pts_last_, pts_curr_);    
        
        E_ = cv::findEssentialMat(pts_curr_, pts_last_, K_, cv::RANSAC, 0.999, 1.0, E_mask_);
        
        inlier_cnt_ = cv::recoverPose(E_, pts_curr_, pts_last_, K_, R, t, E_mask_);

        scale_ = Frame::GetAbsoluteScale(gt_poses_[last_idx_], gt_poses_[curr_idx_]);

        std::cout << "scale_: " << scale_ << std::endl;

        curr_idx_++; 

        if(inlier_cnt_ > 25 && scale_ > 0.1) {

            std::cout << "[sfm.cpp] Valid initial points pair found for sfm pipeline!" << std::endl;
            std::cout << "last_idx_: " << last_idx_  << " curr_idx_: " << curr_idx_ << std::endl;
            std::cout << "R: " << R << " t: " << t << std::endl;
            initialized_ = true;
        
        }

        else {

            std::cout << "[sfm.cpp] Invalid initial points pair -- search for next point!" << std::endl;

        }     
    }

    
    //updating P1
    assert(("[sfm]" , P1_(0,0) == 0.0));

    cv::hconcat(R, t, P1_);

    P1_ = K_ * P1_;

    std::cout << "[sfm] P0: " << P0_ << std::endl;

    std::cout << "[sfm] P1: " << P1_ << std::endl;

    //initial triangulation
    cv::Mat points4d_;
    cv::triangulatePoints(P0_, P1_, pts_last_, pts_curr_, points4d_);

    std::cout << "[sfm] pts_last_.size(): " << pts_last_.size() << std::endl;
    std::cout << "[sfm] points4d_.size(): " << points4d_.size() << std::endl;

    bkp_->updatePointCloudMap(pts_curr_, points4d_);
    
}



void simple_sfm::SimpleSFM::addNextFrame(int frame_idx_) {


    assert(("[sfm]" , frame_idx_ > 1));

    int last_idx_ = frame_idx_ - 1; 

    cv::String last_frame_ = image_file_list_[last_idx_]; 
    cv::String curr_frame_ = image_file_list_[frame_idx_];

    Points2D last_pts_, curr_pts_;    
    Frame::Points2DFromFrames(last_frame_, curr_frame_, last_pts_, curr_pts_);

    assert(("[sfm]" , (int)last_pts_.size() == (int)curr_pts_.size()));

    Points3D object_points_;
    Points2D image_points_;

    //update object_points_ and image_points_
    //bkp_->addNextFrame(last_pts_, curr_pts_, object_points_, image_points_);

    //getRandT
    //triangulateAndUpdateGlobalPointCloud
    


}


