#include "../../include/sfm/sfm.hpp"
#include "../../include/sfm/sfm_utility.hpp"

#include <opencv2/calib3d.hpp>


simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_ = std::make_shared<SFM_IO>(base_folder_);
    bkp_ = std::make_shared<BookKeeping>();


    updateIOParams();
    
}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(image_file_list_);
    io_->getGTPoses(gt_poses_);
    
    P_prev_ = io_->getP0();
    K_  = io_->getK(); 

    cv::Matx41d t_ = io_->gett0();
    t_prev_ = cv::Matx31d(t_(0,0), t_(1, 0), t_(2,0));
    
    R_prev_ = io_->getR0();

}

void simple_sfm::SimpleSFM::runSFMPipeline(){

    initializeSFMPipeline();

    int num_frames_ = (int)image_file_list_.size() ; 

    int frame_idx_ = 2; 

    while(frame_idx_ < num_frames_) {

        addNextFrame(frame_idx_); 
        frame_idx_++;

    }

}


void simple_sfm::SimpleSFM::initializeSFMPipeline() 
{
    std::cout << "[sfm] InitializeSFMPipeline function!" << std::endl;

    bool initialized_ = false; 

    cv::Mat E_, E_mask_;
    cv:: Mat R, t;
    
    Points2D pts_curr_, pts_last_;

    int last_idx_ = 0 ;
    int curr_idx_ = 1;
    int inlier_cnt_ = 0 ;
    
    double scale_; 
        
    std::cout << "[sfm] curr_idx_: " << curr_idx_ << std::endl;

    pts_curr_.resize(0);
    pts_last_.resize(0);

    F0_ = image_file_list_[last_idx_];
    F1_ = image_file_list_[curr_idx_];

    Frame::Points2DFromFrames(F0_, F1_, pts_last_, pts_curr_);   
    
    E_ = cv::findEssentialMat(pts_curr_, pts_last_, K_, cv::RANSAC, 0.999, 1.0, E_mask_);
    
    inlier_cnt_ = cv::recoverPose(E_, pts_curr_, pts_last_, K_, R, t, E_mask_);

    scale_ = Frame::GetAbsoluteScale(gt_poses_[last_idx_], gt_poses_[curr_idx_]);

    curr_idx_++; 

    cv::Matx34d P0_, P1_;
    cv::Matx34d C0_, C1_;

    P0_ = P_prev_;

    C0_ =  {    
                R_prev_(0, 0),  R_prev_(0, 1),  R_prev_(0,2) , t_prev_(0,0),
                R_prev_(1, 0),  R_prev_(1, 1),  R_prev_(1, 2), t_prev_(1,0),
                R_prev_(2, 0),  R_prev_(2, 1),  R_prev_(2, 2), t_prev_(2,0)
            
            };

    assert(("[sfm]" , t.size() == cv::Size(1, 3)));
    assert(("[sfm]" , R.size() == cv::Size(3, 3)));

    std::cout << "[sfm] H1" << std::endl;

    cv::Matx44d T_k_ = {
                            R.at<double>(0, 0),  R.at<double>(0, 1),  R.at<double>(0, 2), t.at<double>(0, 0),
                            R.at<double>(1, 0),  R.at<double>(1, 1),  R.at<double>(1, 2), t.at<double>(1, 0),
                            R.at<double>(2, 0),  R.at<double>(2, 1),  R.at<double>(2, 2), t.at<double>(2, 0),
                            0 , 0 , 0 ,1

                        };
    

    C1_ = C0_ * T_k_;

    P1_ = K_ * C1_;

    std::cout << "[sfm] H2" << std::endl;


    cv::Mat pts_4d_;
    cv::triangulatePoints(P0_, P1_, pts_last_, pts_curr_, pts_4d_);
    
    std::vector<Point3D> pts_3d_;
    convertPointsFromHomogeneous(pts_4d_, pts_3d_);

    bkp_->initializeGlobalPointCloud(pts_3d_);
    
    std::cout << "[sfm] H3 " << std::endl;

    bkp_->initialize2D3DCorrespondance(pts_curr_, pts_3d_);
    std::cout << "[sfm] H4" << std::endl;


    P_prev_ = P1_;
    C_prev_ = C1_;


    std::cout << "[sfm] H5" << std::endl;

}



void simple_sfm::SimpleSFM::addNextFrame(int frame_idx_) {
    
    std::cout << "[sfm] ---> addNextFrame ---> frame_idx_: " << frame_idx_ << std::endl;
    assert(("[sfm]" , frame_idx_ > 0));

    int last_idx_ = frame_idx_ - 1; 

    cv::String last_frame_ = image_file_list_[last_idx_]; 
    cv::String curr_frame_ = image_file_list_[frame_idx_];

    Points2D last_pts_, curr_pts_;    
    Frame::Points2DFromFrames(last_frame_, curr_frame_, last_pts_, curr_pts_);

    assert(("[sfm]" , (int)last_pts_.size() == (int)curr_pts_.size()));

    Points3D object_points_;
    Points2D image_points_;

    bkp_->addNextFrame(last_pts_, curr_pts_, image_points_, object_points_);

    assert(("[sfm]" , (int)image_points_.size() == (int)object_points_.size()));

    cv::Mat rvec_, tvec_; 

    bool flag_ = cv::solvePnPRansac(object_points_, image_points_, K_,cv::Mat(), rvec_, tvec_);

    assert(("[sfm]" , flag_));

    //std::cout << "[sfm] rvec_: " << rvec_ << std::endl; 
    //std::cout << "[sfm] tvec_: " << tvec_ << std::endl; 
    
    //std::cout << "[sfm] rvec_.size(): " << rvec_.size() << std::endl; 
    //std::cout << "[sfm] tvec_.size(): " << tvec_.size() << std::endl; 
    
    cv::Mat R, t(tvec_); 
    cv::Rodrigues(rvec_, R); 
    //cv::Rodrigues(tvec_, t); 

    ///std::cout << "[sfm] R.size(): " << R.size() << std::endl;
    //std::cout << "[sfm] t.size(): " << t.size() << std::endl;

    //std::cout << "[sfm] R: " << R << std::endl;
    //std::cout << "[sfm] t: " << t << std::endl;

    assert(("[sfm]" , R.size() == cv::Size(3, 3)));
    assert(("[sfm]" , t.size() == cv::Size(1, 3)));
    

    cv::Matx34d C_;

    //computes the transform between C_prev_ and C_;
    cv::Matx44d T_k_ = {
                            R.at<double>(0, 0),  R.at<double>(0, 1),  R.at<double>(0,2) , t.at<double>(0,0),
                            R.at<double>(1, 0),  R.at<double>(1, 1),  R.at<double>(1, 2), t.at<double>(1,0),
                            R.at<double>(2, 0),  R.at<double>(2, 1),  R.at<double>(2, 2), t.at<double>(2,0),
                            0 , 0 , 0 ,1

                        };
    
    C_ = C_prev_ * T_k_;

    cv::Matx34d P0_, P1_;
    
    P0_ = P_prev_;

    P1_ = K_ * C_ ;

    cv::Mat pts_4d_;
    cv::triangulatePoints(P0_, P1_, last_pts_, curr_pts_, pts_4d_);  

    
    std::vector<Point3D> pts_3d_; 
    convertPointsFromHomogeneous(pts_4d_, pts_3d_);
    
    bkp_->updateGlobalPointCloud(curr_pts_, pts_3d_);

    P_prev_ = P1_; 
    C_prev_ = C_;

}


