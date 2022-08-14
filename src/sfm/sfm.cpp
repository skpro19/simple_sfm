#include "../../include/sfm.hpp"

#include <opencv2/calib3d.hpp>


simple_sfm::SimpleSFM::SimpleSFM(const std::string &folder_){

    std::cout << "Inside the SimpleFSM Constructor!" << std::endl;

    vo_ = std::make_shared<VisualOdom>(folder_);
    vis_ = std::make_shared<Vis>();

    K_ = vo_->K_;
    
    std::cout << "[sfm] K_: " << K_ << std::endl;
}


void simple_sfm::SimpleSFM::updateCameraPose(cv::Matx44d &pose_){

    vo_->update_pose(pose_);

}

void simple_sfm::SimpleSFM::initalizePipeline() {

    vo_->initialize();
    updateCameraPose(C_0);
    displayCameraPose(C_0);

    std::cout << "SFM Pipeline initialized with C_0 set to: " << C_0 << std::endl;

}

bool simple_sfm::SimpleSFM::processNextFrame(const int &curr_frame_id_){

    std::cout << "Processing frame " << curr_frame_id_ << std::endl;

    bool flag_ = vo_->process_next_frame(curr_frame_id_);

    return flag_ ;

}

void simple_sfm::SimpleSFM::displayCameraPose(const cv::Matx44d &pose_) {

    vis_->viusalizeCameraPose(pose_);

}


//formatting 2d-2d matches so as to pass it to cv::triangulatePoints function
void simple_sfm::SimpleSFM::triangulate3DPoints(std::vector<cv::Matx41d> &points3d_) {

    std::cout << "[sfm] Inside the updatePoints2D function!" << std::endl;

    std::vector<cv::Point2f> pts_0, pts_1;
    vo_->update2DMatches(pts_0 , pts_1);

    assert(pts_0.size() == pts_1.size());

    cv::Matx33d R_0, R_1; 
    cv::Matx31d t_0, t_1;
    cv::Matx34d A_0, A_1; //  A= [R|t]
    cv::Matx34d P_0, P_1;

    std::cout << "[sfm] C_0: " << C_0 << std::endl;
    std::cout << "[sfm] C_1: " << C_1 << std::endl;

    //extracting R and t from C
    R_0 = C_0.get_minor<3, 3>(0, 0);
    R_1 = C_1.get_minor<3, 3>(0,0);

    t_0 = C_0.get_minor<3, 1>(0, 3);
    t_1 = C_1.get_minor<3, 1>(0, 3);
    

    std::cout << "[sfm] R_0: " << R_0 << std::endl;
    std::cout << "[sfm] t_0: " << t_0 << std::endl;

    std::cout << "[sfm] R_1: " << R_1 << std::endl;
    std::cout << "[sfm] t_1: " << t_1 << std::endl;


    //creating extrinsics matrix for the two frames ---> A = [R|t]
    cv::hconcat(R_0, t_0, A_0); 
    cv::hconcat(R_1, t_1, A_1); 

    std::cout << "[sfm] A_0: " << A_0 << std::endl;
    std::cout << "[sfm] A_1: " << A_1 << std::endl;
    

    // P = K * [R|t]
    P_0 = K_ * A_0; 
    P_1 = K_ * A_1;

    std::cout << "[sfm] P_0: " << P_0 << std::endl;
    std::cout << "[sfm] P_1: " << P_1 << std::endl;
    
    cv::triangulatePoints(P_0, P_1, pts_0, pts_1, points3d_);

}

void simple_sfm::SimpleSFM::runSFMPipeline() {

    initalizePipeline();

    int num_frames_ = 4; 

    int curr_frame_idx_ = 1; 

    while(curr_frame_idx_ < num_frames_ - 1) {

        bool flag_ = processNextFrame(curr_frame_idx_);

        std::cout << "curr_frame_idx_: "  << curr_frame_idx_ << " flag_: " << flag_ << std::endl;

        if(!flag_) {

            curr_frame_idx_++;
            continue;

        }

        updateCameraPose(C_1); 

        displayCameraPose(C_1);

        
        std::vector<cv::Matx41d> points3D_;

        triangulate3DPoints(points3D_);

        std::cout << "points3D_.size(): " << points3D_.size() << std::endl;

        
        C_0 = C_1;
        curr_frame_idx_++;



        //update2DMatchList

        //triangulateAndProcessPointCloud

        //C_k_minus_1 = C_k

    }


    //bool flag_ = initalizePipeline(); 


    //update_intial_camera_pose(C_k_minus_1);

    //process_next_frame(C_k, features_pts_1, feature_pts_2);

    //update_projection_matrix(C_k_minus_1, P_k_minus_1); 
    //update_projection_matrix(C_k, P_k);

    //triangulatePoints();
    //update_global_point_cloud();
    
    //bundle_adjustment() 
    //visualize()

}

