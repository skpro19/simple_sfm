#include "../../include/sfm.hpp"



simple_sfm::SimpleSFM::SimpleSFM(const std::string &folder_){

    std::cout << "Inside the SimpleFSM Constructor!" << std::endl;

    vo_ = std::make_shared<VisualOdom>(folder_);
    vis_ = std::make_shared<Vis>();
    
}


void simple_sfm::SimpleSFM::updateCameraPose(cv::Matx44d &pose_){

    vo_->update_pose(pose_);

}

void simple_sfm::SimpleSFM::initalizePipeline() {

    vo_->initialize();
    updateCameraPose(C_k_minus_1);

    std::cout << "SFM Pipeline initialized with C_k_minus_1 set to: " << C_k_minus_1 << std::endl;

}

bool simple_sfm::SimpleSFM::processNextFrame(const int &curr_frame_id_){

    std::cout << "Processing frame " << curr_frame_id_ << std::endl;

    bool flag_ = vo_->process_next_frame(curr_frame_id_);

    if(!flag_) {

        std::cout << "Unable to process view no " << curr_frame_id_ << std::endl;
        return flag_;
    
    }

}


void simple_sfm::SimpleSFM::runSFMPipeline() {

    initalizePipeline();

    int num_frames_ = 100; 

    int curr_frame_idx_ = 1; 

    while(curr_frame_idx_ < num_frames_ - 1) {

        bool flag_ = processNextFrame(curr_frame_idx_++);

        if(!flag_) {continue;}

        updateCameraPose(C_k_); 

        vis_->viusalizeCameraPose(C_k_);

        
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

