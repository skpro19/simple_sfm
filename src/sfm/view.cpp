#include "../../include/sfm/view.hpp"


simple_sfm::View::View(){


}

void simple_sfm::View::updateView(  const std::vector<cv::Point2f> &kp_in_, 
                                    const std::vector<cv::Point3f> &pts_3d_, 
                                     const cv::Matx33f &K_,
                                    const cv::Mat &C_k_){
                            
    used_ = 1;
    processKeypoints(kp_in_);
    processCameraIntrinsics(K_);
    processCameraExtrinsics(C_k_);
    process3dPoints(pts_3d_);


}

void simple_sfm::View::processKeypoints(const std::vector<cv::Point2f> &kps_){

    pts_2d_ = std::make_shared<std::vector<Vec2f> >();
    
    for(int i = 0; i < (int)kps_.size(); i++) {

        pts_2d_->push_back({kps_[i].x, kps_[i].y});
        
    }

    assert(pts_2d_->size() == kps_.size());

}

void simple_sfm::View::processCameraIntrinsics(const cv::Matx33f &in_){

    //cam_intrinsics_ = std::make_shared<Mat3f>(); 
    cam_intrinsics_ << in_(0,0), in_(0,1), in_(0,2),
            in_(1,0), in_(1,1), in_(1,2),
            in_(2,0), in_(2,1), in_(2,2);


}

void simple_sfm::View::processCameraExtrinsics(const cv::Mat &mat_){

    assert(mat_.size() == cv::Size(4,4));

    cv::Matx44f ext_(mat_);

    cam_extrinsics_ <<  ext_(0,0), ext_(0,1), ext_(0,2), ext_(0,3),
                        ext_(1,0), ext_(1,1), ext_(1,2), ext_(1,3),
                        ext_(2,0), ext_(2,1), ext_(2,2), ext_(2,3);


}

//TODO --> Tune norm + check what does norm mean    
void simple_sfm::View::process3dPoints(const std::vector<cv::Point3f> &points_3d_){

    int n_ =    (int)points_3d_.size(); 
    int m_  =   (int)point_cloud_.size();

    pts_3d_ = std::make_shared<std::vector<Vec3f> >();

    int new_pts_counter_ = 0 ;        

    std::cout << "old point_cloud.size(): " << (int)point_cloud_.size() << std::endl;

    for(int j =0 ; j < n_ ; j++){

        Vec3f curr_pt_;
        curr_pt_ << points_3d_[j].x, points_3d_[j].y, points_3d_[j].z;
        
        bool found_ = 0 ;

        float mn_dis_ = 1000 * 1000; 
        int idx_= -1;

        for(int i = 0 ; i < m_; i++){
            
            Vec3f pt_ = View::point_cloud_[i];
            
            Vec3f diff_ = curr_pt_ - pt_;

            float norm_ = diff_.norm();
            
            if(norm_ < 0.1) {
                
                if(norm_ < mn_dis_) {

                    mn_dis_ = norm_;
                    idx_ = i;

                }

                found_ = 1;

            }

        }  

        if(found_) {

            pts_3d_->push_back(point_cloud_[idx_]);

        }else{
            
            View::point_cloud_.push_back(curr_pt_);
            pts_3d_->push_back(curr_pt_);
            new_pts_counter_++;

        }

    }

    std::cout << "new_pts_counter_: " << new_pts_counter_ << std::endl;
    std::cout << "new point_cloud.size(): " << point_cloud_.size() << std::endl;


}