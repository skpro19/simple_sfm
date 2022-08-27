#include "../../include/sfm/bookkeeping.hpp"
#include "../../include/sfm/frame.hpp"

simple_sfm::BookKeeping::BookKeeping() {

    std::cout << "[bkp] Inside the BKP constructor!" << std::endl;

    //TODO: Initialize last_frame_;
}

void simple_sfm::BookKeeping::initializeGlobalPointCloud(const std::vector<Point3D> &pts_3d_) {

    assert(("[bkp]" , (int)global_point_cloud_.size() == 0));

    std::cout << "[bkp] ----> initiliazeGlobalPointCloud function" << std::endl;

    std::copy(pts_3d_.begin(), pts_3d_.end(), std::back_inserter(global_point_cloud_));

    std::cout << "[sfm] global_point_cloud.size() after initialization --> " << global_point_cloud_.size() << std::endl;

}

void simple_sfm::BookKeeping::update2D3DCorrespondance(const std::vector<Point2D> &pts_2d_ , const std::vector<Point3D> &pts_3d_){

    //corr_2d_to_3d_.clear(); 

    std::cout << "[bkp] -----> intialize2D3DCorrespondace " << std::endl;

    assert(("[bkp]" , corr_2d_to_3d_.size() == 0));
    assert(("[bkp]" , pts_2d_.size() == pts_3d_.size()));

    int n_ = (int)pts_2d_.size(); 

    for(int i = 0; i < n_ ; i++) { 

        Point2D pt_2d_ = pts_2d_[i]; 
        Point3D pt_3d_ = pts_3d_[i]; 

        corr_2d_to_3d_[pt_2d_] = pt_3d_;

    }

}

bool simple_sfm::BookKeeping::hasPoint2d(const Point2D &pt_) const {

    return (corr_2d_to_3d_.count(pt_) > 0);

}

bool simple_sfm::BookKeeping::hasPoint3d(const Point3D &pt_3d_) const {

    int n_  = (int)global_point_cloud_.size(); 

    for(int i = 0; i < n_; i++) {

        Point3D pt_ = global_point_cloud_[i]; 

        Point3D diff_ = (pt_.x - pt_3d_.x , pt_.y - pt_3d_.y , pt_.z - pt_3d_.z);

        double dis_ = cv::norm(diff_);  

        if(dis_ <= delta_) {return true; }  

    }

    return false;

}

Point3D simple_sfm::BookKeeping::getPoint3d(const Point2D &pt_) const{

    assert(("[bkp]" , corr_2d_to_3d_.count(pt_) > 0));

    return corr_2d_to_3d_[pt_];

}

void simple_sfm::BookKeeping::addNextFrame(const Points2D &last_pts_ , const Points2D &curr_pts_ , Points2D &img_pts_ , Points3D &obj_pts_) const{

    assert(("[bkp]" , (int)last_pts_.size() == (int)curr_pts_.size()));
    assert(("[bkp]" , (int)img_pts_.size() == 0));
    assert(("[bkp]" , (int)obj_pts_.size() == 0));

    int n_ = (int)last_pts_.size(); 

    for(int i = 0;i < n_ ; i++) {

        Point2D pt_ = last_pts_[i]; 

        if(hasPoint2d(pt_)){

            Point3D pt_3d_ = getPoint3d(pt_);
            
            img_pts_.push_back(curr_pts_[i]);
            obj_pts_.push_back(pt_3d_);

        }

    }
    
    assert(("[bkp]" , (int)obj_pts_.size() == (int)img_pts_.size()));

}


void simple_sfm::BookKeeping::updateGlobalPointCloud(const Points2D &pts_2d_, std::vector<Point3D> &pts_3d_) {
    
    assert(("[bkp]" , ((int)pts_2d_.size() == (int)pts_3d_.size())));

    corr_2d_to_3d_.clear(); 
    assert(("[bkp]" , ((int)corr_2d_to_3d_.size() == 0)));

    int cnt_ = 0 ;

    int n_ = (int)pts_2d_.size(); 

    std::cout << "[bkp] Before updating ----> global_point_cloud.size() " << (int)global_point_cloud_.size() << std::endl;

    for(int i = 0 ;i < n_; i++) {

        Point3D pt_3d_ = pts_3d_[i] ; 
        Point2D pt_2d_ = pts_2d_[i];

        bool pt_found_ = hasPoint3d(pt_3d_); 

        if(!pt_found_) {

            global_point_cloud_.push_back(pt_3d_);
            cnt_++; 
        }

        corr_2d_to_3d_[pt_2d_] = pt_3d_;
        
    }


    std::cout << "[bkp] " << cnt_ << " insertions made to global_point_cloud!" << std::endl;
    
    assert(("[bkp]" , ((int)pts_2d_.size() == (int)corr_2d_to_3d_.size())));

    std::cout << "[bkp] After updating ----> global_point_cloud.size() " << (int)global_point_cloud_.size() << std::endl;

}