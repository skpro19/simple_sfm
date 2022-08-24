#include "../../include/sfm/bookkeeping.hpp"
#include "../../include/sfm/frame.hpp"

simple_sfm::BookKeeping::BookKeeping() {

    std::cout << "[bkp] Inside the BKP constructor!" << std::endl;

    //TODO: Initialize last_frame_;
}


bool simple_sfm::BookKeeping::hasPoint2d(const Point2D &pt_) const {

    return (point_cloud_map_.count(pt_) > 0);

}

Point3D simple_sfm::BookKeeping::getPoint3d(const Point2D &pt_) const{

    assert(("[bkp]" , point_cloud_map_.count(pt_) > 0));

    return point_cloud_map_[pt_];

}

void simple_sfm::BookKeeping::addNextFrame(const Points2D &last_pts_ , const Points2D &curr_pts_ , const Points2D &img_pts_ , const Points3D &obj_pts_) const{

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

//TODO: Might need to re-write

void simple_sfm::BookKeeping::updatePointCloudMap(const Points2D &b_, const cv::Mat pts_3d_) {
    
    assert(("[bkp]" , ((int)b_.size() == (int)pts_3d_.size().width)));

    int n_ = (int)b_.size(); 

    std::cout << "[bkp] Before updating ----> point_cloud_map.size() " << (int)point_cloud_map_.size() << std::endl;

    for(int i = 0 ; i < n_; i++) {

        Point2D pt_b_ = b_[i] ; 

        cv::Mat pt_4d_ = pts_3d_.col(i);

        std::cout << "[bkp] pt_4d_.size(): " << pt_4d_.size() << std::endl; 

        assert(("[bkp]" , ((int)pt_4d_.size().height == 4)));

        cv::Mat pt_3d_; 
        cv::convertPointsFromHomogeneous(pt_4d_, pt_3d_);

        std::cout << "[bkp] pt_3d_.size(): " << pt_3d_.size() << std::endl; 
        assert(("[bkp]" , ((int)pt_3d_.size().height == 3)));

        if(point_cloud_map_.count(pt_b_) > 0 ) {continue;}

        point_cloud_map_[pt_b_] = pt_3d_;

    }
    
    std::cout << "[bkp] After updating ----> point_cloud_map.size() " << (int)point_cloud_map_.size() << std::endl;

    
}