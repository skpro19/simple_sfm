#include "../../include/sfm/sfm_utility.hpp"

namespace simple_sfm{


    bool checkForDuplicates(const std::vector<cv::Point2f> &a_  , const std::vector<cv::Point2f> &b_){

        std::set<std::pair<float, float> > sa_, sb_; 

        assert((int)a_.size() == (int)b_.size());
        
        int n_ = (int)a_.size() ;

        for(int i = 0 ;i < n_;  i++) {

            std::pair<float, float> pa_, pb_; 
            
            pa_ = {a_[i].x, a_[i].y},  pb_ = {b_[i].x, b_[i].y}; 
            
            sa_.insert(pa_);
            sb_.insert(pb_);
        
        }

        return( ( (int)sa_.size() == a_.size() ) && ( (int)sb_.size() == (int)b_.size()));

    }

    /*Eigen::Matrix<float, 9, 1> packCameraIntrinsics(const cv::Matx33f &K) {

        Eigen::Matrix<float, 9, 1> intrinsics_;

        //cv::cv2eigen(K, intrinsics_);

        intrinsics_ <<  K(0,0) , K(0,1), K(0,2),
                         K(1,0), K(1,1), K(1,2),
                         K(2,0), K(2,1), K(2,2); 

        return intrinsics_;

    }    

    Eigen::Matrix<float, 6, 1> packCameraExtrinsics(const cv::Matx34f &K) {

        Eigen::Matrix<float, 6, 1> extrinsics_;

        //cv::cv2eigen(K, intrinsics_);

        extrinsics_ << K(0,0) , K(0,1), K(0,2), K(0,3),
                        K(1,0), K(1,1), K(1,2), K(1,3),
                        K(2,0), K(2,1), K(2,2), K(2,3);

        return extrinsics_;

    }*/ 

    void convertPointsFromHomogeneous(cv::Mat &pts_4d_, std::vector<Point3D> &pts_3d_)
    {
        assert(("[sfm_utility]" , (int)pts_3d_.size() == 0));
        
        pts_4d_ = pts_4d_.t(); 

        int n_ = (int)pts_4d_.size().height; 

        pts_3d_.resize(n_);
        
        cv::convertPointsFromHomogeneous(pts_4d_, pts_3d_);
        
        assert(("[sfm_utility]" , (int)pts_3d_.size() == n_));
        
    }

    cv::Matx34f convert44to34Mat(const cv::Mat &a_)
    {
        assert(a_.size() == cv::Size(4, 4));

        cv::Mat roi = a_(cv::Rect2f(0 , 0, 4 , 3));
        
        assert(roi.size() == cv::Size(4, 3));
        
        return roi;

    }


};