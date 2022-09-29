#ifndef SFM_UTIL_H
#define SFM_UTIL_H


// ========================================================
// =============   Utility functions     =================
// ========================================================


#include "ds.hpp"

#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>

namespace simple_sfm{

    void convertPointsFromHomogeneous(cv::Mat &pts_4d_, std::vector<Point3D> &pts_3d_);
    
    Eigen::Matrix<float, 9, 1> packCameraIntrinsics(const cv::Mat &K);
    Eigen::Matrix<float, 6, 1> packCameraExtrinsics(const cv::Mat &A_);


    
    struct compare_kp {
        bool operator() (const cv::KeyPoint& a_, const cv::KeyPoint& b_) const {
            
            return ((a_.pt.x < b_.pt.x) ||((a_.pt.x == b_.pt.x) && (a_.pt.y < b_.pt.y))); // if x<y then x will come before y. Change this condition as per requirement
        
        }
    };

    struct compare_pt2d {
        bool operator() (const Point2D& a_, const Point2D& b_) const {
            
            return ((a_.x < b_.x) ||((a_.x == b_.x) && (a_.y < b_.y))); // if x<y then x will come before y. Change this condition as per requirement
      
        }
    };

    cv::Matx34f convert44to33Mat(const cv::Mat &a_);


};

#endif