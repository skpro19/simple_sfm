#ifndef SFM_UTIL_H
#define SFM_UTIL_H


#include "sfm.hpp"

namespace simple_sfm{

    void convertPointsFromHomogeneous(cv::Mat &pts_4d_, std::vector<Point3D> &pts_3d_);

    struct compare_kp {
        bool operator() (const cv::KeyPoint& a_, const cv::KeyPoint& b_) const {
            
            return ((a_.pt.x < b_.pt.x) ||((a_.pt.x == b_.pt.x) && (a_.pt.y < b_.pt.y))); // if x<y then x will come before y. Change this condition as per requirement
        
        }
    };

};

#endif