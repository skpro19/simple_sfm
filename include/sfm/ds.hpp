#ifndef DF_H
#define DF_H

//  ======================================================================
//  =============   Relevant Data Structures    ==========================
//  ======================================================================



#include <opencv2/opencv.hpp>
//#include <Eigen/Core>




using KeyPoints         =       std::vector<cv::KeyPoint>;
using Points2d          =       std::vector<cv::Point2d>;
using Points3d          =       std::vector<cv::Point3d>;
using Matches           =       std::vector<cv::DMatch>;
using ImagePair         =       std::pair<int, int>;


const cv::Rect &ROT_    =       cv::Rect(0, 0, 3, 3);
const cv::Rect &TRA_    =       cv::Rect(3, 0, 1, 3);


const int       MIN_POINT_COUNT_FOR_HOMOGRAPHY          =   100;
const int       MIN_POINT_COUNT_FOR_2D3DMATCH           =   30;
const double    POSE_INLIERS_MINIMAL_RATIO              =   0.5;
const double    POSE_INLIERS_MINIMAL_COUNT              =   25;
const double    MIN_REPROJECTION_ERROR                  =   10.0;
const double    MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE    =   0.01;
const double    MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE  =   20.0;

struct Features{
        
    KeyPoints keypoints;
    Points2d  points;
    cv::Mat   descriptors;  

};

struct CloudPoint3d{
    
    cv::Point3d point_;
    std::map<int, int> viewMap; //<view_idx, pt_idx_>

};

struct Match2D3D{

    Points2d pts_2d_; 
    Points3d pts_3d_;

};






#endif