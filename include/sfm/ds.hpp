#ifndef DF_H
#define DF_H

//  ======================================================================
//  =============   Relevant Data Structures    ==========================
//  ======================================================================



#include <opencv2/opencv.hpp>
#include <Eigen/Core>




using KeyPoints     =       std::vector<cv::KeyPoint>;
using Points2d      =       std::vector<cv::Point2d>;
using Points3d      =       std::vector<cv::Point3d>;
using Matches       =       std::vector<cv::DMatch>;
using ImagePair     =       std::pair<int, int>;



const int       MIN_POINT_COUNT_FOR_HOMOGRAPHY  =   100;
const double    POSE_INLIERS_MINIMAL_RATIO      =   0.5;
const double    POSE_INLIERS_MINIMAL_COUNT      =   25;
const double    MIN_REPROJECTION_ERROR          =   10.0;

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


/*using Point2D       =       cv::Point2f ;
using Point3D       =       cv::Point3d;

using Points2D      =       std::vector<Point2D> ;
using Points3D      =       std::vector<cv::Point3f> ;

using Match         =       cv::DMatch; 
using Matches       =       std::vector<Match>;
using Poses         =       std::vector<cv::Mat>;

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector2d Vec2d;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3d;

typedef Eigen::Matrix<float , 6, 1> Vec6f;
typedef Eigen::Matrix<double , 6, 1> Vec6d;

typedef Eigen::Matrix<float, 3, 3> Mat3f;
typedef Eigen::Matrix<double, 3, 3> Mat3d;

typedef Eigen::Matrix<float, 3, 4> Mat34f;
typedef Eigen::Matrix<double, 4, 4> Mat4d;
*/




#endif