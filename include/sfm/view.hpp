#ifndef VIEW_H
#define VIEW_H



#include "sfm.hpp"

#include <eigen3/Eigen/Core>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector2d Vec2d;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3d;

typedef Eigen::Matrix<float , 6, 1> Vec6f;
typedef Eigen::Matrix<double , 6, 1> Vec6d;

typedef Eigen::Matrix<float, 3, 3> Mat3f;
typedef Eigen::Matrix<double, 3, 3> Mat3d;

typedef Eigen::Matrix<double, 4, 4> Mat4d;
typedef Eigen::Matrix<float, 3, 4> Mat34f;


namespace simple_sfm{

    class View{

        public: 

            View();
            
            void updateView(const std::vector<cv::Point2d> &kp_in_, 
                            const std::vector<cv::Point3d> &pts_3d_, 
                            const cv::Matx33d &K_,
                            const cv::Mat &C_k_);

            void process3dPoints(const std::vector<cv::Point3d> &points_3d_);
            void processKeypoints(const std::vector<cv::Point2d> &keypoints_);
            void processCameraIntrinsics(const cv::Matx33d &intrinsics_);
            void processCameraExtrinsics(const cv::Mat &C_k_);

            
            int used_ = -1;
            std::shared_ptr <std::vector<Vec2d> >pts_2d_;
            Mat3d cam_intrinsics_;
            Vec6d cam_extrinsics_;
                
            std::shared_ptr<std::vector<int> >indices_3d_pts_;
            
            inline static std::vector<Vec3d>  point_cloud_;

    };




};





#endif
