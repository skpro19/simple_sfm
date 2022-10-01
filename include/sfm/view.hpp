#ifndef VIEW_H
#define VIEW_H



#include "sfm.hpp"

#include <eigen3/Eigen/Core>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;

typedef Eigen::Matrix<float, 3, 3> Mat3f;
typedef Eigen::Matrix<float, 3, 4> Mat34f;


namespace simple_sfm{

    class View{

        public: 

            View();
            
            void updateView(const std::vector<cv::Point2f> &kp_in_, 
                            const std::vector<cv::Point3f> &pts_3d_, 
                            const cv::Matx33f &K_,
                            const cv::Mat &C_k_);

            void process3dPoints(const std::vector<cv::Point3f> &points_3d_);
            void processKeypoints(const std::vector<cv::Point2f> &keypoints_);
            void processCameraIntrinsics(const cv::Matx33f &intrinsics_);
            void processCameraExtrinsics(const cv::Mat &C_k_);

            
            int used_ = -1;
            std::shared_ptr <std::vector<Vec2f> >pts_2d_;
            Mat3f cam_intrinsics_;
            Mat34f cam_extrinsics_;
            std::shared_ptr<std::vector<Vec3f> >pts_3d_;
            
        
        private:



            
            inline static std::vector<Vec3f>  point_cloud_;

    };




};





#endif
