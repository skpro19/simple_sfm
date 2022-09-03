#ifndef BKP_H
#define BKP_H



#include "ds.hpp"
#include "io.hpp"

#include <map>

namespace simple_sfm {

    struct compare {
        bool operator() (const Point2D& a_, const Point2D& b_) const {
            
            return ((a_.x < b_.x) ||((a_.x == b_.x) && (a_.y < b_.y))); // if x<y then x will come before y. Change this condition as per requirement
        
        }
    };
            

    class BookKeeping{

        public: 
            
            BookKeeping();

            
            void addNextFrame(const Points2D &last_pts_, const Points2D &curr_pts_, Points2D &img_pts_, Points3D &obj_pts_);

            void initializeGlobalPointCloud(const std::vector<Point3D> &pts_3d_);
            
            void initialize2D3DCorrespondance(const std::vector<Point2D> &pts_2d_ , const std::vector<Point3D> &pts_3d_);
            
            void updateGlobalPointCloud(const Points2D &pts_2d_, Points3D &pts_3d_);

            bool hasPoint2d(const Point2D &pt_) const;
            bool hasPoint3d(const Point3D &pt_3d_) const;

            Point3D getPoint3d(const Point2D &pt_);
            
        private:

                
            
            std::vector<Point3D> global_point_cloud_;
            std::map<Point2D, Point3D, compare> corr_2d_to_3d_;
            

            //TODO ---> TUNE
            const int delta_ = 0.1;

    };

};

#endif