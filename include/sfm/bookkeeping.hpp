#ifndef BKP_H
#define BKP_H



#include "ds.hpp"
#include "io.hpp"

#include <map>

namespace simple_sfm {

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

            struct cmpPoint2D {
                
                bool operator()(const cv::Point& a, const Point2D &b) const {
                    
                    return ((a.x > b.x) || ((a.x == b.x) && (a.y > b.y)));
                
                }
            };

            std::vector<Point3D> global_point_cloud_;
            std::map<Point2D, Point3D, cmpPoint2D> corr_2d_to_3d_;
            

            //TODO ---> TUNE
            const int delta_ = 0.1;

    };

};

#endif