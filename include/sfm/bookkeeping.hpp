


#include "ds.hpp"
#include "io.hpp"

#include <map>

namespace simple_sfm {

    class BookKeeping{

        public: 
            
            BookKeeping();

            void addNextFrame(const Point2D &last_pts_, const Points2D &curr_pts_, const Points2D &img_pts_, const Points3D &obj_pts_) const;

            void updatePointCloudMap(const Points2D &pts_curr_, const cv::Mat pts_3d_);
            
            bool hasPoint2d(const Point2D &pt_) const;
            
            Point3D getPoint3d(const Point3D &pt_) const;
            
        private:

            
            std::map<Points2D, Points3D> point_cloud_map_; //2d-3d mapping

            cv::String last_frame_;

            


    };

};
