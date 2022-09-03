#include "../../include/sfm/sfm_utility.hpp"

namespace simple_sfm{

    void convertPointsFromHomogeneous(cv::Mat &pts_4d_, std::vector<Point3D> &pts_3d_)
    {
        
        assert(("[sfm_utility]" , (int)pts_3d_.size() == 0));
        
        pts_4d_ = pts_4d_.t(); 

        int n_ = (int)pts_4d_.size().height; 

        pts_3d_.resize(n_);

        cv::convertPointsFromHomogeneous(pts_4d_, pts_3d_);

        assert(("[sfm_utility]" , (int)pts_3d_.size() == n_));
        
    }

};