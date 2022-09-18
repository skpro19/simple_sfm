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

    cv::Matx34f convert44to33Mat(const cv::Mat &a_)
    {
        assert(a_.size() == cv::Size(4, 4));

        cv::Mat roi = a_(cv::Rect2f(0 , 0, 4 , 3));
        
        assert(roi.size() == cv::Size(4, 3));
        
        return roi;

    }


};