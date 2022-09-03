#include "../../include/sfm/vis.hpp"
#include <opencv2/viz/types.hpp>



simple_sfm::Vis::Vis()
{
    
    Vis::mat_ = cv::Mat::zeros(800, 500, CV_8UC3);

}

void simple_sfm::Vis::drawKeyPoints(const KeyPoints &kp_a_, const KeyPoints &kp_b_)
{

    assert(("[Vis]" , (int)kp_a_.size() == (int)kp_b_.size()));

    int n_ = (int)kp_a_.size() ;

    for(int i = 0 ; i < n_; i++) {

        Point2D p_ = kp_a_[i].pt;
        Point2D q_ = kp_b_[i].pt;
        
        cv::circle(mat_, p_ ,1, cv::viz::Color::pink(), 2);
        cv::circle(mat_, q_ ,1, cv::viz::Color::purple(), 2);
        
    }
    
    cv::imshow("Features", mat_);
    cv::waitKey(10);

}
