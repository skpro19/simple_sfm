#include "../../include/sfm/vis.hpp"


simple_sfm::Vis::Vis()
{
    
   

}

void simple_sfm::Vis::displayFrame(const cv::Mat &frame_) 
{

    cv::imshow("frame", frame_);
    cv::waitKey(100);

}

void simple_sfm::Vis::drawKeyPoints(cv::Mat &mat_, const Points2D &kp_a_, const Points2D &kp_b_)
{

    assert(("[Vis]" , (int)kp_a_.size() == (int)kp_b_.size()));

    //cv::Mat &mat_(frame_);

    int n_ = (int)kp_a_.size() ;

    for(int i = 0 ; i < n_; i++) {

        Point2D p_ = kp_a_[i];
        Point2D q_ = kp_b_[i];
        
        cv::circle(mat_, p_ ,1, cv::viz::Color::pink(), 2);
        cv::circle(mat_, q_ ,1, cv::viz::Color::purple(), 2);
        
    }
    
    cv::imshow("Features", mat_);
    cv::waitKey(1000);

}

void simple_sfm::Vis::drawKeyPoints(cv::Mat &mat_, const Points2D &kp_a_)
{

    assert(("[Vis]" , (int)kp_a_.size() > 0));

    
    int n_ = (int)kp_a_.size() ;

    for(int i = 0 ; i < n_; i++) {

        Point2D p_ = kp_a_[i];
        
        cv::circle(mat_, p_ ,3, cv::viz::Color::red(), 2);

        //cv::circle()
        
    }
    
    cv::imshow("Features", mat_);
    cv::waitKey(1000);

}
