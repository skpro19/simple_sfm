#include "../../include/sfm/vis.hpp"



#include "../../include/sfm/sfm.hpp"

simple_sfm::Vis::Vis()
{
    
   

}

void simple_sfm::Vis::displayFrame(const cv::Mat &frame_) 
{

    
}

void simple_sfm::Vis::updateGroundPose(const cv::Matx34d &pose_)
{

    double x_ = pose_(0, 3) + 200 , y_ = pose_(2, 3) + 100;

    cv::Point p_(x_, y_);
    cv::circle(Vis::gt_mat_, p_ ,1, cv::viz::Color::celestial_blue(), 1);
    cv::imshow("gt_", Vis::gt_mat_);
    
}    

void simple_sfm::Vis::updatePredictedPose(const cv::Mat &pose_)
{

    std::cout << "Inside updatePredictedPose!" << std::endl;

    double x_ = pose_.at<double>(0, 3) + 200 , y_ = pose_.at<double>(2, 3) + 100;

    cv::Point p_(x_, y_);
    cv::circle(Vis::gt_mat_, p_ ,1, cv::viz::Color::white(), 5);
    cv::imshow("gt_", Vis::gt_mat_);
    cv::waitKey(0);

} 


void simple_sfm::Vis::updateBAPose(const cv::Mat &pose_)
{

    double x_ = pose_.at<double>(0, 3) + 200 , y_ = pose_.at<double>(2, 3) + 100;

    cv::Point p_(x_, y_);

    cv::circle(Vis::gt_mat_, p_ ,1, cv::viz::Color::orange(), 1);
    cv::imshow("gt_", Vis::gt_mat_);
    cv::waitKey(1000);

} 


void simple_sfm::Vis::drawKeyPoints(cv::Mat &mat_, const Points2d &kp_a_, const Points2d &kp_b_)
{

    assert(("[Vis]" , (int)kp_a_.size() == (int)kp_b_.size()));

    int n_ = (int)kp_a_.size() ;

    for(int i = 0 ; i < n_; i++) 
    {

        cv::Point2d p_ = kp_a_[i];
        cv::Point2d q_ = kp_b_[i];
        
        cv::circle(mat_, p_ ,1, cv::viz::Color::pink(), 2);
        cv::circle(mat_, q_ ,1, cv::viz::Color::purple(), 2);
        
    }
    
}

void simple_sfm::Vis::drawKeyPoints(cv::Mat &mat_, const Points2d &kp_a_)
{

    assert(("[Vis]" , (int)kp_a_.size() > 0));

    
    int n_ = (int)kp_a_.size() ;

    for(int i = 0 ; i < n_; i++) {

        cv::Point2d p_ = kp_a_[i];        
        cv::circle(mat_, p_ ,3, cv::viz::Color::green(), 2);

    }
    
}

