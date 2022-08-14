#include "../../include/vis.hpp"


simple_sfm::Vis::Vis(){



}

void simple_sfm::Vis::viusalizeCameraPose(const cv::Matx44d &mat_){

    double px_ =  mat_(0,3) + 200 ;
    double py_ = mat_(2,3) + 100;

    std::cout << "(" << px_ << "," << py_ << ")" << std::endl;


    cv::circle(predictions_mat_, cv::Point(px_, py_) ,1, CV_RGB(255,0,0), 2);
    //cv::circle(gt_mat_, cv::Point(gx_, gy_) ,1, CV_RGB(255,255, 255), 2);

    

    //cv::Mat combined_mat_ = gt_mat_ + predictions_mat_;
    //cv::hconcat(gt_mat_, predictions_mat_, combined_mat_);
    
    //cv::imshow("Ground Truth", combined_mat_);
    
    cv::imshow("Trajectory", predictions_mat_);
    cv::waitKey(10);


}

