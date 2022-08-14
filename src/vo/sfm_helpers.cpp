#include "../../include/visual_odom.hpp"


void VisualOdom::update2DMatches(std::vector<cv::Point2f> &v1, std::vector<cv::Point2f> &v2){

    v1 = kp_1f; 
    v2 = kp_2f;

}