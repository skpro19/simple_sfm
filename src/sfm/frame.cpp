#include "../../include/sfm/frame.hpp"
#include "../../include/sfm/sfm_utility.hpp"

#include <opencv2/viz/types.hpp>

simple_sfm::Frame::Frame() {
    
   // Frame::orb_ = cv::ORB::create(5000);

}


double simple_sfm::Frame::getScale(const cv::Mat &prev_poses_, const cv::Mat &curr_poses_) {

    cv::Point3d prev_point_ = {prev_poses_.at<double>(0,3), prev_poses_.at<double>(1,3), prev_poses_.at<double>(2,3)};
    cv::Point3d curr_point_ = {curr_poses_.at<double>(0,3), curr_poses_.at<double>(1,3), curr_poses_.at<double>(2,3)};
    cv::Point3d diff_ = (curr_point_ - prev_point_);

    double scale_ = cv::norm(diff_);
    
    return scale_;
}

void simple_sfm::Frame::extractAndMatchFeatures(const cv::Mat &img_1, const cv::Mat &img_2){
    
    kp_1_matched.clear(); 
    kp_2_matched.clear();

    cv::Mat image_one, image_two;
    cv::cvtColor(img_1, image_one, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_2, image_two, cv::COLOR_BGR2GRAY);
    
    cv::Mat mask = cv::Mat();
    cv::Mat des_1, des_2;
    
    cv::Ptr<cv::ORB>orb_ = cv::ORB::create(5000);


    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
    
    //*** extracting descriptors from keypoints
    orb_->detectAndCompute(image_one, mask, kp_1, des_1);
    orb_->detectAndCompute(image_two, mask, kp_2, des_2);
    
    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
        
    des_1.convertTo(des_1, 0);
    des_2.convertTo(des_2, 0);
    
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::vector<cv::DMatch> brute_hamming_matches;
    matcher->match(des_1, des_2, brute_hamming_matches);

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < des_1. rows; i++ )
    {
        double dist = brute_hamming_matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    std::vector<cv::DMatch> good_matches;
    
    for ( int i = 0; i < des_1.rows; i++ )
    {
        if ( brute_hamming_matches[i].distance <= std::max( 2*min_dist, 20.0 ) )
        {
            good_matches.push_back (brute_hamming_matches[i]);
        }
    }

    for (auto match : good_matches) {
        
        kp_1_matched.push_back(kp_1[match.queryIdx]);
        kp_2_matched.push_back(kp_2[match.trainIdx]);

    }
    //std::cout << "kp_1.size(): " << (int)kp_1.size() << std::endl;
    //std::cout << "kp_1_matched.size(): " << (int)kp_1_matched.size() << std::endl;
        
}
