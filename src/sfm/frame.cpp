//#include "../../include/sfm/frame.hpp"
//#include "../../include/sfm/sfm_utility.hpp"

#include "../../include/sfm/sfm.hpp"

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


Matches simple_sfm::Frame::getMatches(const Features &f1_, const Features &f2_){

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::vector<cv::DMatch> brute_hamming_matches;
    matcher->match(f1_.descriptors, f2_.descriptors, brute_hamming_matches);

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < f1_.descriptors. rows; i++ )
    {
        double dist = brute_hamming_matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    Matches good_matches;
    
    for ( int i = 0; i < f1_.descriptors.rows; i++ )
    {
        if ( brute_hamming_matches[i].distance <= std::max( 2*min_dist, 20.0 ) )
        {
            good_matches.push_back (brute_hamming_matches[i]);
        }
    }

    return good_matches;

}


void simple_sfm::Frame::keypointsToPoints(Features &f_){

    f_.keypoints.resize(0);

    for(const auto &kp_: f_.keypoints) {

        f_.points.push_back(kp_.pt);
    }

    assert(f_.keypoints.size() == f_.points.size());

}


void simple_sfm::Frame::extractFeaturesAndDescriptors(const cv::Mat &img_, Features &features_){
    
    features_.descriptors.resize(0);
    features_.keypoints.resize(0);
    features_.points.resize(0);

    cv::Mat image_;
    cv::cvtColor(img_, image_, cv::COLOR_BGR2GRAY);
    
    cv::Mat mask = cv::Mat();
    
    cv::Ptr<cv::ORB>orb_ = cv::ORB::create(5000);

    orb_->detectAndCompute(image_, mask, features_.keypoints, features_.descriptors);
        
    features_.descriptors.convertTo(features_.descriptors, 0);

}


