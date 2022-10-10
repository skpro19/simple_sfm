//#include "../../include/sfm/frame.hpp"
//#include "../../include/sfm/sfm_utility.hpp"

#include "../../include/sfm/sfm.hpp"

#include <opencv2/viz/types.hpp>

simple_sfm::Frame::Frame() {
    
   // Frame::orb_ = cv::ORB::create(5000);

}

void simple_sfm::Frame::alignFeaturesUsingMatches(  const Features &f1_, const Features &f2_, 
                                                    Features &f1_mat_, Features &f2_mat_, 
                                                    std::vector<int> &ref_f1_, std::vector<int> &ref_f2_,
                                                    const Matches &matches_) {

    Features f1_mat_, f2_mat_;
    
    f1_mat_.keypoints.resize(0); 
    f2_mat_.keypoints.resize(0); 

    f1_mat_.points.resize(0); 
    f2_mat_.points.resize(0); 

    ref_f1_.resize(0);
    ref_f2_.resize(0);


    for(int i = 0 ;i  < (int)matches_.size(); i++) {

        f1_mat_.keypoints.push_back(f1_.keypoints[matches_[i].queryIdx]);
        f2_mat_.keypoints.push_back(f2_.keypoints[matches_[i].trainIdx]);
        
        f1_mat_.points.push_back(f1_.points[matches_[i].queryIdx]);
        f2_mat_.points.push_back(f2_.points[matches_[i].trainIdx]);

        ref_f1_.push_back(matches_[i].queryIdx);
        ref_f2_.push_back(matches_[i].trainIdx);

    }


}

int simple_sfm::Frame::getHomographyInliersCount(const Features &f1_, const Features &f2_, const Matches &matches_){

    Features f1_mat_, f2_mat_;
    
    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_, matches_);

    assert(f1_mat_.keypoints.size()  == matches_.size());

    cv::Mat inlier_mask_, H_;
    H_ = cv::findHomography(f1_mat_.points, f2_mat_.points, inlier_mask_, cv::RANSAC);

    int inlier_cnt_ =  cv::countNonZero(inlier_mask_);

    if(H_.empty()) {
        
        std::cout << "H_ is empty!" << std::endl;
        inlier_cnt_ = 0 ;
    
    }

    return inlier_cnt_;

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


