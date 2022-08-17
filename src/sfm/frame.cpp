#include "../../include/sfm/frame.hpp"


simple_sfm::Frame::Frame() {
    
   // Frame::orb_ = cv::ORB::create(5000);

}

void simple_sfm::Frame::Points2DFromKeyPoints(const KP &kp_ , Points2D &points_) 
{
    assert(("[frames.cpp]" , points_.size() == 0));

    for (const auto &t : kp_) {
        
        points_.push_back(t.pt);

    }

}

double simple_sfm::Frame::GetAbsoluteScale(const cv::Matx34f &curr_pose_ , const cv::Matx34f &prev_pose_)
{

    //std::cout << "[frame]: GetAbsoluteScale" << std::endl;
    
    cv::Point3f prev_point_ = {prev_pose_(0,3), prev_pose_(1,3), prev_pose_(2,3)};
    cv::Point3f curr_point_ = {curr_pose_(0,3), curr_pose_(1,3), curr_pose_(2,3)};
    
    cv::Point3f diff_ = (curr_point_ - prev_point_);

    double scale_ = cv::norm(diff_);

    //std::cout << "[frame] scale_: " << scale_  << std::endl;
    return scale_;

}


void simple_sfm::Frame::Points2DFromFrames(const cv::String &img_a_, const cv::String &img_b_, Points2D &pts_a_, Points2D &pts_b_) 
{

    //std::cout << "[frames] Point2DFromFrames" << std::endl;


    KeyPoints kp_a_ , kp_b_;  
    ExtractKeyPoints(img_a_, img_b_, kp_a_, kp_b_);

   // std::cout << "[frames] H0" << std::endl;


    Matches good_matches_ ; 
    GetGoodMatches(img_a_, img_b_, kp_a_, kp_b_, good_matches_);

    //std::cout << "[frames] H1" << std::endl;

    KeyPoints kp_a_mat_, kp_b_mat_;
    ExtractMatchingKeyPoints(kp_a_, kp_b_, good_matches_, kp_a_mat_, kp_b_mat_);

    //std::cout << "[frames] H2" << std::endl;

    Points2DFromKeyPoints(kp_a_mat_, pts_a_);
    Points2DFromKeyPoints(kp_b_mat_, pts_b_); 

   // std::cout << "[frames] H3" << std::endl;

}

void simple_sfm::Frame::GetGoodMatches(const cv::String &img_a_, const cv::String &img_b_,  KeyPoints &kp_a_,  KeyPoints &kp_b_, Matches &good_matches_)
{

    assert(("[frames.cpp]", (int)good_matches_.size() == 0));
    assert(("[frames.cpp]", (int)kp_a_.size() == (int)kp_b_.size()));

    
    cv::Mat img_1 = cv::imread(img_a_.c_str());
    cv::Mat img_2 = cv::imread(img_b_.c_str());


    cv::Mat image_one, image_two;
    cv::cvtColor(img_1, image_one, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_2, image_two, cv::COLOR_BGR2GRAY);
    
    cv::Mat mask = cv::Mat();
    cv::Mat des_1, des_2;
    
    
    //*** extracting descriptors from keypoints
    Frame::orb_->detectAndCompute(image_one, mask,kp_a_, des_1);
    Frame::orb_->detectAndCompute(image_two, mask, kp_b_, des_2);
        
    des_1.convertTo(des_1, 0);
    des_2.convertTo(des_2, 0);
    
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    Matches brute_hamming_matches;
    matcher->match(des_1, des_2, brute_hamming_matches);

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < des_1. rows; i++ )
    {
        double dist = brute_hamming_matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    //Matches good_matches;
    
    for ( int i = 0; i < des_1.rows; i++ )
    {
        if ( brute_hamming_matches[i].distance <= std::max( 2*min_dist, 20.0 ) )
        {
            good_matches_.push_back (brute_hamming_matches[i]);
        }
    }

}

void simple_sfm::Frame::ExtractKeyPoints(const cv::String &img_a_, const cv::String &img_b_, KeyPoints &kp_1, KeyPoints &kp_2) 
{   

    cv::Mat image_one, image_two;
    
    cv::Mat img_1 = cv::imread(img_a_.c_str());
    cv::Mat img_2 = cv::imread(img_b_.c_str());

    cv::cvtColor(img_1, image_one, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_2, image_two, cv::COLOR_BGR2GRAY);

    Points2D corners_one, corners_two;
    
    int maxCorners = 2000;

    double qualityLevel = 0.01;

    double minDistance = 1.0;

    cv::Mat mask = cv::Mat();
    
    int blockSize = 1;

    bool useHarrisDetector = false;

    double k = 0.04;

    
    //*** keypoints extraction
    cv::goodFeaturesToTrack( image_one, corners_one, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

    cv::goodFeaturesToTrack( image_two, corners_two, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

    //cv::Keypoint::covert()

    cv::KeyPoint::convert(kp_1, corners_one, std::vector<int>());
    cv::KeyPoint::convert(kp_2, corners_two, std::vector<int>());

}

void simple_sfm::Frame::ExtractMatchingKeyPoints(const KP &kp_a_, const KP &kp_b_, const Matches good_matches_, KP &kp_a_mat_, KP &kp_b_mat_)
{   

   // std::cout << "[frames.cpp] ExtractMatchingKeyPoints" << std::endl;

    assert(("[frames]" , (int)kp_a_mat_.size() == 0 && (int)kp_b_mat_.size() == 0));

    //std::cout << "[frames] kp_a.size(): " << (int)kp_a_.size() << " kp_b.size(): " << (int)kp_b_.size() << std::endl;

    //assert(("[frames.cpp]", (int)kp_a_.size() == (int)kp_b_.size()));

    for (auto match : good_matches_) {
        
        kp_a_mat_.push_back(kp_a_[match.queryIdx]);
        kp_b_mat_.push_back(kp_b_[match.trainIdx]);

    }

}