#include "../../include/sfm/frame.hpp"
#include "../../include/sfm/sfm_utility.hpp"



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

double simple_sfm::Frame::GetAbsoluteScale(const cv::Matx34d &curr_pose_ , const cv::Matx34d &prev_pose_)
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

    KeyPoints kp_a_ , kp_b_;  
    ExtractKeyPoints(img_a_, img_b_, kp_a_, kp_b_);

    Matches good_matches_ ; 
    ExtractGoodMatches(img_a_, img_b_, kp_a_, kp_b_, good_matches_);

    KeyPoints kp_a_mat_, kp_b_mat_;
    ExtractMatchingKeyPoints(kp_a_, kp_b_, good_matches_, kp_a_mat_, kp_b_mat_);

    Points2DFromKeyPoints(kp_a_mat_, pts_a_);
    Points2DFromKeyPoints(kp_b_mat_, pts_b_); 

    //std::cout << "[frames] pts_a_.size(): " << (int)pts_a_.size() << std::endl;
    //sstd::cout << "[frames] pts_b_.size(): " << (int)pts_b_.size() << std::endl;
    

}

void simple_sfm::Frame::ExtractGoodMatches(const cv::String &img_a_, const cv::String &img_b_,  KeyPoints &kp_a_,  KeyPoints &kp_b_, Matches &good_matches_)
{

    assert(("[frames.cpp]", (int)good_matches_.size() == 0));
    //assert(("[frames.cpp]", (int)kp_a_.size() == (int)kp_b_.size()));

    
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
  
    int n_ = (int)brute_hamming_matches.size();

    std::cout << "[frame] brute_hamming_matches.size(): " << brute_hamming_matches.size() << std::endl;
  
    std::sort(brute_hamming_matches.begin(), brute_hamming_matches.end(), [](const cv::DMatch &a_, const cv::DMatch &b_){

        return a_.distance < b_.distance;

    });



    double mn_dist_ = brute_hamming_matches[0].distance;
    
    std::cout << "[frames] mn_dist_: " << mn_dist_ << std::endl;
    
    //keeps track of already matched points
   

    for ( int i = 0; i < n_; i++ )
    {   
        
        cv::DMatch m_ = brute_hamming_matches[i]; 
        
        double dis_ = m_.distance; 

        if(dis_ > 5 * mn_dist_) {

            break;

        }

        good_matches_.push_back(m_);

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

    
    std::set<cv::KeyPoint, compare_kp> train_vis_ , query_vis_;
    
    int cnt_ = 0; 

    for (auto match : good_matches_) {
        
        cv::DMatch m_ = match;

        cv::KeyPoint t_ = kp_b_[m_.trainIdx];
        cv::KeyPoint q_ = kp_a_[m_.queryIdx];

        if(train_vis_.count(t_) || query_vis_.count(q_)) {
            
            continue;
        }


        cnt_++;

        train_vis_.insert(t_);
        query_vis_.insert(q_);

        kp_a_mat_.push_back(kp_a_[match.queryIdx]);
        kp_b_mat_.push_back(kp_b_[match.trainIdx]);

    }
    
    //std::cout << "[frames] number of matching keypoints found: " << cnt_ << std::endl; 
    //std::cout << "[frames] train_vis_.size(): " << (int)train_vis_.size() << std::endl;
    //std::cout << "[frames] query_vis_.size(): " << (int)query_vis_.size() << std::endl;
    //std::cout << "[frames] kp_a_mat_.size(): " << (int)kp_a_mat_.size() << std::endl;
    //std::cout << "[frames] kp_b_mat_.size(): " << (int)kp_b_mat_.size() << std::endl;
    
}