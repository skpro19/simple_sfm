//#include "../../include/sfm/frame.hpp"
//#include "../../include/sfm/sfm_utility.hpp"

#include "../../include/sfm/sfm.hpp"

#include <opencv2/viz/types.hpp>

simple_sfm::Frame::Frame() {
    
    //Frame::orb_ = cv::ORB::create(5000);
    //Frame::matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

}

void simple_sfm::Frame::alignFeaturesUsingMatches(  const Features &f1_, const Features &f2_, 
                                                    Features &f1_mat_, Features &f2_mat_, 
                                                    std::vector<int> &ref_f1_, std::vector<int> &ref_f2_,
                                                    const Matches &matches_) {

     
    f1_mat_.keypoints.resize(0); 
    f2_mat_.keypoints.resize(0); 

    f1_mat_.points.resize(0); 
    f2_mat_.points.resize(0); 

    f1_mat_.descriptors = cv::Mat();
    f2_mat_.descriptors = cv::Mat();
    
    ref_f1_.resize(0);
    ref_f2_.resize(0);

    //std::cout << "f1_.size(): " << f1_.keypoints.size() << std::endl;

    for(int i = 0 ;i  < (int)matches_.size(); i++) {

        f1_mat_.keypoints.push_back(f1_.keypoints[matches_[i].queryIdx]);
        f2_mat_.keypoints.push_back(f2_.keypoints[matches_[i].trainIdx]);
        
        f1_mat_.points.push_back(f1_.points[matches_[i].queryIdx]);
        f2_mat_.points.push_back(f2_.points[matches_[i].trainIdx]);

        f1_mat_.descriptors.push_back(f1_.descriptors.row(matches_[i].queryIdx));
        f2_mat_.descriptors.push_back(f2_.descriptors.row(matches_[i].trainIdx));

        ref_f1_.push_back(matches_[i].queryIdx);
        ref_f2_.push_back(matches_[i].trainIdx);

    }

   // std::cout  <<"exiting alignFeaturesUsingMatches!" << std::endl;

}

int simple_sfm::Frame::getHomographyInliersCount(const Features &f1_, const Features &f2_, const Matches &matches_){

    Features f1_mat_, f2_mat_;
    std::vector<int> ref_f1_, ref_f2_;
    
    //std::cout << "f1_.size(): " << f1_.keypoints.size() << std::endl;
    //std::cout << "f1_.size(): " << f1_.points.size() << std::endl;
    //std::cout << "f1_.size(): " << f1_.descriptors.size() << std::endl;

    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_, ref_f1_, ref_f2_, matches_);

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


Matches simple_sfm::Frame::getMatches(const Features &f1_, const Features &f2_){

    //std::cout << "f1_.d.size(): " << f1_.descriptors.size() << " f2_.d.size(): " << f2_.descriptors.size() << std::endl;
    //std::cout << "f1_.k.size(): " << f1_.keypoints.size() << " f2_.k.size(): " << f2_.keypoints.size() << std::endl;
    //std::cout << "f1_.p.size(): " << f1_.points.size() << " f2_.p.size(): " << f2_.points.size() << std::endl;
    
    //cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::vector<Matches> brute_hamming_matches;
    
    Frame::matcher->knnMatch(f1_.descriptors, f2_.descriptors, brute_hamming_matches,  2);

    //std::cout << "H1" << std::endl;
    //std::cout << "matches.size(): " << brute_hamming_matches.size() << std::endl;

    Matches good_matches_;
    
    for(unsigned i = 0; i < brute_hamming_matches.size(); i++) {
        if(brute_hamming_matches[i][0].distance < 0.8 * brute_hamming_matches[i][1].distance) {
            good_matches_.push_back(brute_hamming_matches[i][0]);
        }
    }

    return good_matches_;

}


void simple_sfm::Frame::keypointsToPoints(Features &f_){

    f_.points.resize(0);

    for(const auto &kp_: f_.keypoints) {

        f_.points.push_back(kp_.pt);
    }

    assert(f_.keypoints.size() == f_.points.size());

}


Features simple_sfm::Frame::extractFeaturesAndDescriptors(const cv::Mat &img_){

    Features features_;    
    features_.descriptors.resize(0);
    features_.keypoints.resize(0);
    features_.points.resize(0);

    cv::Mat image_;
    cv::cvtColor(img_, image_, cv::COLOR_BGR2GRAY);
    
    cv::Mat mask = cv::Mat();
    
    //cv::Ptr<cv::ORB>orb_ = cv::ORB::create(5000);
    Frame::orb_ = cv::ORB::create(5000);

    Frame::orb_->detectAndCompute(image_, mask, features_.keypoints, features_.descriptors);

    Frame::keypointsToPoints(features_);
    
    return features_;
}


