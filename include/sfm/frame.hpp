#ifndef FRAME_H
#define FRAME_H


//  ======================================================================
//  =============   Frame processing functions  ==========================
//  ======================================================================


#include "ds.hpp"

#include <numeric>

using KP = KeyPoints;

namespace simple_sfm{

    class Frame{

        public:

            Frame();

            static void extractAndMatchFeatures(const cv::Mat &img_1, const cv::Mat &img_2);    
            static double getScale(const cv::Mat &prev_poses_, const cv::Mat &curr_poses_) ;
            
            static inline std::vector<cv::KeyPoint> kp_1, kp_2;
            static inline std::vector<cv::KeyPoint> kp_1_matched, kp_2_matched; 
            
            
            

            static Features extractFeaturesAndDescriptors(const cv::Mat &img_);
            static void keypointsToPoints(Features &f_);
            static Matches getMatches(const Features &f1_, const Features &f2_);

            static int getHomographyInliersCount(const Features &f1_, const Features &f2_, const Matches &matches_);
            
            static void alignFeaturesUsingMatches(  const Features &f1_, const Features &f2_, 
                                                    Features &f1_mat_, Features &f2_mat_, 
                                                    std::vector<int> &ref_f1_, std::vector<int> &ref_f2_,
                                                    const Matches &matches_) ;
        private:

            static inline cv::Ptr<cv::ORB>  orb_ = cv::ORB::create(5000);
            static inline cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
            
            
    } ;   



};




#endif