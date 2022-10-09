#ifndef FRAME_H
#define FRAME_H


//  ======================================================================
//  =============   Frame processing functions  ==========================
//  ======================================================================


#include "io.hpp"
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
            
            
            

            static void extractFeaturesAndDescriptors(const cv::Mat &img_, Features &features_);
            static void keypointsToPoints(Features &f_);
            static Matches getMatches(const Features &f1_, const Features &f2_);

            


        private:

            //static inline cv::Ptr<cv::ORB>  orb_ = cv::ORB::create(5000);

            //TODO ----> TUNE
            static inline int multiplier_ = 10;
            
            //static std::set<int>  train_set_, query_set_;

    } ;   



};




#endif