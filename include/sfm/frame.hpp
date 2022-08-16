#ifndef IMGPROCESS_H
#define IMGPROCESS_H


//  ======================================================================
//  =============   Frame processing functions  ==========================
//  ======================================================================


#include "io.hpp"
#include "ds.hpp"

using KP = KeyPoints;

namespace simple_sfm{

    class Frame{

        public:

            Frame();

            static void ExtractKeyPoints(const cv::String &img_a_, const cv::String &img_b_, KP &kp_1, KP &kp_2);  
            
            static void GetGoodMatches(const cv::String &img_a_, const cv::String &img_b_, KP &kp_a_, KP &kp_b_, Matches &good_matches_);
            
            static void ExtractMatchingKeyPoints(KP &kp_a_, KP &kp_b_, Matches good_matches_, KP &kp_a_mat_, KP &kp_b_mat_);
            

        private:

            static cv::Ptr<cv::ORB> orb_;





    } ;   



};




#endif