#ifndef IMGPROCESS_H
#define IMGPROCESS_H

#include "io.hpp"

using Points2D      =   std::vector<cv::Point2f> ;
using KeyPoints     =   std::vector<cv::KeyPoint>;


struct Feature{

    KeyPoints kps_;


};

namespace simple_sfm{


    class Frame{

        public:

            Frame();

            void GetMatchingPoints2D(const cv::String &img_a_, const cv::String &img_b_, Points2D &p_a_, Points2D &p_b_);        

            void ExtractFeatures(const cv::String &img_a_, const cv::String &img_b_);   

            void GetGoodFeatures();         



        private:







    }    



};




#endif