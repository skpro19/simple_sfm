#ifndef VIS_H
#define VIS_H


//  ===========================================================================
//  =============   Visualization related functions  ==========================
//  ===========================================================================

#include "sfm.hpp"



namespace simple_sfm {

    class Vis {

        public: 

            Vis();

            static void drawKeyPoints(const KeyPoints &kp_a_, const KeyPoints &kp_b_);
            



        private:

            static cv::Mat mat_;



    };


};

#endif