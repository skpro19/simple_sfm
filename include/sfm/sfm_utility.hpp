#include "sfm.hpp"


namespace simple_sfm{

    cv::Matx34d getProjectionMatrix(const cv::Matx33d &K, const cv::Matx33d &R, cv::Matx31d t) {

        cv::Mat P; 
        cv::hconcat(R, t, P); 
        
        P = K * P; 
        return P;

    }

    //void 

}