#ifndef UNIT_TESTS_H
#define UNIT_TESTS_H


#include <opencv2/opencv.hpp>

namespace simple_sfm{


    class UnitTesting{

        public: 

            static cv::Matx34f getProjectionMatrix(const cv::Mat &A_, const cv::Mat &R_, const cv::Mat &t_);

            static void printSize(const std::string &name_, const cv::Mat &mat_);   

            static void convertFromHomogeneous(const cv::Mat &x_hom_, cv::Mat &x_);

            static void convertToHomogeneous(const cv::Mat &mat_3d_, cv::Mat &mat_4d_);


        private:



    };


};



#endif