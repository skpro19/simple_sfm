#ifndef SFM_H
#define SFM_H


#include <string>
#include <iostream>
#include <vector>


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>




namespace simple_sfm
{   


    class SimpleSFM{

        public:

            //SimpleSFM(std::string &base_folder_);
            SimpleSFM(const std::string &folder_);

            void InitializeSFMPipeline();

            


        private:

              


    };






}; // namespace  




#endif