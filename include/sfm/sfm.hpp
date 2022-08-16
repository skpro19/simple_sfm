#ifndef SFM_H
#define SFM_H


#include <string>
#include <iostream>
#include <vector>
#include <iostream>
#include <memory>



#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

#include "io.hpp"


namespace simple_sfm
{   


    class SimpleSFM{

        public:

            SimpleSFM(const std::string &base_folder_);




        private:

            std::shared_ptr<IO> io_;
            

              


    };






}; // namespace  




#endif