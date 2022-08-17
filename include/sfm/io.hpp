#ifndef IO_H
#define IO_H


// ========================================================
// ============= All IO related functions =================
// ========================================================

#include <string>
#include <iostream>
#include <vector>
#include <iostream>
#include <memory>
#include <sstream>
#include <fstream>



#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>


//#include "sfm.hpp"

namespace simple_sfm{

    class SFM_IO {

        public: 

            SFM_IO(const std::string &base_folder_);

            //getters
            cv::Matx33f getK()                                          const   {   return K_;  }
            cv::Matx34f getP0()                                         const   {   return P0_; }
            cv::Matx33f getR0()                                         const   {   return R0_; }
            cv::Matx41f gett0()                                         const   {   return t0_; }
            void getImageFileNames(std::vector<cv::String> &list_)      const   {   list_ = image_file_names_; }
            void getGTPoses(std::vector<cv::Matx34f> &list_)            const   {   list_ = gt_poses_; }

        private:

            //functions
            
            void LoadDataFiles();
            void LoadProjectionMatrix(const std::string &calib_file_name_);
            void LoadCameraParamsMatrix();
            void LoadGTPoses(const std::string &gt_file_name_);
            void LoadImageFiles(const std::string &img_folder_name_);

            
            //variables

            const std::string               calib_file_name_;
            const std::string               data_dir_;
            const std::string               base_dir_;    
            const std::string               gt_file_name_;
            const std::string               image_dir_;

            cv::Matx34f                     P0_;                            //initial projection matrix
            cv::Matx33f                     K_;
            cv::Matx33f                     R0_;
            cv::Matx41f                     t0_;

            std::vector<cv::Matx34f>        gt_poses_;
            std::vector<cv::String>         image_file_names_;


    };



};

#endif