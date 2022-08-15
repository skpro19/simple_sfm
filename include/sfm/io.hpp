#ifndef IO_H
#define IO_H


#include "sfm.hpp"

#include <sstream>
#include <fstream>

namespace simple_sfm{

    class IO {

        public: 

            IO(const std::string &base_folder_);

            void LoadDataFiles();

            void LoadProjectionMatrix(const std::string &calib_file_name_);

            void LoadCameraParamsMatrix();

            void LoadGTPoses(const std::string &gt_file_name_);

            void LoadImageFiles(const std::string &img_folder_name_);

        
        private:

            const std::string               calib_file_name_;
            const std::string               data_dir_;
            const std::string               base_dir_;    
            const std::string               gt_file_name_;
            const std::string               image_dir_;

            cv::Matx34f                     P0_;                            //initial projection matrix
            cv::Matx33f                     K0_;
            cv::Matx33f                     R0_;
            cv::Matx41f                     t0_;

            std::vector<cv::Matx34f>        gt_poses_;
            std::vector<cv::String>         image_file_names_;


    };



};

#endif