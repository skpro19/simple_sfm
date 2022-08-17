#ifndef SFM_H
#define SFM_H


#include "io.hpp"
#include "ds.hpp"



namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);

            void InitializeSFMPipeline();

            void updateIOParams();

        private:

            std::shared_ptr<SFM_IO>                     io_;
            std::vector<cv::String>                     image_file_list_;

            cv::Matx34f                                 P0_, P1_;
            cv::Matx33f                                 K_;
            cv::Matx33f                                 R0_, R1_;
            cv::Matx41f                                 t0_, t1_;


            cv::String                                  F0_, F1_;       //last and current frames

            Poses                                       gt_poses_;
    };



};






#endif