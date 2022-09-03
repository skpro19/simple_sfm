#ifndef SFM_H
#define SFM_H

#include "bookkeeping.hpp"

namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);


            void updateIOParams();
            

            void initializeSFMPipeline();
            void runSFMPipeline();
            void addNextFrame(int frame_idx_);


            //debugging functions
            void runVOPipeline();


          
        private:

            
            std::shared_ptr<SFM_IO>                     io_;
            std::shared_ptr<BookKeeping>                bkp_;
            
            std::vector<cv::String>                     image_file_list_;

            cv::Matx33d                                 K_;
            cv::Matx34d                                 P_prev_;
            cv::Matx33d                                 R_prev_;
            cv::Matx31d                                 t_prev_;
            cv::Matx34d                                 C_prev_;


            cv::String                                  F0_, F1_;       //last and current frames

            std::vector<cv::Matx34d>                    gt_poses_;
    };



};






#endif