#ifndef SFM_H
#define SFM_H


#include "io.hpp"
#include "frame.hpp"
#include "bookkeeping.hpp"


namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);


            void updateIOParams();
            void runVOPipeline();


            void InitializeSFMPipeline();
            void runSFMPipeline();
            void addNextFrame(int frame_idx_);

            




        private:

            std::shared_ptr<SFM_IO>                     io_;
            std::shared_ptr<BookKeeping>                bkp_;
            
            std::vector<cv::String>                     image_file_list_;

            cv::Matx34d                                 P0_, P1_;
            cv::Matx33d                                 K_;
            cv::Matx33d                                 R0_, R1_;
            cv::Matx41d                                 t0_, t1_;


            cv::String                                  F0_, F1_;       //last and current frames

            std::vector<cv::Matx34d>                    gt_poses_;
    };



};






#endif