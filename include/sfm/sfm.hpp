#ifndef SFM_H
#define SFM_H

// ========================================================
// =============   Core SFM functions     =================
// ========================================================

//#include "bookkeeping.hpp"
#include "sfm_utility.hpp"
#include "vis.hpp"
#include "debug.hpp"
#include "io.hpp"
#include "frame.hpp"

namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);


            void updateIOParams();
            

            void initializeSFMPipeline();
            void runSFMPipeline();
            void addNextFrame(int frame_idx_);


            void runVOPipeline();

           // bool checkForDuplicates(const std::vector<cv::Point2f> &a_  , const std::vector<cv::Point2f> &b_) const;

            
          
        private:

            
            std::shared_ptr<SFM_IO>                     io_;
            //std::shared_ptr<BookKeeping>                bkp_;
            
            std::vector<cv::String>                     frame_list_;

            cv::Matx33f                                 K_;
            cv::Matx34d                                 P_prev_;
            cv::Matx33d                                 R_prev_;
            cv::Matx31d                                 t_prev_;
            cv::Matx34d                                 C_prev_;



            std::vector<cv::Matx34d>                    gt_poses_;

            //**debugging
            //std::vector<cv::KeyPoint> kp_1, kp_2;
            //std::vector<cv::KeyPoint> kp_1_matched, kp_2_matched; 
            void match_features(const cv::Mat &img_1, const cv::Mat &img_2);
            double getScale(int curr_idx_, int prev_idx_);
    };



};






#endif