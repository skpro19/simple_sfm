#ifndef SFM_H
#define SFM_H


#include "io.hpp"
#include "frame.hpp"
#include "bookkeeping.hpp"

namespace simple_sfm{

    /*struct compare {
        bool operator() (const Point2D& a_, const Point2D& b_) const {
            
            return ((a_.x < b_.x) ||((a_.x == b_.x) && (a_.y < b_.y))); // if x<y then x will come before y. Change this condition as per requirement
        
        }
    };*/

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

            std::set<Point2D, compare> s_curr_, s_last_; 
            

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