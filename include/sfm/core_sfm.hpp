#ifndef CORE_SFM_H
#define CORE_SFM_H

// ========================================================
// =============   Core SFM functions     =================
// ========================================================

//#include "bookkeeping.hpp"
//#include "vis.hpp"
//#include "frame.hpp"
#include "io.hpp"
#include "vis.hpp"
#include "sfm_helpers.hpp"
#include "sfm_tests.hpp"
    
namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);

            void initializeSFM();

        private:


            void updateIOParams();   
            void addView(const int frame_idx_);         
           

            //** core vars            
            std::vector<CloudPoint3d>               globalPointCloud_;
            std::vector<CloudPoint3d>               lastPointCloud_;

            

            //** non-core vars                   
            std::vector<cv::String>                 mFrames_;
            std::vector<cv::Matx34f>                mCameraPoses_;

            
            std::shared_ptr<SFM_IO>                     io_;
            cv::Matx33f                                 K_;
            std::vector<cv::Matx34d>                    gt_poses_;
        
            
    };



};






#endif