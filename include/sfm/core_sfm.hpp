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
#include "sfm_util.hpp"
#include "sfm_helpers.hpp"

    
namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);

            void initializeSFM();

        private:


            void updateIOParams();            
           




            std::vector<Features>                   mFeatures_;
            std::vector<std::vector<Matches> >      mMFeatureMatches_;
            std::vector<cv::String>                 mFrames_;
            std::vector<cv::Matx34d>                mCameraPoses_;

            std::vector<CloudPoint3d>               mPointCloud_;
            std::set<int>                           mGoodViews_;
            std::set<int>                           mDoneViews_;


            
            std::shared_ptr<SFM_IO>                     io_;
            
            
            cv::Matx33f                                 K_;
            cv::Matx34d                                 P_prev_;
            cv::Matx33d                                 R_prev_;
            cv::Matx31d                                 t_prev_;
            cv::Matx34d                                 C_prev_;


            std::vector<cv::Matx34d>                    gt_poses_;
            std::vector<cv::Point3f>                    pt_cld__3d_;
        
            
    };



};






#endif