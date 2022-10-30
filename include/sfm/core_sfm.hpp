#ifndef CORE_SFM_H
#define CORE_SFM_H

// ========================================================
// =============   Core SFM functions     =================
// ========================================================

//#include "bookkeeping.hpp"
//#include "vis.hpp"
#include "frame.hpp"
#include "io.hpp"
#include "vis.hpp"

    
namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);

            void initializeSFM();

        private:


            void updateIOParams();            
            bool findCameraMatrices(cv::Matx34f &P1_ , cv::Matx34f &P2_, const Features &f1_, const Features &f2_, const Matches &matches_, Matches &pruned_matches_);
            bool triangulateViews(const cv::Mat &img_a_, 
                                const cv::Mat &img_b_, 
                                const cv::Matx34d &P1_, 
                                const cv::Matx34d &P2_, 
                                std::vector<CloudPoint3d> &pointcloud_);


            void visualizeCloudPointProjections(const cv::Matx34f &P1_, 
                                                const cv::Matx34f &P2_, 
                                                const std::vector<CloudPoint3d> &cloudpoints_);







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