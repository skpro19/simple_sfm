#ifndef SFM_H
#define SFM_H

// ========================================================
// =============   Core SFM functions     =================
// ========================================================

//#include "bookkeeping.hpp"
//#include "vis.hpp"
#include "frame.hpp"
#include "io.hpp"
#include "vis.hpp"

#include <ceres/cost_function.h>
#include <thread>
#include <mutex>
    
namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);


            void updateIOParams();            
            void runSFMPipeline();
            void createFeatureMatchMatrixThreaded();            

            void createFeatureMatrix();
            void createFeatureMatchMatrix();
            void initializeBaselineSFM();
            std::map<float, ImagePair> sortViewsByHomography();
            int getHomographyInliersCount(const Features &f1_, const Features &f2_, const Matches &matches_);
            bool findCameraMatrices(cv::Matx34d &P1_ , cv::Matx34d &P2_, const ImagePair &img_pair_, Matches &pruned_matches_);
            bool triangulateViews(const cv::Matx34d &P1_, const cv::Matx34d &P2_, const ImagePair &img_pair_, std::vector<CloudPoint3d> &pointcloud_);
            void addMoreViewsToReconstruction();
            bool getBestViewIndexToMerge(int &idx_);
            Match2D3D get2D3DMatches(const int idx_);
            bool updateCameraPoseFrom2D3DMatch(cv::Matx34d &camera_pose_, const Match2D3D &match2d3d_);
            void mergeNewPointCloud(std::vector<CloudPoint3d> &pointcloud_);
            

        private:

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
            
            //**debugging
            //std::vector<cv::KeyPoint> kp_1, kp_2;
            //std::vector<cv::KeyPoint> kp_1_matched, kp_2_matched; 
            void match_features(const cv::Mat &img_1, const cv::Mat &img_2);
            double getScale(int curr_idx_, int prev_idx_);

            int camera_cnt_ = 0 ;
            //std::vector<std::shared_ptr<View> > views_;
            
            //std::vector<std::shared_ptr<Mat34f> >cam_extrinsics_;
            
    };



};






#endif