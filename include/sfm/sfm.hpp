#ifndef SFM_H
#define SFM_H

// ========================================================
// =============   Core SFM functions     =================
// ========================================================

//#include "bookkeeping.hpp"
#include "sfm_utility.hpp"
#include "vis.hpp"
#include "io.hpp"
#include "frame.hpp"
#include "view.hpp"
#include "ba.hpp"
//#include "ds.hpp"

#include <ceres/cost_function.h>
#include <opencv2/core/eigen.hpp>



namespace simple_sfm{

    
    class View;

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);


            void updateIOParams();
            

            void initializeSFMPipeline();
            void runSFMPipeline();
            void addNextFrame(int frame_idx_);


            void runVOPipeline();

            void runBundleAdjust(int se_, int en_);
           // bool checkForDuplicates(const std::vector<cv::Point2f> &a_  , const std::vector<cv::Point2f> &b_) const;

            void update3DCloud(const std::vector<cv::Point3f> &pts_);
            
            void extractInliers( const std::vector<cv::Point2d> &kp_1f, 
                                            const std::vector<cv::Point2d> &kp_2f,
                                            std::vector<cv::Point2d> &kp_1f_in_, 
                                            std::vector<cv::Point2d> kp_2f_in_,  
                                            const cv::Mat &E_mask_);


           
        private:

            
            std::shared_ptr<SFM_IO>                     io_;
            
            std::vector<cv::String>                     frame_list_;

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

            std::vector<std::shared_ptr<View> > views_;
            
            std::vector<std::shared_ptr<Mat34f> >cam_extrinsics_;
            
    };



};






#endif