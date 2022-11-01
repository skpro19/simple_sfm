#ifndef SFM_HELPERS_H
#define SFM_HELPERS_H


#include <opencv2/opencv.hpp>

#include "ds.hpp"
#include "frame.hpp"
#include "sfm_util.hpp"
#include "vis.hpp"


namespace simple_sfm{


    class SfmHelper{

        public: 

            static bool findCameraPoseMatrices(cv::Matx34f &P1_,
                                                cv::Matx34f &P2_,
                                                const Features &f1_,
                                                const Features &f2_,
                                                const Matches &matches_,
                                                const cv::Matx33f &K_,
                                                Matches &pruned_matches_);
            

            static bool triangulateViews(const Features &f1_, 
                                        const Features &f2_, 
                                        const int &frame_idx_,
                                        const cv::Matx34d &P1_, 
                                        const cv::Matx34d &P2_, 
                                        const cv::Matx33f &K_,
                                        std::vector<CloudPoint3d> &pointcloud_,
                                        const Matches &pruned_matches_);

           static Match2D3D get2D3DMatches(const int view_idx_, 
                                            const std::vector<CloudPoint3d> &pointcloud_,
                                            const Features &f1_,
                                            const Features &f2_,
                                            const Matches &pruned_matches_);

            

            static void visualizeCloudPointProjections(const cv::Matx34f &C1_, 
                                                            const cv::Matx34f &C2_, 
                                                            const std::vector<CloudPoint3d> &pointcloud_,
                                                            const cv::Matx33f &K_,
                                                            const cv::Mat &base_img_);

            static void projectPCLOnFrameIdx(const int frame_idx_, 
                                                const std::vector<cv::Matx34f> &mCameraPoses_,
                                                const std::vector<cv::String> mFrames_, 
                                                const std::vector<CloudPoint3d> &pointcloud_,
                                                const cv::Matx33f &K_);



        private:



    };


};



#endif