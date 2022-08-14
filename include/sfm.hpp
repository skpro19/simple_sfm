#ifndef SFM_H
#define SFM_H

/*

- Miscellaneous
- Debugging strategy
- Visualizations 
- viz
- PCL visualizer

- Data Structures
- Observation
-  View -> image + camera pose + observations
- 3D Points

- SFM Pipeline
- Initialization
    - See initialization tips from Whatsapp
    - Estimate camera pose and triangulate 
- PnP matching
    - Provide some 2D-3D correspondances
    - Estimate camera pose and triangulate
- Bundle Adjustment

- Visualizations

*/

//#define _GLIBCXX_USE_CXX11_ABI 0

#include <string>
#include "visual_odom.hpp"
#include "vis.hpp"

namespace simple_sfm
{   


   

    

    class SimpleSFM{

        public:

            //SimpleSFM(std::string &base_folder_);
            SimpleSFM(const std::string &folder_);

            void runSFMPipeline();

            void initalizePipeline(); 

            bool processNextFrame(const int &curr_frame_id_);

            void updateCameraPose(cv::Matx44d &pose_);

            void displayCameraPose(const cv::Matx44d &pose_);

            void triangulate3DPoints(std::vector<cv::Matx41d> &points3d_);

            
        private:

              

            cv::Matx44d C_0, C_1;   //camera poses in curr frame and prev frame --> origin is C_1 (at i==0) 
            //cv::Matx34d P_0, P_1;   //camera projection matrices
            cv::Matx33d K_;         //camera intrinsics

            

            //** shared ptrs to other classes
            std::shared_ptr<VisualOdom> vo_; 
            std::shared_ptr<Vis> vis_;      
            
            

    };






}; // namespace  




#endif