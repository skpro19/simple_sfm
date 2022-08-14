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

        private:

              
            
            //*** camera poses in curr frame and prev frame (origin is C_0) 
            cv::Matx44d C_k_, C_k_minus_1; 


            //** shared ptrs to other classes
            std::shared_ptr<VisualOdom> vo_; 
            std::shared_ptr<Vis> vis_;      
            
            

    };






}; // namespace  




#endif