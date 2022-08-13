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


namespace simple_sfm
{

    class SimpleSFM{

        public:

            //SimpleSFM(std::string &base_folder_);
            SimpleSFM(const std::string &folder_);

                    
        private:

            //void  run_sfm_pipeline();

           std::shared_ptr<VisualOdom> vo_;           


    };






}; // namespace  




#endif