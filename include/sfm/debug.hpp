#ifndef SFM_DEBUG_H
#define SFM_DEBUG_H

// ================================================================
// =============   Debugging helper functions     =================
// ================================================================


#include "ds.hpp"

namespace simple_sfm{

    bool checkForDuplicates(const std::vector<cv::Point2f> &a_  , const std::vector<cv::Point2f> &b_);



};

#endif