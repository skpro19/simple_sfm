#include "../../include/sfm/debug.hpp"


namespace simple_sfm
{

    bool checkForDuplicates(const std::vector<cv::Point2f> &a_  , const std::vector<cv::Point2f> &b_){

        std::set<std::pair<float, float> > sa_, sb_; 

        assert((int)a_.size() == (int)b_.size());
        
        int n_ = (int)a_.size() ;

        for(int i = 0 ;i < n_;  i++) {

            std::pair<float, float> pa_, pb_; 
            
            pa_ = {a_[i].x, a_[i].y},  pb_ = {b_[i].x, b_[i].y}; 
            
            sa_.insert(pa_);
            sb_.insert(pb_);
        
        }

        return( ( (int)sa_.size() == a_.size() ) && ( (int)sb_.size() == (int)b_.size()));

    }

};