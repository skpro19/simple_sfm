#include "../../include/visual_odom.hpp"


int main(){
    
    VisualOdom vo_("/home/skpro19/simple_sfm/");

    vo_.run_vo_pipeline();

    return 0;



}