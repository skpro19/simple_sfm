#include "../../include/sfm/sfm.hpp"
//#include "../../include/sfm/io.hpp"


int main(){

    const std::string base_folder_ = "/home/skpro19/simple_sfm/";

    
    simple_sfm::SimpleSFM sfm_(base_folder_);   
    
    sfm_.runVOPipeline();
    
    
}