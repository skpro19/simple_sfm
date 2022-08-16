#include "../../include/sfm/sfm.hpp"


simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) {

    io_ = std::make_shared<SFM_IO>(base_folder_);

    
}

void simple_sfm::SimpleSFM::updateIOParams() {

    io_->getImageFileNames(image_file_list_);
    
    P0_ = io_->getP0();
    K_  = io_->getK(); 
    t0_ = io_->gett0();
    R0_ = io_->getR0();

}

void simple_sfm::SimpleSFM::InitializeSFMPipeline() {

    



}

