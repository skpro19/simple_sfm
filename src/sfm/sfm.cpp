#include "../../include/sfm/sfm.hpp"




simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) {

    io_ = std::make_shared<IO>(base_folder_);

        

}



