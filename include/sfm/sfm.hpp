#ifndef SFM_H
#define SFM_H


#include "io.hpp"



namespace simple_sfm{

    class SimpleSFM{

        public: 

            SimpleSFM(const std::string &base_folder_);


        private:

            std::shared_ptr<SFM_IO> io_;



    };



};






#endif