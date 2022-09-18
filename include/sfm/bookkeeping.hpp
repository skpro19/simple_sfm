#ifndef BKP_H
#define BKP_H 

//  ======================================================================
//  =============   Bookkeeping functions       ==========================
//  ======================================================================


#include "sfm_utility.hpp"
#include "frame.hpp"

#include <map>

namespace simple_sfm {

    
    class BookKeeping{

        public: 
            
            BookKeeping();

            
            
            
        private:

            

            //TODO ---> TUNE
            const int delta_ = 0.1;

    };

};

#endif