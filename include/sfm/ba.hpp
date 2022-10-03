#ifndef BA_H
#define BA_H 

#include <ceres/ceres.h>
#include <ceres/rotation.h>

//typedef Eigen::Matrix<double, 3, 3> Mat3d;

#include "sfm.hpp"

namespace simple_sfm{

    struct ReprojectionError{

        ReprojectionError(const double obs_x, const double obs_y, const Mat3d &intrinsics): obs_x_(obs_x), obs_y_(obs_y), intrinsics_(intrinsics){}

        template<typename T>
        bool operator()(const T* const extrinsics, //extrincs
                        const T* const X, //world point
                        T* residules) const
        {
            
            
            T x[3];
            ceres::AngleAxisRotatePoint(extrinsics, X, x);
            
            x[0] += extrinsics[3];
            x[1] += extrinsics[4];
            x[2] += extrinsics[5];
            
            T pred_x_ = intrinsics_(0,0) * x[0] / x[2] + intrinsics_(0,2);
            T pred_y_ = intrinsics_(1,1) * x[1] / x[2] + intrinsics_(1,2); 
            
            residules[0] = pred_x_ - T(obs_x_);
            residules[1] = pred_y_ - T(obs_y_);
            
            return true;

        }
        
        static ceres::CostFunction* create(const float observed_x,
                                       const float observed_y, const Mat3d intrinsics){
            
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
                    new ReprojectionError(observed_x, observed_y, intrinsics)));
        }


        const double obs_x_, obs_y_;
        const Mat3d intrinsics_;
    };


    
};


#endif 

