#ifndef BA_H
#define BA_H 

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace simple_sfm{

    struct ReprojectionError{

        ReprojectionError(const double obs_x, const double obs_y, const Mat3f &intrinsics): obs_x_(obs_x), obs_y_(obs_y), intrinsics_(intrinsics){}

        template<typename T>
        bool operator()(const T* const extrinsics, //extrincs
                        const T* const point, //world point
                        T* residules) const
        {
            
            
            Vec3f RX_, t_, p_;
            ceres::AngleAxisRotatePoint(extrinsics, point, RX_);
            
            t_ << extrinsics[3] , extrinsics[4] , extrinsics[5];
            
            p_ = RX_ + t_;  // p_ =  R_ * point_ + t_;

            Vec3f pred_ = intrinsics_ * p_;     // x = K[R|t] * X


            T pred_x_ = pred_[0]/pred_[2];  
            T pred_y_ = pred_[1]/pred_[2]; 
            

            residules[0] = pred_x_ - T(xo_);
            residules[1] = pred_y_ - T(yo_);
            
            return true;

        }
        
        static ceres::CostFunction* create(const float observed_x,
                                       const float observed_y, const Mat3f intrinsics){
            
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
                    new ReprojectionError(observed_x, observed_y, intrinsics)));
        }


        const double obs_x_, obs_y_;
        const Mat3f intrinsics_;
    };



    





};




#endif 

