#ifndef BA_H
#define BA_H 

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace simple_sfm{

    struct ReprojectionError{

        ReprojectionError(const double xo, const double yo): xo_(xo), yo_(yo){}

        template<typename T>
        bool operator()(const T* const intrinsics, //intrinsics
                        const T* const extrinsics, //extrincs
                        const T* const point, //world point
                        T* residules) const
        {
            
            const T& fx_                = intrinsics[0];
            const T& fy_                = intrinsics[4];
            const T& cx_                = intrinsics[2];
            const T& cy_                = intrinsics[5];
            
            T x[3];
            ceres::AngleAxisRotatePoint(extrinsics, point, x);
            x[0] += extrinsics[3];
            x[1] += extrinsics[4];
            x[2] += extrinsics[5];

            T pred_x_; 
            T pred_y_; 
            T pred_z_; 
            

            pred_x_ = T(fx_) * T(point[0]) + T(cx_) * T(point[2]);
            pred_y_ = T(fy_) * T(point[1]) + T(cy_) * T(point[2]);
            pred_z_ = T(point[2]);
            
            pred_x_ = T(pred_x_)/T(pred_z_);
            pred_y_ = T(pred_y_)/T(pred_z_);
            
            residules[0] = pred_x_ - T(xo_);
            residules[1] = pred_y_ - T(yo_);
            
            return true;

        }
        
        static ceres::CostFunction* create(const float observed_x,
                                       const float observed_y){
            
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 6, 3>(
                    new ReprojectionError(observed_x, observed_y)));
        }


        double xo_, yo_;



    };



    





};




#endif 

