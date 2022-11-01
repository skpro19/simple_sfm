#include "../../include/sfm/sfm_util.hpp"



cv::Matx34f simple_sfm::SfmUtil::getProjectionMatrix(const cv::Mat &A_, const cv::Mat &R_, const cv::Mat &t_){

    assert(A_.rows == 3 && A_.cols == 3);
    assert(R_.rows == 3 && R_.cols == 3); 
    assert(t_.rows == 3 && t_.cols == 1);

    cv::Mat P_; 

    cv::hconcat(R_, t_, P_);

    SfmUtil::printSize("P_: " , P_);
    SfmUtil::printSize("R_: " , R_);
    SfmUtil::printSize("t_: " , t_);
    SfmUtil::printSize("A_: " , A_);
    
    P_ = A_ * P_; 

    assert(P_.rows == 3 && P_.cols == 4);

    return cv::Matx34f(P_);

}

void simple_sfm::SfmUtil::startSeparator(const std::string &text){

    std::cout << std::endl;
    std::cout << "################## " << "START OF " << text << " ############" << std::endl;
    std::cout << std::endl;

}

void simple_sfm::SfmUtil::endSeparator(const std::string &text) {

    std::cout << std::endl;
    std::cout << "################## " << "END OF " << text << " ############" << std::endl;
    std::cout << std::endl;


}




void simple_sfm::SfmUtil::printSize(const std::string &name_, const cv::Mat &mat_){

    std::cout << name_ <<  " ---> (" << mat_.rows << ","  << mat_.cols << ")" << std::endl;

}

// x_hom_ ==> (3 * 2516)
void simple_sfm::SfmUtil::convertFromHomogeneous(const cv::Mat &x_hom_, cv::Mat &x_){

    int c_ = x_hom_.cols;
    int r_ = x_hom_.rows;
    
    //std::cout << "x_hom_.col(0): " << x_hom_.col(0) << std::endl;

    cv::convertPointsFromHomogeneous(x_hom_.t(), x_);

    //std::cout << "x_.row(0): " << x_.row(0) << std::endl;

    //printSize("x_", x_);
    
    float *data = (float*)x_.data;

    x_ = cv::Mat(cv::Size(r_ - 1,  c_), CV_32F, data);

    //printSize("x_" , x_);

    //std::cout << "x_.row(0): " << x_.row(0) << std::endl;

}

// mat_3d_ ==> (3 * 2516)
void simple_sfm::SfmUtil::convertToHomogeneous(const cv::Mat &mat_3d_, cv::Mat &mat_4d_){

    ///printSize("mat_3d_" , mat_3d_);

    int cols_ = mat_3d_.cols;
    int rows_ = mat_3d_.rows;

    //cv::Mat mat_4d_;
    cv::convertPointsToHomogeneous(mat_3d_.t(), mat_4d_);
    
    //std::cout << "mat_4d.size(): " << mat_4d_.rows << " , " << mat_4d_.cols << std::endl;

    //std::cout << "mat_4d_.row(0): " << mat_4d_.row(0) << std::endl;
    
    float *data = (float*)mat_4d_.data;

    mat_4d_ = cv::Mat(cv::Size(rows_ + 1,  cols_), CV_32F, data);

    //std::cout << "mat_4d.size(): " << mat_4d_.rows << " , " << mat_4d_.cols << std::endl;

    //std::cout << "mat_4d_.row(0): " << mat_4d_.row(0) << std::endl;

}


