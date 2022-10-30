#include "../../include/sfm/core_sfm.hpp"

//#include <opencv2/core/check.hpp>


void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(mFrames_);
    io_->getGTPoses(gt_poses_);
    
    P_prev_ = io_->getP0();
    K_  = io_->getK(); 

    cv::Matx41d t_ = io_->gett0();
    t_prev_ = cv::Matx31d(t_(0,0), t_(1, 0), t_(2,0));
    
    R_prev_ = io_->getR0();

}

//TODO --> check alignFeaturesUsingMatches for bug ; add inlier ratio check ; tune inlier ratio condition
bool simple_sfm::SimpleSFM::findCameraMatrices(cv::Matx34f &P1_ , cv::Matx34f &P2_, const Features &f1_, const Features &f2_, const Matches &matches_, Matches &pruned_matches_){


    std::cout << "########################################################################" << std::endl;


    cv::Mat E_, R, t, inlier_mask_;

    
    Features f1_mat_, f2_mat_; 
    std::vector<int> ref_f1_, ref_f2_;

    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_, ref_f1_, ref_f2_, matches_);
    
    const Points2d &pts1_ = f1_mat_.points;
    const Points2d &pts2_ = f2_mat_.points;


    std::cout << "pts1_.size(): " << pts1_.size() << " pts2_.size(): " << pts2_.size() << std::endl;
    
    E_ = cv::findEssentialMat(pts1_, pts2_, K_,cv::RANSAC, 0.999, 1.0, inlier_mask_);
    
    //std::cout << "E_: " << E_ << std::endl;

    const int inlier_cnt_ = cv::recoverPose(E_, pts1_, pts2_, K_, R, t, inlier_mask_);

    std::cout << "R_: " << R << " t: " << t << std::endl;

    std::cout << "inlier_cnt_: " << inlier_cnt_ << std::endl;

    if(inlier_cnt_ < POSE_INLIERS_MINIMAL_COUNT) { return false;}

    P1_ = cv::Matx34d::eye();

    P2_ = cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    pruned_matches_.resize(0);

    //const Matches &matches_ = mMFeatureMatches_[img_pair_.first][img_pair_.second];
        
    for(int i = 0 ; i < inlier_mask_.rows; i++) {

        if(inlier_mask_.at<uchar>(i)){

            pruned_matches_.push_back(matches_[i]);

        }
    }   

   std::cout << "########################################################################" << std::endl;

    return true;
   
}

bool simple_sfm::SimpleSFM::triangulateViews(const cv::Mat &img_a_, const cv::Mat &img_b_, const cv::Matx34d &P1_, const cv::Matx34d &P2_, std::vector<CloudPoint3d> &pointcloud_){

    
    
    const Features &f1_ = Frame::extractFeaturesAndDescriptors(img_a_); 
    const Features &f2_ = Frame::extractFeaturesAndDescriptors(img_b_); 
    
    const Matches &matches_ = Frame::getMatches(f1_, f2_);

    Features f1_mat_, f2_mat_;
    std::vector<int> ref_f1_, ref_f2_;
    Frame::alignFeaturesUsingMatches(f1_, f2_, f1_mat_, f2_mat_,ref_f1_, ref_f2_,  matches_);

    
    cv::Mat normalized_pts_1_, normalized_pts_2_;

    cv::undistortPoints(f1_mat_.points, normalized_pts_1_, K_, cv::Mat());
    cv::undistortPoints(f2_mat_.points, normalized_pts_2_, K_, cv::Mat());
    
    
    cv::Mat pts_4d_;
    cv::triangulatePoints(P1_, P2_, normalized_pts_1_, normalized_pts_2_, pts_4d_);

    
    cv::Mat pts_3d_;
    cv::convertPointsFromHomogeneous(pts_4d_.t(), pts_3d_);
    
    
    cv::Mat rvec1_;
    cv::Rodrigues(P1_.get_minor<3, 3>(0,0), rvec1_);
    cv::Mat tvec1_(P1_.get_minor<3,1>(0,3).t());
    

    cv::Mat rvec2_;
    cv::Rodrigues(P2_.get_minor<3, 3>(0,0), rvec2_);
    cv::Mat tvec2_(P2_.get_minor<3,1>(0,3).t());

    
    Points2d projected_pts_1_, projected_pts_2_;
    
    cv::projectPoints(pts_3d_, rvec1_, tvec1_, K_, cv::Mat(), projected_pts_1_);
    cv::projectPoints(pts_3d_, rvec2_, tvec2_, K_, cv::Mat(), projected_pts_2_);

    int rejected_3d_pts_cnt_ = 0 ;
    for(int k = 0; k < pts_3d_.rows; k++){

        double d1_ = cv::norm(projected_pts_1_[k] - f1_mat_.points[k]);
        double d2_ = cv::norm(projected_pts_2_[k] - f2_mat_.points[k]);

        if(d1_ > MIN_REPROJECTION_ERROR or d2_ > MIN_REPROJECTION_ERROR){
            
            rejected_3d_pts_cnt_++;
            continue;

        }

        CloudPoint3d cloud_pt_;
        cloud_pt_.point_ = cv::Point3d{pts_3d_.at<double>(k,0), pts_3d_.at<double>(k,1), pts_3d_.at<double>(k,2)};

        //cloud_pt_.viewMap[i] = ref_f1_[k];
        //cloud_pt_.viewMap[j] = ref_f2_[k];
        
        pointcloud_.push_back(cloud_pt_);
        

    }
    
    return true;
    
}

void printSize(const std::string &name_, const cv::Mat &mat_){

    std::cout << name_ <<  " ---> (" << mat_.rows << ","  << mat_.cols << ")" << std::endl;

}

// x_hom_ ==> (3 * 2516)
void convertFromHomogeneous(const cv::Mat &x_hom_, cv::Mat &x_){

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
void convertToHomogeneous(const cv::Mat &mat_3d_, cv::Mat &mat_4d_){

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



void simple_sfm::SimpleSFM::visualizeCloudPointProjections(const cv::Matx34f &P1_, 
                                                            const cv::Matx34f &P2_, 
                                                            const std::vector<CloudPoint3d> &pointcloud_,
                                                            const cv::Mat &img_a_){

    std::cout << "##############" << std::endl;
    std::vector<cv::Point3f> v_; 
    for(const auto &t: pointcloud_) v_.push_back(t.point_);


    
    cv::Mat mat_4d_, mat_3d_ = cv::Mat(v_).reshape(1).t();
    convertToHomogeneous(mat_3d_, mat_4d_);
    
    cv::Mat x_,  x_hom_ = cv::Mat(P2_) * mat_4d_.t();
    convertFromHomogeneous(x_hom_ , x_);   // x_ ===> (2516 * 2) , x_hom_ ==> (3 * 2516)

    
    std::cout << "###########" << std::endl;
    //Vis::draw2DPoints(x_);
    
    //cv::Mat gt_mat_ = cv::Mat::zeros(376, 1241, CV_8UC3);

    //for(int i = 0 ; i < 50; i++) std::cout << x_.row(i) << std::endl;

    //std::cout << "%%%%" << std::endl;

    for(int i = 0 ; i < x_.rows; i++) {
        
        //std::cout << x_.row(i) << std::endl;
        cv::circle(img_a_, {x_.at<float>(i,0), x_.at<float>(i,1)} ,1, cv::viz::Color::celestial_blue(), 1);
        
    }

    cv::imshow("gt_", img_a_);
    cv::waitKey(0);


}
