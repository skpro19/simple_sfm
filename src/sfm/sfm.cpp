#include "../../include/sfm/sfm.hpp"

#include <opencv2/calib3d.hpp>

//#include <pcl/visualization/cloud_viewer.h>

simple_sfm::SimpleSFM::SimpleSFM(const std::string &base_folder_) 
{

    io_     =   std::make_shared<SFM_IO>(base_folder_);
   
    updateIOParams();

    //ceres::Problem::Options problem_options_;
    //ceres::Problem problem(problem_options_);

}

void simple_sfm::SimpleSFM::runBundleAdjust(){

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
        
    int n_ = 10; 

    int unused_cnt_ = 0 ;

    std::cout << "n_: " << n_ << std::endl;

    for(int i = 0 ; i < n_; i++){
        
        if(views_[i] == nullptr){

            continue;

        }
        
        std::shared_ptr<View> view_(views_[i]);
        
        assert(view_->pts_2d_->size() == view_->indices_3d_pts_->size());

        int m_ = (int)view_->pts_2d_->size();

        Mat3d intrinsics_ = view_->cam_intrinsics_;
        Vec6d &extrinsics_ = view_->cam_extrinsics_;

        std::cout << "index: " << i << std::endl;
        std::cout << "extrinsics_: " << extrinsics_ << std::endl;
        for(int j = 0 ; j < m_; j++) {
            
            float x_= view_->pts_2d_->at(j)[0];
            float y_= view_->pts_2d_->at(j)[1];

            Vec3d &pt_ = View::point_cloud_[view_->indices_3d_pts_->at(j)];
            
            ceres::CostFunction* cost_function = ReprojectionError::create(x_, y_, intrinsics_);
            
            problem.AddResidualBlock(cost_function, NULL, &extrinsics_(0), &pt_(0));

        }
    }

    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.use_inner_iterations = true;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    std::cout << "After BA" << std::endl;

    for(int i = 0; i < n_; i++) {

        if(views_[i] == nullptr) {continue;}
        std::shared_ptr<View> view_(views_[i]);

        cv::Matx44d ext_mat_ = unpackCameraExtrinsics(view_->cam_extrinsics_);
        Vis::updatePredictedPose(cv::Mat(ext_mat_));
        
        std::cout << "index: " << i << std::endl;
        std::cout << "extrinsics_: " << view_->cam_extrinsics_ << std::endl;


    }
}

void simple_sfm::SimpleSFM::updateIOParams() 
{

    io_->getImageFileNames(frame_list_);
    io_->getGTPoses(gt_poses_);
    
    P_prev_ = io_->getP0();
    K_  = io_->getK(); 

    cv::Matx41d t_ = io_->gett0();
    t_prev_ = cv::Matx31d(t_(0,0), t_(1, 0), t_(2,0));
    
    R_prev_ = io_->getR0();

}

void simple_sfm::SimpleSFM::update3DCloud(const std::vector<cv::Point3f> &pts_)
{
    std::cout << "pt_cld_3d_.size(): " << (int)pt_cld__3d_.size() << std::endl;

    int n_ = (int)pts_.size() ; 

    for(int i = 0 ;i < n_; i++) 
    {
        
        cv::Point3d p1_ = pts_[i]; 

        int m_ = (int)pt_cld__3d_.size(); 

        bool found_ = false;

        for(int j =0 ; j  < m_ ; j++) {

            cv::Point3d p2_ = pt_cld__3d_[j];

            double dis_ = cv::norm(p1_ - p2_);

            if(dis_  < 0.05) {

                found_ = true; 
                break;

            }

        }

        if(!found_) {

            pt_cld__3d_.push_back(p1_);

        } 

    }

    std::cout << "pt_cld_3d_.size(): " << (int)pt_cld__3d_.size() << std::endl;

}

void simple_sfm::SimpleSFM::runVOPipeline(){

    cv::Mat E_, R, t;
    cv::Mat R_f, t_f;

    cv::Mat C_k_, C_k_minus_1_;
    cv::Mat T_k_;

    R_f.convertTo(R_f, CV_64F);
    t_f.convertTo(t_f, CV_64F);

    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);    

    C_k_.convertTo(C_k_, CV_64F);
    C_k_minus_1_.convertTo(C_k_minus_1_, CV_64F);
    
    cv::Mat NO_ROT_ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat NO_T_ = cv::Mat::zeros(3, 1, CV_64F);
    
    cv::Mat TEMP_ = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1); 
    TEMP_.convertTo(TEMP_, CV_64F);
    
    //** Initializing C_k_minus_1 with 0 translation and 0 rotation w.r.t. the 'some' initial co-ordinate frame
    cv::hconcat(NO_ROT_, NO_T_, C_k_minus_1_);
    cv::vconcat(C_k_minus_1_, TEMP_, C_k_minus_1_);

    int last_idx_ = 0 ;

    int sz_ = frame_list_.size(); 

    views_.resize(sz_);

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    

    for(int i = 1 ; i < 10; i++) {

        std::cout << "i: " << i << std::endl;
    
        Frame::kp_1.resize(0); 
        Frame::kp_2.resize(0); 

        Frame::kp_1_matched.resize(0); 
        Frame::kp_2_matched.resize(0);
    
        cv::Mat img_1 = cv::imread(frame_list_[last_idx_].c_str());
        cv::Mat img_2 = cv::imread(frame_list_[i].c_str());

        Frame::extractAndMatchFeatures(img_1, img_2);

        assert((int)Frame::kp_1_matched.size() == (int)Frame::kp_2_matched.size());

        std::vector<cv::Point2d> kp_1f, kp_2f; //array of keypoint co-ordinates

        //std::cout << "i: " << i << " kp_1_matched.size(): " << (int)Frame::kp_1_matched.size() << std::endl;

        for(int k = 0; k < (int)Frame::kp_1_matched.size(); k++) {

            cv::Point2f p1_ = Frame::kp_1_matched[k].pt, p2_ = Frame::kp_2_matched[k].pt;
            
            kp_1f.push_back(p1_); 
            kp_2f.push_back(p2_);

            cv::circle( img_1, p1_, 2, cv::viz::Color::yellow(), -1 );
            cv::line(img_1, p1_, p2_, cv::viz::Color::pink());
            cv::circle( img_1, p2_, 2, cv::viz::Color::orange_red(), -1 );
        }
        
        //std::cout << "HI" << std::endl;
        //======== check for duplicate kps ==============

        bool dup_kp_flag_ = checkForDuplicates(kp_1f, kp_2f);
        
        if(!dup_kp_flag_) {continue;}

        //===============================================

        cv::Mat E_mask_;
        E_mask_.convertTo(E_mask_, CV_64F);

        E_ = cv::findEssentialMat(kp_2f, kp_1f, K_,cv::RANSAC, 0.999, 1.0, E_mask_);

        int inlier_cnt_ =0 ; 

        inlier_cnt_ = cv::recoverPose(E_, kp_2f, kp_1f, K_, R, t, E_mask_);

        //std::cout << "inlier_cnt_: " << inlier_cnt_ << std::endl;

        double scale_;

        double del_z_ = std::fabs(t.at<double>(2,0)); 
        double del_y_ = std::fabs(t.at<double>(1,0));
        double del_x_ = std::fabs(t.at<double>(0,0));
        
        
        if(inlier_cnt_ < 25) {continue;}

        bool duplicate_flag_ = checkForDuplicates(kp_1f, kp_2f);

        assert(duplicate_flag_);

        scale_ = Frame::getScale(cv::Mat(gt_poses_[i]), cv::Mat(gt_poses_[last_idx_])); 

        bool flag_ = ((scale_ > 0.1) &&  (del_z_ > del_x_) && (del_z_ > del_y_))  ; 
        
        std::cout << std::endl;

        if(!flag_) {continue;}
        

        cv::Mat temp_ = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1); 
        temp_.convertTo(temp_, CV_64F);

        T_k_.convertTo(T_k_, CV_64F);

        cv::hconcat(R, t, T_k_);
        cv::vconcat(T_k_, temp_, T_k_);

        C_k_ =  C_k_minus_1_ * T_k_;
     
        //  =============== extract inlier kps from E_mask_  =======================

        std::vector<cv::Point2d> kp_1f_in_, kp_2f_in_; //inlier kps_
        std::vector<int> v_(E_mask_);

        int mask_sum_ = std::accumulate(v_.begin(), v_.end(), 0);
        
        int v_sz_ = (int)v_.size() ; 

        assert(v_sz_ == E_mask_.size().height);
        assert((int)kp_1f_in_.size() == 0 && (int)kp_2f_in_.size() == 0);
    
        for(int vi = 0 ; vi < v_sz_ ; vi++) {

            int mask_ = v_[vi]; 

            if(mask_) {

                kp_1f_in_.push_back(kp_1f[vi]);
                kp_2f_in_.push_back(kp_2f[vi]);
            }

        }

        assert((int)kp_1f_in_.size() == mask_sum_);

        //  ================================================================================

        //  ====================    Triangulating 3d points from 2d  ======================
        
        cv::Matx34f P_k_, P_k_minus_1_;
        cv::Matx34f c_k_, c_k_minus_1_; 

        c_k_ = convert44to34Mat(C_k_);
        c_k_minus_1_ = convert44to34Mat(C_k_minus_1_);

        P_k_ = K_ * c_k_;
        P_k_minus_1_ = K_ * c_k_minus_1_;
        
        cv::Mat pts_4d_;

        cv::triangulatePoints(P_k_, P_k_minus_1_, kp_1f_in_, kp_2f_in_, pts_4d_);
        
        //std::cout << "kp_1f_in_.size(): " << (int)kp_2f_in_.size() << std::endl;
       
        std::vector<cv::Point3d> pts_3d_;
        
        convertPointsFromHomogeneous_(pts_4d_, pts_3d_);
        
        assert((int)pts_3d_.size() == (int)kp_2f_in_.size());
        
        // * ======================= VIEW PROCESSING  ===========================
    
        std::shared_ptr<View> view_ = std::make_shared<View>();
        
        view_->updateView(kp_2f_in_, pts_3d_, K_, C_k_);
        views_[i] = view_;

        // * ===================================================================

            
        
        //  ==================================================================================

        C_k_minus_1_ = C_k_;
        last_idx_ = i;

        std::cout <<  i << "--->[x y z]: " << "(" <<C_k_.at<double>(0, 3) << "," << C_k_.at<double>(1, 3) << "," << C_k_.at<double>(2,3) << ")" << std::endl; 
        
        

        //cv::imshow("img_1" , img_1);
        //Vis::drawKeyPoints(img_1, kp_1f, kp_2f);

        //Vis::updateGroundPose(cv::Mat(gt_poses_[i]));
        //Vis::updatePredictedPose(C_k_);

        cv::waitKey(10);
    }

    //std::cout << "pt_3d_cld_.size(): " << (int)pt_cld__3d_.size() << std::endl;

    //runBundleAdjust();  
}

