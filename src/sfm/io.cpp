#include "../../include/sfm/io.hpp"



simple_sfm::SFM_IO::SFM_IO(const std::string &base_folder_)
                                                    :calib_file_name_("calib.txt")
                                                    ,base_dir_(base_folder_)
                                                    ,data_dir_(base_folder_ + "data/00/")
                                                    ,gt_file_name_("00.txt")
                                                    ,image_dir_("image_0/")
{

    std::cout << "[io]: Inside IO constructor!" << std::endl;
    LoadDataFiles();

}

void simple_sfm::SFM_IO::LoadDataFiles()
{

    std::cout << "[io] Inside LoadDataFiles function!"  << std::endl;

    LoadProjectionMatrix(calib_file_name_);
    LoadCameraParamsMatrix();
    LoadGTPoses(gt_file_name_);
    LoadImageFiles(image_dir_);

}

void simple_sfm::SFM_IO::LoadProjectionMatrix(const std::string &calib_file_name_)
{

    std::cout << "[io]: Inside the LoadProjectionMatrix function!" << std::endl;

    std::string calib_file_ = data_dir_ + calib_file_name_;
    
    std::cout << "[io]  calib_file_path_: " << calib_file_ << std::endl;


    std::ifstream calib_;

    calib_.open(calib_file_);
    
    if(calib_.is_open()) {

        std::string line_;
        std::getline(calib_, line_);

        //std::cout << "line_: " << line_ << std::endl;

        std::string s_;
        float f_;

        std::stringstream ss_(line_);

        ss_ >> s_; // bypass "P0:""

        //std::cout << "s_: " << s_ << std::endl;

        std::vector<float> v_;

        int cnt_ = 12;
        while(ss_ >> f_ && cnt_ > 0) {

            v_.push_back(f_);
            cnt_--;

        }

        P0_ = cv::Mat(v_).reshape(0, 3);

        std::cout << "[io]  P0_: " << P0_ << std::endl;

        
    }


    else {

        std::cerr << "Unable to read calib file!" << std::endl;

    }

    //cv::FileStorage fs_(calib_file_, cv::FileStorage::READ);

}


void simple_sfm::SFM_IO::LoadCameraParamsMatrix()
{

    std::cout << "[io] Inside the LoadCameraParamsMatrix function!" << std::endl;
    cv::decomposeProjectionMatrix(P0_, K_, R0_, t0_);

    std::cout << "[io]  K_: " << K_ << std::endl;
    std::cout << "[io]  R0_: " << R0_ << std::endl;
    std::cout << "[io]  t0_: " << t0_ << std::endl;

    
}



void simple_sfm::SFM_IO::LoadGTPoses(const std::string &gt_file_name_)
{

    std::cout << "[io] Inside the LoadGTPoses function!" << std::endl;

    std::string gt_file_ = data_dir_ + gt_file_name_;
    
    std::ifstream gt_;

    int cnt = 1; 

    gt_.open(gt_file_);
    
    if(gt_.is_open()) {

        std::string line_;

        while(std::getline(gt_, line_) ){

            //float f_;
            double f_;
            std::stringstream ss_(line_);
            std::vector<float> v_; 

            for(int i = 0; i < 12 ; i++) {

                ss_ >> f_; 
                v_.push_back(f_);

            }
            
            cv::Matx34f gt_;
            gt_ = cv::Mat(v_).reshape(0, 3);
            //gt_.convertTo(gt_, CV_64F);

            gt_poses_.push_back(gt_);

            std::cout << "[io]  gt_: " << gt_ << std::endl;

        }


    }



    else {

        std::cerr << "[io]  Unable to read calib file!" << std::endl;

    }

}



void simple_sfm::SFM_IO::LoadImageFiles(const std::string &img_folder_name_)
{

    std::string image_folder_ = data_dir_ + img_folder_name_;

    cv::glob(image_folder_, image_file_names_, false);

    
}