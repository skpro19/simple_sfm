#ifndef DF_H
#define DF_H

using Point2D       =       cv::Point2f;
using Point3D      =       cv::Point3f;

using Points2D      =       std::vector<Point2D> ;
using Points3D      =       std::vector<cv::Point3f> ;

using KeyPoints     =       std::vector<cv::KeyPoint>;
using Match         =       cv::DMatch; 
using Matches       =       std::vector<Match>;
using Poses         =       std::vector<cv::Mat>;






#endif