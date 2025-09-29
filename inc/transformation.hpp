#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP

#include <opencv2/opencv.hpp>
#include "motion.hpp"

namespace
{
    //相机内参矩阵
    cv::Mat K = (cv::Mat_<double>(3,3) << 
                    1.7774091341308808e+03, 0., 7.1075979428865026e+02, 
                    0.,1.7754170626354828e+03, 5.3472407285624729e+02, 
                    0., 0., 1.);
    //多项式畸变系数:k1,k2,p1,p2,k3
    cv::Mat dicoef = (cv::Mat_<double>(1,5) << 
                    -5.6313426428564950e-01, 1.8301501710641366e-01, 
                    1.9661478907901904e-03, 9.6259122849674621e-04,
                    5.6883803390679100e-01);
    //PnP解算对象三维坐标
    std::vector<cv::Point3d> objectPoints = {
                    {-65.0, -55.0, 0.0},   //左上
                    {-65.0, 0.0, 0.0},    //左中
                    {-65.0, 55.0, 0.0},  //左下
                    {65.0, -55.0, 0.0},   //右上
                    {65.0, 0.0, 0.0},     //右中
                    {65.0, 55.0, 0.0}   }; //右下

    //gimbal to world坐标系转换矩阵
    cv::Mat getRotationMatrixG2W()
    {
        cv::Mat R_yaw = (cv::Mat_<double>(3,3)<<
            cos(Motion::yaw * CV_PI / 180.0),0,-sin(Motion::yaw * CV_PI / 180.0),
            0,1,0,
            sin(Motion::yaw * CV_PI / 180.0),0,cos(Motion::yaw * CV_PI / 180.0));  
        cv::Mat R_pitch = (cv::Mat_<double>(3,3)<<
            1,0,0,
            0,cos(Motion::pitch * CV_PI / 180.0),sin(Motion::pitch * CV_PI / 180.0),
            0,-sin(Motion::pitch * CV_PI / 180.0),cos(Motion::pitch * CV_PI / 180.0));
        cv::Mat R_roll = (cv::Mat_<double>(3,3)<<
            cos(Motion::roll * CV_PI / 180.0),sin(Motion::roll * CV_PI / 180.0),0,
            -sin(Motion::roll * CV_PI / 180.0),cos(Motion::roll * CV_PI / 180.0),0,
            0,0,1);
        return R_yaw * R_pitch * R_roll;
    }

    cv::Mat getRotationMatrixW2G()
    {
        return getRotationMatrixG2W().t();
    }
}

namespace Transformation
{
    //使用solvePnP解算出目标的旋转向量和平移向量
    void solvePnP(const std::vector<cv::Point2d>& imagePoints, cv::Mat& rvec, cv::Mat& tvec)
    {
        cv::solvePnP(objectPoints, imagePoints, K, dicoef, rvec, tvec, true,cv::SOLVEPNP_IPPE);
    }

    //将tvec转换到世界坐标系下
    void transformToWorldCoord(const cv::Mat& rvec, const cv::Mat& tvec, cv::Mat& tvec_world, cv::Mat& rmat_world)
    {
        cv::Mat R = getRotationMatrixG2W();
        tvec_world = cv::Mat(R * tvec);
        //把rvec转换为旋转矩阵
        cv::Mat rmat_cam;
        cv::Rodrigues(rvec, rmat_cam);     // rvec → rmat_cam (3x3)
        rmat_world = R * rmat_cam; // 世界系旋转矩阵 (3x3)
        //计算世界系下的rvec的x分量的指向
    }

    void projectWorldPointToImage(cv::Point3d objectPoints,cv::Mat& img)
    {

        //转换到相机坐标系
        cv::Mat pt_mat = (cv::Mat_<double>(3,1) << objectPoints.x,objectPoints.y,objectPoints.z);
        cv::Mat R = getRotationMatrixW2G();
        cv::Mat pt_cam = R * pt_mat; //相机坐标系下的点(考虑tx,ty为0,即相机在云台中心)
        //投影到图像平面
        cv::Mat pt_img = K * pt_cam;
        cv::Point2d pt;
        pt.x = pt_img.at<double>(0) / pt_img.at<double>(2);
        pt.y = pt_img.at<double>(1) / pt_img.at<double>(2);
        //画点
        // std::cout<<"Projected point: " << pt << std::endl;
        cv::circle(img,cv::Point(pt.x,pt.y),8,cv::Scalar(0,255,0),cv::FILLED);
    }

    cv::Point3d calculateCenterPoint(cv::Mat tvec1, cv::Mat tvec2, cv::Mat r_mat1, cv::Mat r_mat2)
    {
        double t1x = tvec1.at<double>(0);
        double t1z = tvec1.at<double>(2);
        double t2x = tvec2.at<double>(0);
        double t2z = tvec2.at<double>(2);
        //获取rmat的x轴在世界坐标系下的指向
        std::cout<<"r_mat1: "<<r_mat1<<std::endl;
        std::cout<<"r_mat2: "<<r_mat2<<std::endl;
        double r1x = r_mat1.at<double>(0,0);
        double r1z = r_mat1.at<double>(2,0);
        double r2x = r_mat2.at<double>(0,0);
        double r2z = r_mat2.at<double>(2,0);
        std::cout<<"r1x: "<<r1x<<" r1z: "<<r1z<<std::endl;
        std::cout<<"r2x: "<<r2x<<" r2z: "<<r2z<<std::endl;
        //解方程组，计算出两条直线的最近点
        double lambda2 = (r2z*t1x - r2x*t1z - r2z*t2x + r2x*t2z) / (r2z*r1x - r2x*r1z - r2z*r2x + r2x*r2z);
        double lambda1 = (t1x + lambda2*r1x - t2x) / (r2x - r1x);
        double y = (tvec1.at<double>(1) + tvec2.at<double>(1)) / 2.0;
        double z = (tvec1.at<double>(2) + lambda1*r1z + tvec2.at<double>(2) + lambda2*r2z) / 2.0;
        return cv::Point3d((tvec1.at<double>(0) + lambda1*r1x + tvec2.at<double>(0) + lambda2*r2x) / 2.0,y,z);
    }
}
#endif