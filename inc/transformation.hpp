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
    cv::Mat objectPoints = (cv::Mat_<double>(6,3) << 
                    -65.0, 55.0, 0.,   //左上
                    -65.0, 0.0, 0.,    //左中
                    -65.0, -55.0, 0.,  //左下
                    65.0, 55.0, 0.,   //右上
                    65.0, 0., 0.,     //右中   
                    65.0, -55.0, 0.); //右下

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
    void solvePnP(const std::vector<cv::Point2f>& imagePoints, cv::Mat& rvec, cv::Mat& tvec)
    {
        cv::solvePnP(objectPoints, imagePoints, K, dicoef, rvec, tvec);
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

    void projectWorldPointToImage(cv::Point3f objectPoints,cv::Mat& img)
    {

        //转换到相机坐标系
        cv::Mat pt_mat = (cv::Mat_<double>(3,1) << objectPoints.x,objectPoints.y,objectPoints.z);
        cv::Mat R = getRotationMatrixW2G();
        cv::Mat pt_cam = R * pt_mat; //相机坐标系下的点(考虑tx,ty为0,即相机在云台中心)
        //投影到图像平面
        cv::Mat pt_img = K * pt_cam;
        cv::Point2f pt;
        pt.x = pt_img.at<double>(0) / pt_img.at<double>(2);
        pt.y = pt_img.at<double>(1) / pt_img.at<double>(2);
        //画点
        // std::cout<<"Projected point: " << pt << std::endl;
        cv::circle(img,cv::Point(pt.x,pt.y),8,cv::Scalar(0,255,0),cv::FILLED);
    }
}
#endif