#ifndef MOTION_HPP
#define MOTION_HPP

#include <opencv2/opencv.hpp>

//获取车子自身的运动信息

namespace Motion
{
    double yaw = 0.0f; //yaw角，单位度
    double pitch = 0.0f; //pitch角，单位度
    double roll = 0.0f; //roll角，单位度
    double w_yaw_max = 180.0f; //yaw最大角速度，单位度/s
    double w_pitch_max = 180.0f; //pitch最大角速度，单位度/s
    cv::Point3d direction;
}




#endif