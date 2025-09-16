#ifndef MOTION_HPP
#define MOTION_HPP

#include <opencv2/opencv.hpp>

//获取车子自身的运动信息

namespace Motion
{
    float yaw = 0.0f; //yaw角，单位度
    float pitch = 0.0f; //pitch角，单位度
    float roll = 0.0f; //roll角，单位度
    float w_yaw_max = 180.0f; //yaw最大角速度，单位度/s
    float w_pitch_max = 180.0f; //pitch最大角速度，单位度/s
    cv::Point3f direction;
}




#endif