#ifndef MOTION_HPP
#define MOTION_HPP

#include <opencv2/opencv.hpp>

//获取车子自身的运动信息

namespace Motion
{
    float yaw = 0.0f; //yaw角，单位度
    float pitch = 0.0f; //pitch角，单位度
    float roll = 0.0f; //roll角，单位度
    cv::Point3f direction;
}




#endif