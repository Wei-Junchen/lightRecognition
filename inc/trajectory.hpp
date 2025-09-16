#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <opencv2/opencv.hpp>

namespace
{
    float bulletSpeed = 25.0f * 1000.0f; //mm/s
    float gravity = 9.81f * 1000.0f; //mm/s^2
}


namespace Trajectory
{
    //规定vec3f的分量为yaw,pitch,总路径消耗时间
    cv::Vec3f calculateTvec(cv::Point3f target_position)
    {
        //考虑PnP解算系x轴向右，y轴向下，z轴向里
        float yaw = -std::atan2(target_position.x, target_position.z) * 180.0f / CV_PI; //x < 0 ,z > 0时，yaw > 0
        float distance = std::sqrt(target_position.x * target_position.x + target_position.z * target_position.z);
        
        float pitch1 = atan2(gravity * distance, bulletSpeed * bulletSpeed * 2.0f) * 180.0f / CV_PI; //抛物线最高点在目标前方时，pitch > 0
        float pitch2 = 90.0f - pitch1; //抛物线最高点在目标后方时，pitch < 0
        // std::cout<<"pitch1: "<<pitch1<<", pitch2: "<<pitch2<<std::endl;
        float t1 = distance / (bulletSpeed * std::cos(pitch1 * CV_PI / 180.0f));
        float t2 = distance / (bulletSpeed * std::cos(pitch2 * CV_PI / 180.0f));
        if(t1 < t2)
        {
            return cv::Vec3f(yaw, pitch1, t1);
        }
        else
        {
            return cv::Vec3f(yaw, pitch2, t2);
        }
    }
}

#endif