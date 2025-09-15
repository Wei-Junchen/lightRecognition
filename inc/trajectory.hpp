#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <opencv2/opencv.hpp>

namespace
{
    float bulletSpeed = 0.25f * 1000.0f; //mm/s
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
        //解一元二次方程求pitch
        float a = -0.5f * gravity * distance * distance / (bulletSpeed * bulletSpeed);
        float b = distance;
        float c = target_position.y;

        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            //无解
            return cv::Vec3f(0, 0, 0);
        }

        //如果有两个解，选择pitch较小的那个
        float sqrt_discriminant = std::sqrt(discriminant);
        float pitch1 = std::atan2((-b + sqrt_discriminant), (2 * a)) * 180.0f / CV_PI;
        float pitch2 = std::atan2((-b - sqrt_discriminant), (2 * a)) * 180.0f / CV_PI;
        float pitch = std::min(pitch1, pitch2); //选择较小的角度
        float time = distance / (bulletSpeed * std::cos(pitch * CV_PI / 180.0f));
        return cv::Vec3f(yaw, pitch, time);
    }
}

#endif