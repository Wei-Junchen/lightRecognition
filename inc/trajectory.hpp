#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <opencv2/opencv.hpp>

namespace
{
    double bulletSpeed = 25.0f * 1000.0f; //mm/s
    double gravity = 9.81f * 1000.0f; //mm/s^2
}


namespace Trajectory
{
    //规定vec3f的分量为yaw,pitch,总路径消耗时间
    cv::Vec3f calculateTvec(cv::Point3d target_position)
    {
        //考虑PnP解算系x轴向右，y轴向下，z轴向里
        double yaw = -std::atan2(target_position.x, target_position.z) * 180.0f / CV_PI; //x < 0 ,z > 0时，yaw > 0
        double distance = std::sqrt(target_position.x * target_position.x + target_position.z * target_position.z);
        
        // 检查抛物线是否有解：当 gravity * distance > bulletSpeed^2 时无解
        double discriminant = gravity * distance / (bulletSpeed * bulletSpeed);
        if(discriminant > 1.0f)
        {
            // 抛物线无解，目标距离太远
            return cv::Vec3f(0.0f, 0.0f, 0.0f);
        }
        
        double pitch1 = atan2(gravity * distance, bulletSpeed * bulletSpeed * 2.0f) * 180.0f / CV_PI; //抛物线最高点在目标前方时，pitch > 0
        double pitch2 = 90.0f - pitch1; //抛物线最高点在目标后方时，pitch < 0
        
        double t1 = distance / (bulletSpeed * std::cos(pitch1 * CV_PI / 180.0f));
        double t2 = distance / (bulletSpeed * std::cos(pitch2 * CV_PI / 180.0f));
        
        // 检查时间是否为正值
        if(t1 <= 0.0f && t2 <= 0.0f)
            return cv::Vec3f(0.0f, 0.0f, 0.0f);
        
        // 选择较小的正时间
        if(t1 > 0.0f && t2 > 0.0f)
        {
            if(t1 < t2)
                return cv::Vec3f(yaw, pitch1, t1);
            else
                return cv::Vec3f(yaw, pitch2, t2);
        }
        else if(t1 > 0.0f)
            return cv::Vec3f(yaw, pitch1, t1);
        else if(t2 > 0.0f)
            return cv::Vec3f(yaw, pitch2, t2);
        else
            return cv::Vec3f(0.0f, 0.0f, 0.0f);
    }
}

#endif