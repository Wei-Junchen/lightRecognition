#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include <opencv2/opencv.hpp>
#include "trajectory.hpp"
#include "tracker.hpp"

namespace
{
    float step = 0.02f; //s
    float max_error = 0.02f; //s
    float dt_max = 1.0f; //s
}

namespace Shooter
{
    class CapableTrajectory
    {
    public:
        static inline std::vector<CapableTrajectory> trajectories;
        static void computeTrajectory();
        void print() const
        {
            std::cout << "Root min: " << root_min << ", Root max: " << root_max << ", dt: " << dt << ", tracked_index: " << tracked_index << std::endl;
        }
        int getTrackedIndex() const { return tracked_index; }
        float getDt() const { return dt; }
    private:
        cv::Vec3f root_min; //yaw,pitch,time
        cv::Vec3f root_max; //yaw,pitch,time
        float dt; //时间增量
        int tracked_index; //被跟踪的装甲板在trackedArmors中的索引
    };


    //计算并返回云台需要转动的yaw和pitch角度，以及子弹飞行时间
    //输入目标的世界坐标系位置
    void CapableTrajectory::computeTrajectory()
    {
        trajectories.clear();
        for(auto& tracked : ArmorTracker::trackedArmors)
        {
            bool isFoundMinRoot = false;
            for(float t = 0; t < dt_max; t += step)
            {
                ArmorTracker::TrackedArmor::predictTrackedArmors(step);
                cv::Point3f target_pos = tracked.armor_.predict_position;
                cv::Vec3f root = Trajectory::calculateTvec(target_pos);
                //无解
                if(root == cv::Vec3f(0,0,0))
                    continue;
                // std::cout<<"root_now"<<root<<std::endl;
                if(std::abs(root[2] - t) <= max_error)
                {
                    CapableTrajectory traj;
                    traj.dt = t;
                    traj.tracked_index = &tracked - &ArmorTracker::trackedArmors[0];
                    trajectories.push_back(traj);
                    if(!isFoundMinRoot)
                        traj.root_min = root;
                    else
                    {
                        traj.root_max = root;
                        break;
                    }
                    isFoundMinRoot = true;
                }
            }
        }
    }
}
#endif