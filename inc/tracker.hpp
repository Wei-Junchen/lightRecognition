#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "armor.hpp"
#include "filter.hpp"
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <chrono>

namespace Shooter { class CapableTrajectory; }
class Car;

namespace
{
    constexpr double pixel_distance_threshold = 200.0f; //像素距离阈值，超过这个距离就不匹配了
}

namespace ArmorTracker
{
    class TrackedArmor
    {
        friend class Shooter::CapableTrajectory;
        friend class ::Car;
    public:
        static void HungarianMatch(std::vector<Armor>& detectedArmors);
        static void predictTrackedArmors(double dt);

        TrackedArmor(const Armor& armor) : 
            armor_(armor),lostCount_(0),last_time_(std::chrono::steady_clock::now()),
            ekf_(Eigen::Vector<double,9>((Eigen::Vector<double,9>() 
            <<  armor.tvec_world.at<double>(0),
                armor.tvec_world.at<double>(1),
                armor.tvec_world.at<double>(2),
                0,
                0,
                0,
                armor.angle_world,0,0).finished())) { absolute_id_ = ++global_id_; }
        ~TrackedArmor() = default;

        void update(const Armor& detectedArmor)
        {
            lostCount_ = 0; //重置丢失计数器S
            Eigen::Vector<double,4> measurement = (Eigen::Vector<double,4>() << 
                detectedArmor.tvec_world.at<double>(0),
                detectedArmor.tvec_world.at<double>(1),
                detectedArmor.tvec_world.at<double>(2),
                detectedArmor.angle_world).finished();

            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - last_time_).count();
            last_time_ = std::chrono::steady_clock::now();

            Eigen::Vector<double,9> state_merged = ekf_.PredictAndUpdate(measurement, dt);

            // std::cout<<"estimated state: " << state_merged.transpose() << std::endl;

            armor_ = detectedArmor;
            armor_.tvec_world.at<double>(0) = state_merged(0);
            armor_.tvec_world.at<double>(1) = state_merged(1);
            armor_.tvec_world.at<double>(2) = state_merged(2);
            armor_.vx = state_merged(3);
            armor_.vy = state_merged(4);
            armor_.vz = state_merged(5);
            armor_.angle_world = state_merged(6);
            armor_.w = state_merged(7);
            armor_.r = state_merged(8);
        }
        
        int getLostCount() const { return lostCount_; }
        const Armor& getArmor() const { return armor_; }

        void predictArmors(double dt)
        {
            //更新所有跟踪装甲板的预测位置
                armor_.predict_position = cv::Point3f(armor_.tvec_world.at<double>(0) + armor_.vx * dt,
                                                        armor_.tvec_world.at<double>(1) + armor_.vy * dt,
                                                        armor_.tvec_world.at<double>(2) + armor_.vz * dt);
        }
        int getFollowCount() const { return followCount_; }
        int getAbsoluteId() const { return absolute_id_; }
    private:
        ExtendedKalmanFilter<9, 4> ekf_;
        std::chrono::steady_clock::time_point last_time_;
        Armor armor_;
        
        bool isCatored = false; //是否被车体关联
        int lostCount_; //丢失计数器
        int followCount_ = 0; //跟踪计数器
        int absolute_id_; //全局唯一ID

        static inline int maxLostCount = 1; //最大丢失计数器
        static inline int global_id_ = 0; //全局唯一ID计数器
    };
    
    std::vector<TrackedArmor> trackedArmors;

    //使用匈牙利算法进行匹配
    //这里为了简单起见，使用了贪心算法进行匹配
    void TrackedArmor::HungarianMatch(std::vector<Armor>& detectedArmors)
    {
        //将检测到的装甲板与已跟踪的装甲板进行匹配
        std::vector<bool> matched(detectedArmors.size(), false);
        for(auto& tracked : trackedArmors)
        {
            tracked.lostCount_++;
            /*  
                通过tvec的距离动态调整min_distance,
                因为装甲板匹配是通过两帧之间的距离来判断的，
                不是tvec(像素匹配更简单而且直观，符合人眼观测规律)
            */
            double distance_to_camera = cv::norm(tracked.armor_.tvec_world);
            double min_distance = pixel_distance_threshold /( distance_to_camera * 1e-3); //距离越远，允许的像素距离小
            // std::cout<<"min_distance: " << min_distance << std::endl;
            int min_index = -1;
            for(size_t i = 0; i < detectedArmors.size(); i++)
            {
                if(matched[i])
                    continue;
                cv::Point2f predict_center(tracked.armor_.box.center.x, tracked.armor_.box.center.y);
                double distance = cv::norm(predict_center - detectedArmors[i].box.center);
                // std::cout<<"distance: " << distance << std::endl;
                if(distance < min_distance && tracked.armor_.id == detectedArmors[i].id)
                {
                    min_distance = distance;
                    min_index = i;
                }
            }
            if(min_index != -1)
            {
                matched[min_index] = true;
                tracked.update(detectedArmors[min_index]);
                tracked.followCount_++;
            }
        }
        //将未匹配的检测到的装甲板加入跟踪列表
        for(size_t i = 0; i < detectedArmors.size(); i++)
        {
            if(!matched[i])
            {
                trackedArmors.emplace_back(detectedArmors[i]);
                std::cout<<"New tracked armor: " << detectedArmors[i].tvec_world.t() << std::endl;
            }
        }
        //移除丢失过多次的跟踪装甲板
        trackedArmors.erase(std::remove_if(trackedArmors.begin(), trackedArmors.end(),
                            [](const TrackedArmor& ta){ return ta.getLostCount() > TrackedArmor::maxLostCount; }),
                            trackedArmors.end());
    }

    void getTrackedArmors(std::vector<Armor>& output)
    {
        output.clear();
        for(const auto& ta : trackedArmors)
        {
            output.push_back(ta.getArmor());
        }
    }

    void TrackedArmor::predictTrackedArmors(double dt)
    {
        //更新所有跟踪装甲板的预测位置
        for(auto& tracked : trackedArmors)
        {
            tracked.armor_.predict_position = cv::Point3f(tracked.armor_.tvec_world.at<double>(0) + tracked.armor_.vx * dt,
                                                        tracked.armor_.tvec_world.at<double>(1) + tracked.armor_.vy * dt,
                                                        tracked.armor_.tvec_world.at<double>(2) + tracked.armor_.vz * dt);
        }
    }
}

#endif