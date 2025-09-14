#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "armor.hpp"
#include "filter.hpp"
#include <vector>

namespace
{
    constexpr int MEASURE_DIM = 3;
    constexpr int STATE_DIM = 6;
    //初始化卡尔曼滤波器
    KalmanFilter initFilter(STATE_DIM, MEASURE_DIM, ArmorFilter::transitionMatrix, ArmorFilter::measurementMatrix, cv::Mat(), ArmorFilter::processNoiseCov, ArmorFilter::measurementNoiseCov,cv::Mat::eye(STATE_DIM, STATE_DIM, CV_32F));
}

namespace ArmorTracker
{
    class TrackedArmor
    {
    public:
        static void HungarianMatch(std::vector<Armor>& detectedArmors);

        TrackedArmor(const Armor& armor, KalmanFilter kf = initFilter) : armor_(armor), kf_(kf), lostCount_(0)
        {
            cv::Mat state = (cv::Mat_<float>(6,1) << armor.tvec_world.at<float>(0), armor.tvec_world.at<float>(1),
                             armor.tvec_world.at<float>(2),0,0,0);
            kf_.UpdateState(state);
        }
        void PredictAndUpdate()
        {
            cv::Mat prediction = kf_.PredictAndUpdate();
            armor_.tvec_world.at<float>(0) = prediction.at<float>(0);
            armor_.tvec_world.at<float>(1) = prediction.at<float>(1);
            armor_.tvec_world.at<float>(2) = prediction.at<float>(2);
            armor_.vx = prediction.at<float>(3);
            armor_.vy = prediction.at<float>(4);
            armor_.vz = prediction.at<float>(5);
        }
        cv::Mat PredictWithoutUpdate()
        {
            return kf_.PredictWithoutUpdate();
        }
        void update(const Armor& detectedArmor)
        {
            lostCount_ = 0; //重置丢失计数器
            cv::Mat measurement = (cv::Mat_<float>(3,1) << detectedArmor.tvec_world.at<float>(0),
                                   detectedArmor.tvec_world.at<float>(1),
                                   detectedArmor.tvec_world.at<float>(2));
            cv::Mat estimated = kf_.PredictAndUpdate(measurement);
            armor_ = detectedArmor;
            armor_.tvec_world.at<float>(0) = estimated.at<float>(0);
            armor_.tvec_world.at<float>(1) = estimated.at<float>(1);
            armor_.tvec_world.at<float>(2) = estimated.at<float>(2);
            armor_.vx = estimated.at<float>(3);
            armor_.vy = estimated.at<float>(4);
            armor_.vz = estimated.at<float>(5);
        }
        int getLostCount() const { return lostCount_; }
        const Armor& getArmor() const { return armor_; }
    private:
        KalmanFilter kf_;
        int lostCount_; //丢失计数器
        Armor armor_;
        static inline int maxLostCount = 5; //最大丢失计数器
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
            TrackedArmor tmp = tracked;
            tmp.PredictAndUpdate();
            float min_distance = 1000.0f;
            int min_index = -1;
            std::cout<<"Predicted position: " << tmp.getArmor().tvec_world.t() << std::endl;
            //std::cout<<"veclocity: " << tmp.getArmor().vx << ", " << tmp.getArmor().vy << ", " << tmp.getArmor().vz << std::endl;
            for(size_t i = 0; i < detectedArmors.size(); i++)
            {
                if(matched[i])
                    continue;
                float distance = cv::norm(tmp.getArmor().tvec_world - detectedArmors[i].tvec_world);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    min_index = i;
                }
            }
            if(min_index != -1)
            {
                matched[min_index] = true;
                tracked.update(detectedArmors[min_index]);
            }
        }
        //将未匹配的检测到的装甲板加入跟踪列表
        for(size_t i = 0; i < detectedArmors.size(); i++)
        {
            if(!matched[i])
            {
               trackedArmors.emplace_back(detectedArmors[i]);
               std::cout<<"Added new tracked armor: " << trackedArmors.back().getArmor().tvec_world.t() << std::endl;
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
}

#endif