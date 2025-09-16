#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "armor.hpp"
#include "filter.hpp"
#include <vector>

#define FILTER_TYPE 1 //1 for 3D filter, 2 for 2D filter

namespace Shooter { class CapableTrajectory; }

namespace
{
    //tvec_world三维位置+三维速度卡尔曼滤波器
    constexpr int MEASURE_DIM = 3;
    constexpr int STATE_DIM = 6;
    //初始化卡尔曼滤波器
    // KalmanFilter initFilter(STATE_DIM, MEASURE_DIM, ArmorFilter::transitionMatrix, ArmorFilter::measurementMatrix, cv::Mat(), ArmorFilter::processNoiseCov, ArmorFilter::measurementNoiseCov,
    //                        (cv::Mat_<float>(6,6) << 1,0,0,0,0,0,
    //                                                 0,1,0,0,0,0,
    //                                                 0,0,10,0,0,0,
    //                                                 0,0,0,100,0,0,
    //                                                 0,0,0,0,100,0,
    //                                                 0,0,0,0,0,1000));//速度初始方差设置大一点，因为只能观测位置
    KalmanFilter initFilter(STATE_DIM, MEASURE_DIM, ArmorFilter::transitionMatrix, ArmorFilter::measurementMatrix, cv::Mat(), ArmorFilter::makeProcessNoiseCov6(ArmorFilter::dt,5000.0f), ArmorFilter::measurementNoiseCov,
                        (cv::Mat_<float>(6,6) <<    1,0,0,0,0,0,
                                                    0,1,0,0,0,0,
                                                    0,0,1,0,0,0,
                                                    0,0,0,100,0,0,
                                                    0,0,0,0,100,0,
                                                    0,0,0,0,0,1000));//速度初始方差设置大一点，因为只能观测位置
    //平面像素位置+二维速度卡尔曼滤波器
    constexpr int MEASURE_DIM_2D = 2;
    constexpr int STATE_DIM_2D = 4;
    KalmanFilter initFilter2D(STATE_DIM_2D, MEASURE_DIM_2D, ArmorFilter::transitionMatrix2D, ArmorFilter::measurementMatrix2D, cv::Mat(), ArmorFilter::processNoiseCov2D, ArmorFilter::measurementNoiseCov2D,
                           (cv::Mat_<float>(4,4) << 1,0,0,0,
                                                    0,1,0,0,
                                                    0,0,1,0,
                                                    0,0,0,1));//速度初始方差设置大一点，因为只能观测位置
}

//注意他tvec为double类型，下面使用template索引时要注意，而且我从double窄化为float了
//todo: 这里可以考虑把v也改为double类型

namespace ArmorTracker
{
    class TrackedArmor
    {
        friend class Shooter::CapableTrajectory;
    public:
        static void HungarianMatch(std::vector<Armor>& detectedArmors);
        static void predictTrackedArmors(float dt);
#if FILTER_TYPE == 1
        TrackedArmor(const Armor& armor, KalmanFilter kf = initFilter) : armor_(armor), kf_(kf), lostCount_(0)
        {
            cv::Mat state = (cv::Mat_<float>(6,1) << armor.tvec_world.at<double>(0), armor.tvec_world.at<double>(1),
                             armor.tvec_world.at<double>(2),0,0,0);
            kf_.UpdateState(state);
        }
#elif FILTER_TYPE == 2
        TrackedArmor(const Armor& armor, KalmanFilter kf = initFilter2D) : armor_(armor), kf_(kf), lostCount_(0)
        {
            cv::Mat state = (cv::Mat_<float>(4,1) << armor.box.center.x, armor.box.center.y,
                             0  ,0);
            kf_.UpdateState(state);
        }
#endif
        void PredictAndUpdate()
        {
            cv::Mat prediction = kf_.PredictAndUpdate();
#if FILTER_TYPE == 1
            armor_.tvec_world.at<double>(0) = prediction.at<float>(0);
            armor_.tvec_world.at<double>(1) = prediction.at<float>(1);
            armor_.tvec_world.at<double>(2) = prediction.at<float>(2);
            armor_.vx = prediction.at<float>(3);
            armor_.vy = prediction.at<float>(4);
            armor_.vz = prediction.at<float>(5);
#elif FILTER_TYPE == 2
            armor_.box.center.x = prediction.at<float>(0);
            armor_.box.center.y = prediction.at<float>(1);
            armor_.vx = prediction.at<float>(2);
            armor_.vy = prediction.at<float>(3);
#endif
        }
        cv::Mat PredictWithoutUpdate()
        {
            return kf_.PredictWithoutUpdate();
        }
        void update(const Armor& detectedArmor)
        {
            lostCount_ = 0; //重置丢失计数器
#if FILTER_TYPE == 1
            cv::Mat measurement = (cv::Mat_<float>(3,1) << detectedArmor.tvec_world.at<double>(0),
                                   detectedArmor.tvec_world.at<double>(1),
                                   detectedArmor.tvec_world.at<double>(2));
            cv::Mat estimated = kf_.PredictAndUpdate(measurement);
            // std::cout<<"estimated state: " << estimated.t() << std::endl;
            armor_ = detectedArmor;
            armor_.tvec_world.at<double>(0) = estimated.at<float>(0);
            armor_.tvec_world.at<double>(1) = estimated.at<float>(1);
            armor_.tvec_world.at<double>(2) = estimated.at<float>(2);
            armor_.vx = estimated.at<float>(3);
            armor_.vy = estimated.at<float>(4);
            armor_.vz = estimated.at<float>(5);
#elif FILTER_TYPE == 2
            cv::Mat measurement = (cv::Mat_<float>(2,1) << detectedArmor.box.center.x,
                                   detectedArmor.box.center.y);
            cv::Mat estimated = kf_.PredictAndUpdate(measurement);
            armor_ = detectedArmor;
            armor_.box.center.x = estimated.at<float>(0);
            armor_.box.center.y = estimated.at<float>(1);
            armor_.vx = estimated.at<float>(2);
            armor_.vy = estimated.at<float>(3);
#endif
        }
        int getLostCount() const { return lostCount_; }
        const Armor& getArmor() const { return armor_; }

        void predictArmors(float dt)
        {
            //更新所有跟踪装甲板的预测位置
                armor_.predict_position = cv::Point3f(armor_.tvec_world.at<double>(0) + armor_.vx * dt,
                                                        armor_.tvec_world.at<double>(1) + armor_.vy * dt,
                                                        armor_.tvec_world.at<double>(2) + armor_.vz * dt);
        }
    private:
        KalmanFilter kf_;
        int lostCount_; //丢失计数器
        Armor armor_;
        static inline int maxLostCount = 1; //最大丢失计数器
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
            cv::Mat prediction = tracked.PredictWithoutUpdate();
            float min_distance = 300.0f;
            int min_index = -1;
            // std::cout<<"Predicted state: " << prediction.t() << std::endl;
            for(size_t i = 0; i < detectedArmors.size(); i++)
            {
                if(matched[i])
                    continue;
#if FILTER_TYPE == 1
                cv::Mat predict_tvec = (cv::Mat_<float>(3,1) << prediction.at<float>(0),
                                        prediction.at<float>(1),
                                        prediction.at<float>(2));
                //convert float to double
                predict_tvec.convertTo(predict_tvec, CV_64F);
                float distance = cv::norm(predict_tvec - detectedArmors[i].tvec_world);
                // std::cout<<"distance: " << distance << std::endl;
#elif FILTER_TYPE == 2
                cv::Point2f predict_center(prediction.at<float>(0), prediction.at<float>(1));
                float distance = cv::norm(predict_center - detectedArmors[i].box.center);
#endif
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

    void TrackedArmor::predictTrackedArmors(float dt)
    {
        //更新所有跟踪装甲板的预测位置
        for(auto& tracked : trackedArmors)
        {
            tracked.armor_.predict_position = cv::Point3f(tracked.armor_.tvec_world.at<double>(0) + tracked.armor_.vx * dt,
                                                    tracked.armor_.tvec_world.at<double>(1) + tracked.armor_.vy * dt,
                                                    tracked.armor_.tvec_world.at<double>(2) + tracked.armor_.vz * dt);
            // std::cout<<"Predicted position: " << tracked.armor_.predict_position << std::endl;
        }
    }
}

#endif