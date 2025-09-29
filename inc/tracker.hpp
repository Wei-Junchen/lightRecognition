#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "armor.hpp"
#include "filter.hpp"
#include <vector>
#include <queue>

#define FILTER_TYPE 1 //1 for 3D filter, 2 for 2D filter

namespace Shooter { class CapableTrajectory; }
class Car;

namespace
{
    constexpr double pixel_distance_threshold = 300.0f; //像素距离阈值，超过这个距离就不匹配了
    //tvec_world三维位置+三维速度卡尔曼滤波器
    constexpr int MEASURE_DIM = 3;
    constexpr int STATE_DIM = 6;
    //初始化卡尔曼滤波器
    KalmanFilter initFilter(STATE_DIM, MEASURE_DIM, ArmorFilter::transitionMatrix, ArmorFilter::measurementMatrix, cv::Mat(), ArmorFilter::makeProcessNoiseCov6(ArmorFilter::dt,10000.0f), ArmorFilter::measurementNoiseCov,
                        (cv::Mat_<double>(6,6) <<    1,0,0,0,0,0,
                                                    0,1,0,0,0,0,
                                                    0,0,1,0,0,0,
                                                    0,0,0,100,0,0,
                                                    0,0,0,0,100,0,
                                                    0,0,0,0,0,1000));//速度初始方差设置大一点，因为只能观测位置
    //平面像素位置+二维速度卡尔曼滤波器
    constexpr int MEASURE_DIM_2D = 2;
    constexpr int STATE_DIM_2D = 4;
    KalmanFilter initFilter2D(STATE_DIM_2D, MEASURE_DIM_2D, ArmorFilter::transitionMatrix2D, ArmorFilter::measurementMatrix2D, cv::Mat(), ArmorFilter::processNoiseCov2D, ArmorFilter::measurementNoiseCov2D,
                           (cv::Mat_<double>(4,4) << 1,0,0,0,
                                                    0,1,0,0,
                                                    0,0,1,0,
                                                    0,0,0,1));//速度初始方差设置大一点，因为只能观测位置
}

//注意他tvec为double类型，下面使用template索引时要注意，而且我从double窄化为double了
//todo: 这里可以考虑把v也改为double类型

namespace ArmorTracker
{
    class TrackedArmor
    {
        friend class Shooter::CapableTrajectory;
        friend class ::Car;
    public:
        static void HungarianMatch(std::vector<Armor>& detectedArmors);
        static void predictTrackedArmors(double dt);
#if FILTER_TYPE == 1
        TrackedArmor(const Armor& armor, KalmanFilter kf = initFilter) : armor_(armor), kf_(kf), lostCount_(0)
        {
            cv::Mat state = (cv::Mat_<double>(6,1) << armor.tvec_world.at<double>(0), armor.tvec_world.at<double>(1),
                             armor.tvec_world.at<double>(2),0,0,0);
            kf_.UpdateState(state);
            absolute_id_ = ++global_id_;
        }
#elif FILTER_TYPE == 2
        TrackedArmor(const Armor& armor, KalmanFilter kf = initFilter2D) : armor_(armor), kf_(kf), lostCount_(0)
        {
            cv::Mat state = (cv::Mat_<double>(4,1) << armor.box.center.x, armor.box.center.y,
                             0  ,0);
            kf_.UpdateState(state);
        }
#endif
        void PredictAndUpdate()
        {
            cv::Mat prediction = kf_.PredictAndUpdate();
#if FILTER_TYPE == 1
            armor_.tvec_world.at<double>(0) = prediction.at<double>(0);
            armor_.tvec_world.at<double>(1) = prediction.at<double>(1);
            armor_.tvec_world.at<double>(2) = prediction.at<double>(2);
            armor_.vx = prediction.at<double>(3);
            armor_.vy = prediction.at<double>(4);
            armor_.vz = prediction.at<double>(5);
#elif FILTER_TYPE == 2
            armor_.box.center.x = prediction.at<double>(0);
            armor_.box.center.y = prediction.at<double>(1);
            armor_.vx = prediction.at<double>(2);
            armor_.vy = prediction.at<double>(3);
#endif
        }
        cv::Mat PredictWithoutUpdate()
        {
            return kf_.PredictWithoutUpdate();
        }
        void update(const Armor& detectedArmor)
        {
            lostCount_ = 0; //重置丢失计数器
            if(followCount_ >= 2)
                recent_ids_.push(detectedArmor.id);
            if(recent_ids_.size() > 10) //只保留最近10个
                recent_ids_.pop();
#if FILTER_TYPE == 1
            cv::Mat measurement = (cv::Mat_<double>(3,1) << detectedArmor.tvec_world.at<double>(0),
                                   detectedArmor.tvec_world.at<double>(1),
                                   detectedArmor.tvec_world.at<double>(2));
            cv::Mat estimated = kf_.PredictAndUpdate(measurement);
            std::cout<<"estimated state: " << estimated.t() << std::endl;
            int tmpid = armor_.id_car;
            armor_ = detectedArmor;
            armor_.tvec_world.at<double>(0) = estimated.at<double>(0);
            armor_.tvec_world.at<double>(1) = estimated.at<double>(1);
            armor_.tvec_world.at<double>(2) = estimated.at<double>(2);
            armor_.vx = estimated.at<double>(3);
            armor_.vy = estimated.at<double>(4);
            armor_.vz = estimated.at<double>(5);
            armor_.id_car = tmpid;
            // 计算recent_ids_队列中众数
            std::queue<int> temp = recent_ids_;
            std::map<int, int> freq_map;
            while (!temp.empty()) {
                freq_map[temp.front()]++;
                temp.pop();
            }
            armor_.id = (freq_map.size() > 0) ? std::max_element(freq_map.begin(), freq_map.end(),
                [](const auto& a, const auto& b) { return a.second < b.second; })->first : detectedArmor.id;
#elif FILTER_TYPE == 2
            cv::Mat measurement = (cv::Mat_<double>(2,1) << detectedArmor.box.center.x,
                                   detectedArmor.box.center.y);
            cv::Mat estimated = kf_.PredictAndUpdate(measurement);
            armor_ = detectedArmor;
            armor_.box.center.x = estimated.at<double>(0);
            armor_.box.center.y = estimated.at<double>(1);
            armor_.vx = estimated.at<double>(2);
            armor_.vy = estimated.at<double>(3);
#endif
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
        KalmanFilter kf_;
        int lostCount_; //丢失计数器
        Armor armor_;
        bool isCatored = false; //是否被车体关联
        int followCount_ = 0; //跟踪计数器
        std::queue<int> recent_ids_; //最近跟踪到的装甲板ID队列
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