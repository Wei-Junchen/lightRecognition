#ifndef CAR_HPP
#define CAR_HPP

//此头文件意在整车建模，以此求得车体中心点
#include "armor.hpp"
#include "tracker.hpp"
#include <vector>

class Car
{
    static inline std::vector<Car> cars; //所有车辆
public:
    static void updateFromTrackedArmors(std::vector<ArmorTracker::TrackedArmor>& trackedArmors)
    {
        for(auto& car:cars)
        {
            car.lostCount_++;
            car.isLost_ = true;
        }
        for(auto& armor:trackedArmors)
        {
            if(armor.getFollowCount() < 3 && !armor.isCatored) //至少跟踪2帧以上
                continue;
            bool isCatored = false;
            for(auto& car:cars)
            {
                if(car.id_ == armor.getArmor().getId())
                {
                    car.lostCount_ = 0;
                    car.isLost_ = false;
                    isCatored = true;
                    if(!armor.isCatored) //同一辆车最多4块装甲板
                    {
                        car.armors_[car.armorCount_++] = armor.getArmor();
                        armor.armor_.id_car = car.armorCount_ - 1;
                        armor.isCatored = true;
                    }
                    if(car.armorCount_ >= 4)
                        car.armorCount_ = 0;
                }
            }
            if(!isCatored)
            {
                Car car;
                car.id_ = armor.getArmor().getId();
                //只有一块装甲板还无法确定车体中心，暂时将车体中心设为(0,0,0)
                car.center_ = cv::Vec3f(0,0,0);
                armor.armor_.id_car = car.armorCount_;
                car.armors_[car.armorCount_++] = armor.getArmor();
                armor.isCatored = true;
                cars.push_back(car);
                std::cout<<"New car id: " << car.id_ << std::endl;
            }
        }
        //遍历所有车辆，看是否lost,丢失5帧以上就erase
        cars.erase(std::remove_if(cars.begin(), cars.end(),
                            [](const Car& car){ return car.lostCount_ > 5; }),
                            cars.end());
    }
private:
    Armor armors_[4];
    int id_; //car id
    int lostCount_ = 0; //丢失计数器
    int armorCount_ = 0; //装甲板数量
    bool isLost_ = false; //是否丢失
    cv::Vec3f center_;  //车体中心矢量 
};

#endif