#ifndef CAR_HPP
#define CAR_HPP

//此头文件意在整车建模，以此求得车体中心点
#include "armor.hpp"
#include "tracker.hpp"
#include "transformation.hpp"
#include <vector>

class Car
{
    static inline std::vector<Car> cars; //所有车辆
public:
    static void DrawCars(cv::Mat& img)
    {
        for(const auto& car : cars)
        {
            if(car.isLost_)
                continue;
            //画出车体中心
            Transformation::projectWorldPointToImage(car.center_, img);
            //画出装甲板
            std::vector<Armor> trackedArmor;
            for(int i=0;i<4;++i)
            {
                if(car.isTracked_[i])
                    trackedArmor.push_back(car.armors_[i]);
            }
            Armor::DrawArmor(img, trackedArmor);
        }
    }

    static void calculateCarsCenter()
    {
        for(auto& car:cars)
        {
            if(car.isLost_)
                continue;
            car.calculateCenter();
        }
    }

    static void updateFromTrackedArmors(std::vector<ArmorTracker::TrackedArmor>& trackedArmors)
    {
        std::vector<cv::Point2d> carsLeftRight(cars.size(),cv::Point2d(10000.0f,-10000.0f)); //存储每辆车的左右边界点
        for(auto& car:cars)
        {
            car.lostCount_++;
            car.isLost_ = true;
            for(int i=0;i<4;++i)
                car.isTracked_[i] = false;
        }
        //更新车辆的追踪装甲板信息
        for(auto& armor:trackedArmors)
        {
            for(auto& car:cars)
            {
                for(int i=0;i<4;++i)
                {
                    if(car.armorsAbsoluteId_[i] == armor.getAbsoluteId())
                    {
                        car.lostCount_ = 0;
                        car.isLost_ = false;
                        car.armors_[i] = armor.getArmor();
                        car.armors_[i].id_car = i;
                        car.isTracked_[i] = true;
                        if(armor.getArmor().getCenter().x < carsLeftRight[&car - &cars[0]].x)
                            carsLeftRight[&car - &cars[0]].x = armor.getArmor().getCenter().x;
                        if(armor.getArmor().getCenter().x > carsLeftRight[&car - &cars[0]].y)
                            carsLeftRight[&car - &cars[0]].y = armor.getArmor().getCenter().x;
                        break;
                    }
                }
            }
        }
        //遍历carsLeftRight，更新每辆车的左右边界点
        for(size_t i=0;i<cars.size();++i)
        {
            if(carsLeftRight[i].x == 10000.0f && carsLeftRight[i].y == -10000.0f)
                continue;
            cars[i].x_min = carsLeftRight[i].x;
            cars[i].x_max = carsLeftRight[i].y;
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
                    isCatored = true;
                    if(!armor.isCatored) //同一辆车最多4块装甲板
                    {
                        // std::cout<<"Car id " << car.id_ << " found armor id " << armor.getArmor().getId() << " center: " << armor.getArmor().getCenter() << std::endl;
                        //与上一块装甲板的像素x位置进行比较
                        //如果新的装甲板x大于上一块装甲板x，则说明新的装甲板在右侧
                        if(car.x_max < armor.getArmor().getCenter().x)
                        {
                            car.armorCount_++;
                            if(car.armorCount_ >= 4) //最多4块装甲板
                                car.armorCount_ = 0;
                            car.armors_[car.armorCount_] = armor.getArmor();
                            car.armorsAbsoluteId_[car.armorCount_] = armor.getAbsoluteId();
                            armor.armor_.id_car = car.armorCount_;
                            car.lastArmorId = car.armorCount_;
                        }
                        //否则在左侧
                        else if(car.x_min > armor.getArmor().getCenter().x)
                        {
                            car.armorCount_--;
                                         if(car.armorCount_ < 0)
                                car.armorCount_ = 3;
                            car.armors_[car.armorCount_] = armor.getArmor();
                            car.armorsAbsoluteId_[car.armorCount_] = armor.getAbsoluteId();
                            armor.armor_.id_car = car.armorCount_;
                            car.lastArmorId = car.armorCount_;
                        }
                        //否则说明掉了几帧识别
                        else
                        {
                            std::cout<<"Armor x position error, maybe lost some frames!"<<std::endl;
                        }
                        armor.isCatored = true;
                    }
                }
            }
            if(!isCatored)
            {
                Car car;
                car.id_ = armor.getArmor().getId();
                //只有一块装甲板还无法确定车体中心，暂时将车体中心设为(0,0,0)
                car.center_ = cv::Vec3f(0,0,0);
                armor.armor_.id_car = car.armorCount_;
                car.armors_[car.armorCount_] = armor.getArmor();
                car.armorsAbsoluteId_[car.armorCount_] = armor.getAbsoluteId();
                armor.isCatored = true;
                car.lastArmorId = car.armorCount_;
                cars.push_back(car);
                std::cout<<"New car id: " << car.id_ << std::endl;
            }
        }
        //遍历所有车辆，看是否lost,丢失5帧以上就erase
        cars.erase(std::remove_if(cars.begin(), cars.end(),
                            [](const Car& car){ return car.lostCount_ > 20; }),
                            cars.end());
    }
    
    void calculateCenter()
    {
        //至少两块装甲板才能计算车体中心
        int trackedIndex[2] = {-1,-1};
        int trackedCount = 0;
        for(int i=0;i<4;++i)
        {
            if(isTracked_[i])
                trackedIndex[trackedCount++] = i;
        }
        if(trackedCount < 2)
            return;
        center_ = Transformation::calculateCenterPoint(armors_[trackedIndex[0]].tvec_world, armors_[trackedIndex[1]].tvec_world,
                                                      armors_[trackedIndex[0]].rmat_world, armors_[trackedIndex[1]].rmat_world);
        std::cout<<"Successfully calculate car id " << id_ << " center: " << center_ << std::endl;
    }
private:
    Armor armors_[4];
    bool isTracked_[4] = {false,false,false,false};
    int armorsAbsoluteId_[4] = {-1,-1,-1,-1}; //全局唯一ID
    int id_; //car id
    int lostCount_ = 0; //丢失计数器
    int armorCount_ = 0; //装甲板计数器
    int lastArmorId = -1; //上一块装甲板ID
    bool isLost_ = false; //是否丢失
    double x_min;
    double x_max;
    cv::Vec3d center_;  //车体中心矢量
};

#endif