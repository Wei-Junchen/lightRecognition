#ifndef CAR_HPP
#define CAR_HPP

//此头文件意在整车建模，以此求得车体整体状态
#include "armor.hpp"
#include "tracker.hpp"
#include "transformation.hpp"
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <optional>

class Car
{
    static inline std::vector<Car> cars; //所有车辆
public:
    static void DrawCars(cv::Mat& img)
    {
        for(auto const& car : cars)
        {
            if(car.isLost_)
                continue;
            //画出车体中心
            Transformation::projectWorldPointToImage(car.center_, img);
            //画出装甲板
            std::vector<Armor> trackedArmor;
            for(auto const& armorData : car.armors_)
            {
                if(armorData.isTracked)
                    trackedArmor.push_back(armorData.armor);
            }
            Armor::DrawArmor(img, trackedArmor);
        }
    }

    void calculateCenter()
    {
        //至少两块装甲板才能计算车体中心
        int trackedIndex[2] = {-1,-1};
        int trackedCount = 0;
        for(auto const& armorData: armors_)
        {
            if(armorData.isTracked)
            {
                trackedIndex[trackedCount++] = &armorData - &armors_[0];
                if(trackedCount >= 2)
                    break;
            }
        }
        if(trackedCount < 2)
            return;

        center_ = Transformation::calculateCenterPoint(armors_[trackedIndex[0]].armor.tvec_world, armors_[trackedIndex[1]].armor.tvec_world,
                                                      armors_[trackedIndex[0]].armor.rmat_world, armors_[trackedIndex[1]].armor.rmat_world);
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
             //更新车辆的追踪装甲板信息
            car.lostCount_++;
            car.isLost_ = true;
            for(auto& armor:car.armors_)
                armor.isTracked = false;
            
            for(auto const& trackedArmor:trackedArmors)
            {
                for(auto& armor:car.armors_)
                {
                    if(armor.absoluteId == trackedArmor.getAbsoluteId())
                    {
                        car.lostCount_ = 0;
                        car.isLost_ = false;
                        armor.armor = trackedArmor.getArmor();
                        armor.isTracked = true;
                        Armor thisArmor = trackedArmor.getArmor();
                        if(thisArmor.getCenter().x < carsLeftRight[&car - &cars[0]].x)
                            carsLeftRight[&car - &cars[0]].x = thisArmor.getCenter().x;
                        if(thisArmor.getCenter().x > carsLeftRight[&car - &cars[0]].y)
                            carsLeftRight[&car - &cars[0]].y = thisArmor.getCenter().x;
                        break;
                    }
                }
            }
            std::optional<ArmorData*> updateArmor = ArmorData::getOldest(car.armors_);
            if(updateArmor)
            {
                Armor const* armor = &(*updateArmor)->armor;
                Eigen::Vector<double,3> measurement;
                measurement << armor->angle_world, armor->w, armor->r;
                double dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - car.lastTime_).count();
                car.lastTime_ = std::chrono::steady_clock::now();
                (*updateArmor)->lastTime_ = std::chrono::steady_clock::now();
                car.ekf_.PredictAndUpdate(measurement, dt);
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
            if(armor.getFollowCount() < 2 && !armor.isCatored) //至少跟踪2帧以上
                continue;
            bool isCatored = false;
            for(auto& car:cars)
            {
                if(car.id_ == armor.getArmor().getId())
                {
                    isCatored = true;
                    if(!armor.isCatored) //同一辆车最多4块装甲板
                    {
                        //新的装甲板加入车体后，根据车体滤波器重置自身状态的角速度和转弯半径
                        Eigen::VectorXd state_values(2);
                        state_values << car.ekf_.getState()(1), car.ekf_.getState()(2);
                        Eigen::VectorXd covariances(2);
                        covariances << 20, 200;
                        armor.ekf_.resetPartialState({7,8}, state_values, covariances);
                        //与上一块装甲板的像素x位置进行比较
                        //如果新的装甲板x大于上一块装甲板x，则说明新的装甲板在右侧
                        if(car.x_max < armor.getArmor().getCenter().x)
                        {
                            car.armorCount_++;
                            if(car.armorCount_ >= 4) //最多4块装甲板
                                car.armorCount_ = 0;

                            car.armors_[car.armorCount_].armor = armor.getArmor();
                            car.armors_[car.armorCount_].absoluteId = armor.getAbsoluteId();

                            armor.armor_.id_car = car.armorCount_;
                            car.lastArmorId = car.armorCount_;
                        }
                        //否则在左侧
                        else if(car.x_min > armor.getArmor().getCenter().x)
                        {
                            car.armorCount_--;
                            if(car.armorCount_ < 0)
                                car.armorCount_ = 3;

                            car.armors_[car.armorCount_].armor = armor.getArmor();
                            car.armors_[car.armorCount_].absoluteId = armor.getAbsoluteId();

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
            //如果没有找到可以归类的车体，则新建一辆车体
            if(!isCatored)
            {
                Armor armorData = armor.getArmor();
                Car car(Eigen::Vector3d(armorData.angle_world,armorData.w,armorData.r));
                car.id_ = armorData.id;
                //只有一块装甲板还无法确定车体中心，暂时将车体中心设为(0,0,0)
                car.center_ = cv::Vec3f(0,0,0);

                armor.armor_.id_car = car.armorCount_;
                car.armors_[car.armorCount_].armor = armor.getArmor();
                car.armors_[car.armorCount_].absoluteId = armor.getAbsoluteId();
                armor.armor_.id_car = car.armorCount_;
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

    Car(Eigen::Vector<double,3> init_state): ekf_(init_state,Eigen::Matrix<double,3,3>::Identity(),CarFilter::measurementMatrix,
                                                  CarFilter::processNoiseCov,CarFilter::measurementNoiseCov,CarFilter::f,CarFilter::f_jacobian),lastTime_(std::chrono::steady_clock::now()) {}
private:
    struct ArmorData
    {
        Armor armor;
        bool isTracked = false;
        int absoluteId = -1; //全局唯一ID
        std::chrono::steady_clock::time_point lastTime_ ;

        static std::optional<ArmorData*> getOldest(std::vector<ArmorData>& armorDatas)
        {
            ArmorData* oldest = nullptr;
            for(auto& armordata:armorDatas)
            {
                if(armordata.isTracked)
                {
                    if(oldest == nullptr || armordata < *oldest)
                        oldest = &armordata;
                }
            }
            return oldest ? std::optional<ArmorData*>(oldest) : std::nullopt;
        }

        bool operator<(ArmorData const& other) const
        {
            return absoluteId < other.absoluteId;
        }
    };

    std::vector<ArmorData> armors_ = std::vector<ArmorData>(4); //四块装甲板

    int id_; //car id
    int lostCount_ = 0; //丢失计数器

    int armorCount_ = 0; //装甲板计数器

    int lastArmorId = -1; //上一块装甲板ID

    bool isLost_ = false; //是否丢失
    double x_min,x_max; //左右边界点x坐标

    cv::Vec3d center_;  //车体中心矢量

    //状态向量: [theta,omega,radius]^T
    ExtendedKalmanFilter<3,3> ekf_; //用于车体状态估计的扩展卡尔曼滤波器
    std::chrono::steady_clock::time_point lastTime_;
};

#endif