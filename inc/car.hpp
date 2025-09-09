#ifndef CAR_HPP
#define CAR_HPP

#include "armor.hpp"



class Car
{
    
private:
    Armor armors[4];
    int id; //car id
    cv::Mat center;  //车体中心矢量 
};

#endif