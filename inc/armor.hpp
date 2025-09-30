#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>
#include "cnn.hpp"
#include "transformation.hpp"

namespace ArmorTracker { class TrackedArmor; }
namespace Shooter { class CapableTrajectory; }
class Car;

struct MyRotatedRect
{
    cv::Point2f vertices[4];
    cv::Point2f center;
    cv::Size2f size;
    double angle;
    int id;
};

namespace
{
    //重排四边形的点，使得0-4顺时针顺序，0是最左上角点
    void reArrangeVertices(cv::Point2f* vertices)
    {
        float min = vertices[0].x + vertices[0].y;
        int index = 0;
        for(int i = 1; i < 4; i++)
        {
            float sum = vertices[i].x + vertices[i].y;
            if(sum < min)
            {
                min = sum;
                index = i;
            }
        }
        cv::Point2f tmp[4];
        for(int i=0;i<4;++i)
            tmp[i] = vertices[(i+index)%4];

        for(int i=0;i<4;++i)
            vertices[i] = tmp[i];
    }
}

class Armor
{
    friend class ArmorTracker::TrackedArmor;
    friend class Shooter::CapableTrajectory;
    friend class Car;
public:
    static void DrawArmor(cv::Mat& img,const std::vector<Armor>& armors)
    {
        for(const auto& armor : armors)
        {
            // if(armor.id_car == -1)
            //     continue;
            //画出两个灯条
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.lights[0].vertices[i], armor.lights[0].vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.lights[1].vertices[i], armor.lights[1].vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            //中心点
            cv::circle(img, (armor.lights[0].center + armor.lights[1].center) / 2, 6, cv::Scalar(0, 255 , 255));
            for(int i=0;i<2;++i)
            {
                cv::circle(img, (armor.lights[i].vertices[0] + armor.lights[i].vertices[1]) / 2, 3, cv::Scalar(i==0?255:0 , 0 , i==0?0:255),-1);
                // cv::putText(img, std::to_string(((armor.lights[i].vertices[0] + armor.lights[i].vertices[1]) / 2).x), (armor.lights[i].vertices[0] + armor.lights[i].vertices[1]) / 2 - cv::Point2f(20,30),
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
                // cv::putText(img, std::to_string(((armor.lights[i].vertices[0] + armor.lights[i].vertices[1]) / 2).y), (armor.lights[i].vertices[0] + armor.lights[i].vertices[1]) / 2 - cv::Point2f(20,10),
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
                cv::circle(img, armor.lights[i].center,3,cv::Scalar(255,0,255),-1);

                // cv::putText(img, std::to_string(armor.lights[i].center.x), armor.lights[i].center - cv::Point2f(20,10),
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
                    
                // cv::putText(img, std::to_string(armor.lights[i].center.y), armor.lights[i].center - cv::Point2f(20,-10),
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);

                cv::circle(img, (armor.lights[i].vertices[2] + armor.lights[i].vertices[3]) / 2, 3, cv::Scalar(i==0?0:255 , 0 , i==0?255:0),-1);
                
                // cv::putText(img, std::to_string(((armor.lights[i].vertices[2] + armor.lights[i].vertices[3]) / 2).x), (armor.lights[i].vertices[2] + armor.lights[i].vertices[3]) / 2 - cv::Point2f(20,-10),
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
                // cv::putText(img, std::to_string(((armor.lights[i].vertices[2] + armor.lights[i].vertices[3]) / 2).y), (armor.lights[i].vertices[2] + armor.lights[i].vertices[3]) / 2 - cv::Point2f(20,-30),
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);

            }
            //画出装甲板的最小外接矩形
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.box.vertices[i], armor.box.vertices[(i+1)%4], cv::Scalar(255, 0, 0), 2);

            //标出装甲板颜色
            cv::putText(img, armor.type == 0 ? "Blue" : "Red", (armor.lights[0].center + armor.lights[1].center) / 2,
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            
            //标出装甲板ID
            cv::putText(img, std::to_string(armor.id), armor.box.vertices[3],
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            cv::putText(img, std::to_string(armor.id_car), armor.box.vertices[2],
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            // std::cout<<armor.predict_position<<std::endl;
            Transformation::projectWorldPointToImage(armor.predict_position,img);

        }
    }
    Armor() = default;
    Armor(MyRotatedRect light1, MyRotatedRect light2, int type): type(type)
    {
        if(light1.center.x == light2.center.x)
            return;
        
        //确保lights[0]在左边，lights[1]在右边
        lights[0] = light1.center.x < light2.center.x ? light1 : light2;
        lights[1] = light1.center.x < light2.center.x ? light2 : light1;
        //保证lights的vertices顺序
        
        reArrangeVertices(lights[0].vertices);
        reArrangeVertices(lights[1].vertices);

        //计算装甲板的最小外接矩形
        box.center = (lights[0].center + lights[1].center) / 2;
        box.size.width = cv::norm(lights[0].center - lights[1].center);
        box.size.height = lights[0].size.height + lights[1].size.height;
        box.angle = std::atan2(lights[1].center.y - lights[0].center.y, lights[1].center.x - lights[0].center.x) * 180 / CV_PI;
        //计算装甲板的四个顶点
        cv::Point2f direction = (lights[1].center - lights[0].center) / cv::norm(lights[1].center - lights[0].center);

        cv::Point2f perpendicular = lights[0].size.area() < lights[1].size.area() ? \
                    ((lights[0].vertices[3] - lights[0].vertices[0]) / cv::norm(lights[0].vertices[3] - lights[0].vertices[0])) : 
                    ((lights[1].vertices[3] - lights[1].vertices[0]) / cv::norm(lights[1].vertices[0] - lights[1].vertices[3]));

        float max_width = std::max(lights[0].size.width, lights[1].size.width);
        float min_height = std::min(lights[0].size.height, lights[1].size.height);
        box.vertices[0] = lights[0].center - perpendicular * max_width * 1.2f + direction * min_height * 1.0f;
        box.vertices[3] = lights[0].center + perpendicular * max_width * 1.2f + direction * min_height * 1.0f;
        box.vertices[1] = lights[1].center - perpendicular * max_width * 1.2f - direction * min_height * 1.0f;
        box.vertices[2] = lights[1].center + perpendicular * max_width * 1.2f - direction * min_height * 1.0f;

        //裁减装甲板图像
        cv::Mat mask = cv::Mat::zeros(box.size.height, box.size.width, CV_8UC1);
        cv::Point2f src_points[4] = {
            box.vertices[0], box.vertices[1], box.vertices[2], box.vertices[3]
        };
        //转换为CNN输入大小
        cv::Point2f dst_points[4] = {
            {0,0},{CNN_INPUT_SIZE,0},{CNN_INPUT_SIZE,CNN_INPUT_SIZE},{0,CNN_INPUT_SIZE}
        };
        cv::Mat perspectiveTransform = cv::getPerspectiveTransform(src_points, dst_points);
        cv::warpPerspective(*frame, box_image, perspectiveTransform, cv::Size(CNN_INPUT_SIZE, CNN_INPUT_SIZE));
        //(去除1通道)灰度化
        if(type == 0) //蓝色装甲板去除蓝色通道
        {
            box_image.forEach<cv::Vec3b>([](cv::Vec3b &pixel, const int * position) -> void {
                pixel[0] = 0; //去除蓝色通道
            });
        }
        else //红色装甲板
        {
            box_image.forEach<cv::Vec3b>([](cv::Vec3b &pixel, const int * position) -> void {
                pixel[2] = 0; //去除红色通道
            });
        }
        cv::cvtColor(box_image, box_image, cv::COLOR_BGR2GRAY);
        //计算全图平均亮度
        double mean_brightness = cv::mean(box_image)[0];
        //通过乘上系数使得平均亮度达到100
        double alpha = 100.0 / mean_brightness;
        box_image.convertTo(box_image, -1, alpha, 0);

        //保存图片到本地，测试用
        //cv::imwrite("../armorImage/" + std::to_string(totalArmor++) + ".jpg", box_image);

        id = CNN::predict(box_image);

        // cv::namedWindow("box_image" + std::to_string(id), cv::WINDOW_NORMAL);
        // cv::imshow("box_image" + std::to_string(id), box_image);

        Transformation::solvePnP(std::vector<cv::Point2d>{(lights[0].vertices[0] + lights[0].vertices[1])/2.0,(lights[0].vertices[3]+lights[0].vertices[2])/2.0,
                                                            (lights[1].vertices[0] + lights[1].vertices[1])/2.0,(lights[1].vertices[3]+lights[1].vertices[2])/2.0},
                                 rvec, tvec);

        //计算装甲板距离
        center_distance = cv::norm(tvec);
        //转换到世界坐标系
        Transformation::transformToWorldCoord(rvec, tvec, tvec_world, rmat_world);
        // std::cout<<"rmat_world:"<<std::endl<<rmat_world<<std::endl;
        //计算世界系下的rvec的x分量的指向
        cv::Mat armor_x = rmat_world.inv() * (cv::Mat_<double>(3,1) << 1.0, 0.0, 0.0);
        angle_world = -std::atan2(armor_x.at<double>(2,0), armor_x.at<double>(0,0)) * 180.0 / CV_PI;
        //预测
        // std::cout<<"angle_world: "<<angle_world<<std::endl;
    } 
    //获取装甲板ID，使用CNN识别，用DNN导入模型
    int getId() const
    {
        return id;
    }
    ~Armor() = default;
    
    cv::Point getCenter() const
    {
        return box.center;
    }
    
    static void setFrame(std::shared_ptr<cv::Mat> f)
    {
        Armor::frame = f;
    }

    int id_car = -1;
private:
    bool is_detected;
    int type; // 0 for blue, 1 for red
    int id;
    inline static std::shared_ptr<cv::Mat> frame = nullptr; //当前帧图像
    cv::Mat box_image; //装甲板的图像，用于CNN识别 
    MyRotatedRect box; //装甲板的最小外接矩形
    MyRotatedRect lights[2];
    cv::Mat rvec;   //旋转向量
    cv::Mat tvec;   //平移向量
    double center_distance;
    inline static size_t totalArmor = 0; //总识别到的装甲板数

    double vx = 0.0, vy = 0.0, vz = 0.0; //速度
    double w; //角速度
    double r; //转弯半径
    cv::Point3d predict_position; //预测位置
    cv::Mat tvec_world; //世界坐标系下的tvec
    cv::Mat rmat_world; //世界坐标系下的旋转矩阵
    double angle_world; //世界坐标系下的rvec的x分量的指向
};

#endif