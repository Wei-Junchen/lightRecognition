#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>
#include "cnn.hpp"

struct MyRotatedRect
{
    cv::Point2f vertices[4];
    cv::Point2f center;
    cv::Size2f size;
    float angle;
};

class Armor
{
public:
    static void DrawArmor(cv::Mat& img, std::vector<Armor>& armors)
    {
        for(const auto& armor : armors)
        {
            //画出两个灯条
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.lights[0].vertices[i], armor.lights[0].vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.lights[1].vertices[i], armor.lights[1].vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            //中心点
            cv::circle(img, (armor.lights[0].center + armor.lights[1].center) / 2, 6, cv::Scalar(0, 255 , 255));
            // //画出装甲板中心连线
            // cv::line(img, armor.lights[0].center, armor.lights[1].center, cv::Scalar(255, 0, 0), 2);
            //画出装甲板的最小外接矩形
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.box.vertices[i], armor.box.vertices[(i+1)%4], cv::Scalar(255, 0, 0), 2);

            //标出装甲板颜色
            cv::putText(img, armor.type == 0 ? "Blue" : "Red", (armor.lights[0].center + armor.lights[1].center) / 2,
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            
        }
    }
    Armor(MyRotatedRect light1, MyRotatedRect light2, int type): type(type)
    {
        lights[0] = light1;
        lights[1] = light2;

        //计算装甲板的最小外接矩形
        box.center = (light1.center + light2.center) / 2;
        box.size.width = cv::norm(light1.center - light2.center);
        box.size.height = light1.size.height + light2.size.height;
        box.angle = std::atan2(light2.center.y - light1.center.y, light2.center.x - light1.center.x) * 180 / CV_PI;
        //计算装甲板的四个顶点
        cv::Point2f direction = (light2.center - light1.center) / cv::norm(light2.center - light1.center);
        cv::Point2f perpendicular(-direction.y, direction.x); //垂直方向单位向量    
        float max_width = std::max(light1.size.width, light2.size.width);
        box.vertices[0] = light1.center - perpendicular * max_width * 1.2f;
        box.vertices[1] = light1.center + perpendicular * max_width * 1.2f;
        box.vertices[2] = light2.center + perpendicular * max_width * 1.2f;
        box.vertices[3] = light2.center - perpendicular * max_width * 1.2f;

        id = getId();
    }
    //获取装甲板ID，使用CNN识别，用DNN导入模型
    int getId()
    {
        //转换为CNN输入大小
        cv::Point2f dst_points[4] = {
            cv::Point(0, 0),
            cv::Point(box.size.width - 1, 0),
            cv::Point(box.size.width - 1, box.size.height - 1),
            cv::Point(0, box.size.height - 1)
        };

        //先透视变换得到正面图像
        cv::Mat perspectiveTransform = cv::getPerspectiveTransform(box.vertices, dst_points);
        cv::Mat warped;
        return 0;
    }
    ~Armor() = default;

private:
    int type; // 0 for blue, 1 for red
    int id;
    MyRotatedRect box; //装甲板的最小外接矩形
    MyRotatedRect lights[2];
};



#endif