#ifndef RECOGNITION_HPP
#define RECOGNITION_HPP

#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include "paramConfig.hpp"
#include "armor.hpp"

namespace
{
    bool isRectPaired(MyRotatedRect box1, MyRotatedRect box2)
    {
        //计算两个矩形中心点距离
        double center_dist = cv::norm(box1.center - box2.center);
        //计算两个矩形宽度平均值
        double avg_width = (box1.size.width + box2.size.width) / 2.0;
        //计算两个矩形角度差,不用默认的angle属性，因为angle属性在某些情况下会有180度的偏差，而是通过height所在边与水平线的夹角来计算
        double angle_diff = box1.angle - box2.angle;
        //归一化到[-90, 90]范围
        if(angle_diff > 90)
            angle_diff = 180 - angle_diff;
        else if(angle_diff < -90)
            angle_diff = -180 - angle_diff;

        //计算两个矩形中心连线 与水平线夹角
        double center_line_angle = std::atan2(box2.center.y - box1.center.y, box2.center.x - box1.center.x) * 180.0f / CV_PI;
        if(center_line_angle < 0.0f)
            center_line_angle += 180.0f;
            
        //计算两个矩形角度与中心连线夹角差
        double angle_diff1 = box1.angle - center_line_angle;
        //归一化到[-90, 90]范围
        if(angle_diff1 > 90)
            angle_diff1 = 180 - angle_diff1;
        else if(angle_diff1 < -90)
            angle_diff1 = -180 - angle_diff1;
        double angle_diff2 = box2.angle - center_line_angle;
        //归一化到[-90, 90]范围
        if(angle_diff2 > 90)
            angle_diff2 = 180 - angle_diff2;
        else if(angle_diff2 < -90)
            angle_diff2 = -180 - angle_diff2;

        bool isOutputTerminal = setting.isTerminalOutput();
        if(isOutputTerminal)
            std::cout<<"box1: "<<box1.id<<" box2: "<<box2.id<<" :\n";
        //如果考虑车子不会侧翻，那么灯条一定是不会水平的
        if(box1.angle < 10.0f || box1.angle > 170.0f || box2.angle < 10.0f || box2.angle >170.0f)
        {
            if(isOutputTerminal)
                std::cout<<"box angle too small: "<<box1.angle<<","<<box2.angle<<std::endl;
            return false;
        }

        // if(box1.size.width/box1.size.height > 7.5f || box2.size.width/box2.size.height > 7.5f)
        // {
        //     if(isOutputTerminal)
        //         std::cout<<"box aspect ratio too large: "<<box1.size.width/box1.size.height<<","<<box2.size.width/box2.size.height<<std::endl;
        //     return false;
        // }
        //判断两个矩形宽是否相似
        if(std::abs(box1.size.width - box2.size.width) / std::min(box1.size.width, box2.size.width) > 0.7f)
        {
            if(isOutputTerminal)
                std::cout<<"box width not similar: "<<box1.size.width<<","<<box2.size.width<<std::endl;
            return false;
        }

        //判断是否满足匹配条件
        if(center_dist > avg_width * 5.0f || center_dist < avg_width * 1.25f)
        {
            if(isOutputTerminal)
                std::cout<<"box center distance not match: "<<center_dist<<","<<avg_width * 7.0f<<","<<avg_width * 1.5f<<std::endl;
            return false;
        }
        
        if(std::abs(angle_diff) > 10.0f)
        {
            if(isOutputTerminal)
                std::cout<<"box angle difference too large: "<<angle_diff<<std::endl;
            return false;
        }
            
        // if((std::abs(box1.size.area() - box2.size.area()) / std::min(box1.size.area(), box2.size.area())) > 3.0f)
        // {
        //     if(isOutputTerminal)
        //         std::cout<<"box size area not match: "<<box1.size.area()<<","<<box2.size.area()<<std::endl;
        //     return false;
        // }

        if(std::abs(angle_diff1) < 75.0f || std::abs(angle_diff2) < 75.0f)
        {
            if(isOutputTerminal)
                std::cout<<"box angle difference to center line too large: "<<angle_diff1<<","<<angle_diff2<<std::endl;
            return false;
        }
        
        return true;
    }
}

class Recognition
{
public:
    static void RecognitionAll()
    {
        for(auto& inst : instance)
        {
            inst->Recognize();
        }
    }
    // Factory method to create and manage the instance
    static std::shared_ptr<Recognition> createRecognition(HsvParam param, std::shared_ptr<cv::Mat> src,int colorType)
    {
        Recognition newInstance(param, src);
        newInstance.colorType = colorType;
        std::shared_ptr<Recognition> new_instance = std::make_shared<Recognition>(newInstance);
        instance.push_back(new_instance);
        return new_instance;
    }

    // No need to manually manage instance in the destructor
    ~Recognition() = default;

    void Recognize()
    {
        //follow the same order to depose different color
        //hsv空间区间色彩二值化
        if(src_img->empty())
        {
            std::cerr << "Error: Source image is empty!" << std::endl;
            return;
        }
        cv::cvtColor(*src_img, out_img, cv::COLOR_BGR2HSV);
        //if hue-min > hue-max, it means the range crosses the 0 point
        if(hsv_param.hue_min > hsv_param.hue_max)
        {
            cv::Mat lower_hue_range, upper_hue_range, lower_mask, upper_mask;
            //lower part
            cv::inRange(out_img,
                        cv::Scalar(hsv_param.hue_min, hsv_param.saturation_min, hsv_param.value_min),
                        cv::Scalar(hsv_param.hue_max, hsv_param.saturation_max, hsv_param.value_max),
                        lower_mask);
            //upper part
            cv::inRange(out_img,
                        cv::Scalar(0, hsv_param.saturation_min, hsv_param.value_min),
                        cv::Scalar(hsv_param.hue_max, hsv_param.saturation_max, hsv_param.value_max),
                        upper_mask);
            //combine
            cv::bitwise_or(lower_mask, upper_mask, out_img);
        }
        else
        {
            cv::inRange(out_img,
                        cv::Scalar(hsv_param.hue_min, hsv_param.saturation_min, hsv_param.value_min),
                        cv::Scalar(hsv_param.hue_max, hsv_param.saturation_max, hsv_param.value_max),
                        out_img);
        }
#if PARAM_CONFIG_DEBUG
        if(setting.isDebug())
            cv::imshow("After inRange", out_img);
#endif
        //图像进行中值滤波
        cv::medianBlur(out_img, out_img, 3);
        // cv::imshow("After medianBlur", out_img);
        //连通域分析
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(out_img, labels, stats, centroids);
        //删除小连通域
        for(int i = 1; i < num_labels; i++) //从1开始，跳过背景
        {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if(area < 50) //面积小于80的连通域去掉
            {
                out_img.setTo(0, labels == i);
            }
        }
        // cv::imshow("After connectedComponentsWithStats", out_img);
        //封闭区域洞填充
        // cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // cv::morphologyEx(out_img, out_img, cv::MORPH_CLOSE, morph_kernel);
#if PARAM_CONFIG_DEBUG
//         if(setting.isDebug())
//             cv::imshow("After morphologyEx", out_img);
#endif
        std::vector<std::vector<cv::Point>> contours;
        //找出所有轮廓
        cv::findContours(out_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //绘画出所有轮廓
        std::vector<MyRotatedRect> boxes;
        std::vector<bool> used(contours.size(), false);
        int id_counter = 0;
        for(const auto& contour : contours)
        {
            //使用最小外接平行四边形
            cv::RotatedRect box = cv::minAreaRect(contour);
            if(box.size.area() < 20.0f) //面积太小的去掉
                continue;
            MyRotatedRect mybox;
            box.points(mybox.vertices);

            double dist = cv::norm(mybox.vertices[0] - mybox.vertices[1]);
            double dist1 = cv::norm(mybox.vertices[1] - mybox.vertices[2]);
            double angle;
            if(dist > dist1)
                angle = std::atan2(mybox.vertices[0].y - mybox.vertices[1].y, mybox.vertices[0].x - mybox.vertices[1].x) * 180.0f / CV_PI;
            else
                angle = std::atan2(mybox.vertices[1].y - mybox.vertices[2].y, mybox.vertices[1].x - mybox.vertices[2].x) * 180.0f / CV_PI;

            if(angle < 0.0f)
                angle += 180.0f;

            mybox.size = dist > dist1 ? cv::Size2f(dist, dist1) : cv::Size2f(dist1, dist);
            mybox.center = box.center;
            mybox.angle = angle;
            mybox.id = id_counter++;
            boxes.push_back(mybox);

            //画出每个灯条
            // for(int i = 0; i < 4; i++)
            // {
            //     cv::line(*src_img, mybox.vertices[i], mybox.vertices[(i+1)%4], cv::Scalar(0, 255, 255), 2);
            // }
            // cv::putText(*src_img,std::to_string(mybox.id),box.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            //在灯条上显示角度和长短边像素
            // cv::putText(*src_img,
            //             "Angle: " + std::to_string(double(mybox.angle)) + ", H: " + std::to_string(double(mybox.size.height)) + ", W: " + std::to_string(double(mybox.size.width)),
            //             box.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
                
        }

        //对一个装甲板上的两个灯条进行匹配，匹配后不再与其它灯条进行匹配,思路：通过两个平行四边形的中心距离和当前平行四边形的长宽比进行匹配，同时考虑角度
        armors.clear();

        for(int i = 0; i < boxes.size(); i++)
        {
            if(used[i])
                continue;
            for(int j = i + 1; j < boxes.size(); j++)
            {
                if(used[j])
                    continue;

                //匹配条件：中心距离在一定范围内，且高度相近，且角度差小于一定值,面积大小差距不超过5倍
                if(::isRectPaired(boxes[i], boxes[j]))
                {
                    // std::cout << "!!!!Matched Box1: " << boxes[i].center << " with Box2: " << boxes[j].center << std::endl;
                    armors.push_back(Armor(boxes[i], boxes[j], colorType));
                    used[i] = true;
                    used[j] = true;
                    break; //匹配成功后跳出内层循环，避免重复匹配
                }
            }
        }
    }

    void getOutput(cv::Mat& dst) const
    {
        dst = out_img.clone();
    }

    //insert new armor in rear
    void getArmor(std::vector<Armor>& armors)
    {
        for(auto & armor : this->armors)
        {
            armors.push_back(armor);
        }
    }

    // Update HSV parameters 尽量少用
    void updateParam(const HsvParam& param)
    {
        hsv_param = param;
    }

private:
    static inline std::vector<std::shared_ptr<Recognition>> instance;

    std::shared_ptr<cv::Mat> src_img;
    cv::Mat out_img;
    std::vector<Armor> armors;
    HsvParam hsv_param;
    int colorType; // 0 for blue, 1 for red
    Recognition(HsvParam param,std::shared_ptr<cv::Mat> const src): hsv_param(param), src_img(src) {}
};

#endif