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
        
        cv::Point2f tempVertices[4];
        tempVertices[0] = light1.center - perpendicular * max_width * 1.2f;
        tempVertices[1] = light1.center + perpendicular * max_width * 1.2f;
        tempVertices[2] = light2.center + perpendicular * max_width * 1.2f;
        tempVertices[3] = light2.center - perpendicular * max_width * 1.2f;

        //box.vertices顺序重排，确保顺时针旋转
        int x[4],y[4];
        x[0] = 0;
        for(int i = 1; i < 4; i++)
        {
            for(int j = 0; j<i; ++j)
            {
                if(tempVertices[i].x <tempVertices[x[j]].x)
                {
                    for(int k = i; k>j; --k)
                        x[k] = x[k-1];
                    x[j] = i;
                    break;
                }
                if(j == i-1)
                    x[i] = i;
            }    
        }
        y[0] = 0;
        for(int i = 1; i < 4; i++)
        {
            for(int j = 0; j<i; ++j)
            {
                if(tempVertices[i].y <tempVertices[y[j]].y)
                {
                    for(int k = i; k>j; --k)
                        y[k] = y[k-1];
                    y[j] = i;
                    break;
                }
                if(j == i-1)
                    y[i] = i;
            }    
        }

        //先确定左半边,必定是x坐标最小的两个点，再确定上下位置，求交集

        if(x[0] == y[0]) //x[0]是左上
        {
            box.vertices[0] = tempVertices[x[0]];
            box.vertices[3] = tempVertices[x[1]];
            box.vertices[1] = tempVertices[y[1]];
            box.vertices[2] = tempVertices[6 - x[0] - x[1] - y[1]];
        }
        else if(x[0] == y[1]) //x[0]是左上
        {
            box.vertices[0] = tempVertices[x[0]];
            box.vertices[3] = tempVertices[x[1]];
            box.vertices[1] = tempVertices[y[0]];
            box.vertices[2] = tempVertices[6 - x[0] - x[1] - y[0]];
        }
        else if(x[1] == y[0]) //x[1]是左上
        {
            box.vertices[0] = tempVertices[x[1]];
            box.vertices[3] = tempVertices[x[0]];
            box.vertices[1] = tempVertices[y[1]];
            box.vertices[2] = tempVertices[6 - x[0] - x[1] - y[1]];
        }
        else if(x[1] == y[1]) //x[1]是左上
        {
            box.vertices[3] = tempVertices[x[0]];
            box.vertices[0] = tempVertices[x[1]];
            box.vertices[1] = tempVertices[y[0]];
            box.vertices[2] = tempVertices[6 - x[0] - x[1] - y[0]];
        }
        

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

        cv::namedWindow("box_image", cv::WINDOW_NORMAL);
        cv::imshow("box_image", box_image);
        id = getId();
    }
    //获取装甲板ID，使用CNN识别，用DNN导入模型
    int getId()
    {
        return 0;
    }
    ~Armor() = default;
    
    static void setFrame(std::shared_ptr<cv::Mat> f)
    {
        Armor::frame = f;
    }

private:

    int type; // 0 for blue, 1 for red
    int id;
    inline static std::shared_ptr<cv::Mat> frame = nullptr; //当前帧图像
    cv::Mat box_image; //装甲板的图像，用于CNN识别 
    MyRotatedRect box; //装甲板的最小外接矩形
    MyRotatedRect lights[2];
};



#endif