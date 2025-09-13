#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>
#include "cnn.hpp"
#include "motion.hpp"

cv::Mat K = (cv::Mat_<double>(3,3) << 
                 1.7774091341308808e+03, 0., 7.1075979428865026e+02, 
                 0.,1.7754170626354828e+03, 5.3472407285624729e+02, 
                 0., 0., 1.);

cv::Mat dicoef = (cv::Mat_<double>(1,5) << 
                 -5.6313426428564950e-01, 1.8301501710641366e-01, 
                 1.9661478907901904e-03, 9.6259122849674621e-04,
                 5.6883803390679100e-01);

cv::Mat objectPoints = (cv::Mat_<double>(6,3) << 
                 -65.0, 55.0, 0.,   //左上
                 -65.0, 0.0, 0.,    //左中
                 -65.0, -55.0, 0.,  //左下
                  65.0, 55.0, 0.,   //右上
                  65.0, 0., 0.,     //右中   
                  65.0, -55.0, 0.); //右下

struct MyRotatedRect
{
    cv::Point2f vertices[4];
    cv::Point2f center;
    cv::Size2f size;
    float angle;
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
};

class Armor
{
    friend class ArmorFilter;
public:
    static void DrawArmor(cv::Mat& img,const std::vector<Armor>& armors)
    {
        for(const auto& armor : armors)
        {
            if(armor.is_detected == false)
                continue;
            //画出两个灯条
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.lights[0].vertices[i], armor.lights[0].vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            for(int i = 0; i < 4; i++)
                cv::line(img, armor.lights[1].vertices[i], armor.lights[1].vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            //中心点
            cv::circle(img, (armor.lights[0].center + armor.lights[1].center) / 2, 6, cv::Scalar(0, 255 , 255));
            for(int i=0;i<2;++i)
            {
                cv::circle(img, (armor.lights[i].vertices[0] + armor.lights[i].vertices[1]) / 2, 3, cv::Scalar(255 , 0 , 255),-1);
                cv::circle(img, armor.lights[i].center,3,cv::Scalar(255,0,255),-1);
                cv::circle(img, (armor.lights[i].vertices[2] + armor.lights[i].vertices[3]) / 2, 3, cv::Scalar(255 , 0 , 255),-1);

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
        cv::Point2f perpendicular(-direction.y, direction.x); //垂直方向单位向量

        float max_width = std::max(lights[0].size.width, lights[1].size.width);
        box.vertices[0] = lights[0].center - perpendicular * max_width * 1.2f + direction * lights[0].size.height * 1.0f;
        box.vertices[3] = lights[0].center + perpendicular * max_width * 1.2f + direction * lights[0].size.height * 1.0f;
        box.vertices[1] = lights[1].center - perpendicular * max_width * 1.2f - direction * lights[1].size.height * 1.0f;
        box.vertices[2] = lights[1].center + perpendicular * max_width * 1.2f - direction * lights[1].size.height * 1.0f;

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
        //通过乘上系数使得平均亮度达到150
        double alpha = 100.0 / mean_brightness;
        box_image.convertTo(box_image, -1, alpha, 0);
        //保存图片到本地，测试用
        //cv::imwrite("../armorImage/" + std::to_string(totalArmor++) + ".jpg", box_image);
        id = getId();
        // cv::namedWindow("box_image" + std::to_string(id), cv::WINDOW_NORMAL);
        // cv::imshow("box_image" + std::to_string(id), box_image);
        cv::solvePnP(objectPoints, std::vector<cv::Point2f>{(lights[0].vertices[0] + lights[0].vertices[1])/2.0,lights[0].center,(lights[0].vertices[3]+lights[0].vertices[2])/2.0,
                                                            (lights[1].vertices[0] + lights[1].vertices[1])/2.0,lights[1].center,(lights[1].vertices[3]+lights[1].vertices[2])/2.0},
                      K, dicoef, rvec, tvec);
        center_distance = cv::norm(tvec);
        //转换到世界坐标系
        cv::Mat R_yaw = (cv::Mat_<double>(3,3)<<
            cos(Motion::yaw * CV_PI / 180.0),0,-sin(Motion::yaw * CV_PI / 180.0),
            0,1,0,
            sin(Motion::yaw * CV_PI / 180.0),0,cos(Motion::yaw * CV_PI / 180.0));  
        cv::Mat R_pitch = (cv::Mat_<double>(3,3)<<
            1,0,0,
            0,cos(Motion::pitch * CV_PI / 180.0),sin(Motion::pitch * CV_PI / 180.0),
            0,-sin(Motion::pitch * CV_PI / 180.0),cos(Motion::pitch * CV_PI / 180.0));
        cv::Mat R_roll = (cv::Mat_<double>(3,3)<<
            cos(Motion::roll * CV_PI / 180.0),sin(Motion::roll * CV_PI / 180.0),0,
            -sin(Motion::roll * CV_PI / 180.0),cos(Motion::roll * CV_PI / 180.0),0,
            0,0,1);
        cv::Mat R = R_yaw * R_pitch * R_roll;
        tvec_world = cv::Mat(R * tvec);
        //把rvec转换为旋转矩阵
        cv::Mat rmat_cam;
        cv::Rodrigues(rvec, rmat_cam);     // rvec → rmat_cam (3x3)
        rmat_world = R * rmat_cam; // 世界系旋转矩阵 (3x3)
        std::cout<<"rmat_world: "<<rmat_world<<std::endl;
        //计算世界系下的rvec的x分量的指向
        cv::Mat tmp;
        rmat_world.col(0).copyTo(tmp); // 世界系rvec (3x1)
        angle_world = std::atan2(tmp.at<double>(2,0), tmp.at<double>(0,0)) * 180.0 / CV_PI;
        std::cout<<"angle_world: "<<angle_world<<std::endl;
        std::cout<<"tvec_world: "<<tvec_world.t()<<std::endl;
    } 
    //获取装甲板ID，使用CNN识别，用DNN导入模型
    int getId()
    {
        return CNN::predict(box_image);
    }
    ~Armor() = default;
    
    static void setFrame(std::shared_ptr<cv::Mat> f)
    {
        Armor::frame = f;
    }

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
    float center_distance;
    inline static size_t totalArmor = 0; //总识别到的装甲板数
    float vx, vy; //速度

    cv::Mat tvec_world; //世界坐标系下的tvec
    cv::Mat rmat_world; //世界坐标系下的旋转矩阵
    double angle_world; //世界坐标系下的rvec的x分量的指向
};


float dt = 0.05; //时间间隔

//定义armor的状态空间((cx,cy,h,w,vx,vy))
cv::Mat state = (cv::Mat_<float>(6,1) << 0,0,0,0,0,0);

//定义状态转移矩阵
cv::Mat transitionMatrix = (cv::Mat_<float>(6,6) <<
    1,0,0,0,dt,0,
    0,1,0,0,0,dt,
    0,0,1,0,0,0,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1);

//定义测量矩阵
cv::Mat measurementMatrix = (cv::Mat_<float>(4,6) <<
    1,0,0,0,0,0,
    0,1,0,0,0,0,
    0,0,1,0,0,0,
    0,0,0,1,0,0);
//定义测量噪声协方差矩阵
cv::Mat measurementNoiseCov = (cv::Mat_<float>(4,4) <<
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1);

//定义协方差噪声矩阵
cv::Mat processNoiseCov = (cv::Mat_<float>(6,6) <<
    1,0,0,0,0,0,
    0,1,0,0,0,0,
    0,0,1,0,0,0,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1);

class ArmorFilter
{
public:
    void updateArmor(std::vector<Armor>& detectedArmors)
    {
        //先对已跟踪的装甲板进行预测
        for(int i=0;i<focusedArmors.size();++i)
        {
            lostCount[i]++;
            cv::KalmanFilter& kf = kalmanFilters[i];
            cv::Mat prediction = kf.predict();
            focusedArmors[i].box.center.x = prediction.at<float>(0);
            focusedArmors[i].box.center.y = prediction.at<float>(1);
            focusedArmors[i].box.size.width = prediction.at<float>(2);
            focusedArmors[i].box.size.height = prediction.at<float>(3);
            focusedArmors[i].vx = prediction.at<float>(4);
            focusedArmors[i].vy = prediction.at<float>(5);
            // std::cout<<"predicted pos: "<<focusedArmors[i].box.center<<std::endl;
            // std::cout<<"predicted v: "<<focusedArmors[i].vx<<","<<focusedArmors[i].vy<<std::endl;
        }
        //将检测到的装甲板与已跟踪的装甲板进行匹配
        std::vector<bool> matched(detectedArmors.size(), false);
        for(auto& armor : focusedArmors)
        {
            float min_distance = 300.0f;
            int min_index = -1;
            for(size_t i = 0; i < detectedArmors.size(); i++)
            {
                if(matched[i]) continue;
                // std::cout<<"center: "<<armor.box.center<<" " <<detectedArmors[i].box.center<<std::endl;
                float distance = cv::norm(armor.box.center - detectedArmors[i].box.center);
                // std::cout<<"distance: "<<distance<<std::endl;
                if(distance < min_distance)
                {
                    min_distance = distance;
                    min_index = i;
                }
            }
            if(min_index != -1)
            {
                lostCount[&armor - &focusedArmors[0]] = 0; //重置丢失计数器
                matched[min_index] = true;
                //加上观测值再次更新kalman
                cv::KalmanFilter& kf = kalmanFilters[&armor - &focusedArmors[0]];

                cv::Mat measurement = (cv::Mat_<float>(4,1) << detectedArmors[min_index].box.center.x,
                                       detectedArmors[min_index].box.center.y,
                                       detectedArmors[min_index].box.size.width,
                                       detectedArmors[min_index].box.size.height);

                cv::Mat estimated = kf.correct(measurement);
                armor.box.center.x = estimated.at<float>(0);
                armor.box.center.y = estimated.at<float>(1);
                armor.box.size.width = estimated.at<float>(2);
                armor.box.size.height = estimated.at<float>(3);
                armor.vx = estimated.at<float>(4);
                armor.vy = estimated.at<float>(5);
                //更新其他信息
                armor.lights[0] = detectedArmors[min_index].lights[0];
                armor.lights[1] = detectedArmors[min_index].lights[1];
                armor.box.vertices[0] = detectedArmors[min_index].box.vertices[0];
                armor.box.vertices[1] = detectedArmors[min_index].box.vertices[1];
                armor.box.vertices[2] = detectedArmors[min_index].box.vertices[2];
                armor.box.vertices[3] = detectedArmors[min_index].box.vertices[3];
                armor.rvec = detectedArmors[min_index].rvec;
                armor.tvec = detectedArmors[min_index].tvec;
                armor.center_distance = detectedArmors[min_index].center_distance;
                armor.id = detectedArmors[min_index].id;
            }
        }
        //删除丢失超过5帧的装甲板
        for(int i = lostCount.size() - 1; i >= 0; i--)
        {
            if(lostCount[i] != 0)
            {
                if(lostCount[i] > 2)
                {
                    lostCount.erase(lostCount.begin() + i);
                    kalmanFilters.erase(kalmanFilters.begin() + i);
                    focusedArmors.erase(focusedArmors.begin() + i);
                }
                //丢失未超过2帧，kalman继续预测
                else
                {
                    cv::KalmanFilter& kf = kalmanFilters[i];
                    cv::Mat prediction = kf.predict();
                    focusedArmors[i].box.center.x = prediction.at<float>(0);
                    focusedArmors[i].box.center.y = prediction.at<float>(1);
                    focusedArmors[i].box.size.width = prediction.at<float>(2);
                    focusedArmors[i].box.size.height = prediction.at<float>(3);
                    focusedArmors[i].vx = prediction.at<float>(4);
                    focusedArmors[i].vy = prediction.at<float>(5);
                    focusedArmors[i].is_detected = false;
                    // focusedArmors[i].lights[0].center.x = prediction.at<float>(0) - focusedArmors[i].box.size.width / 2;
                    // focusedArmors[i].lights[0].center.y = prediction.at<float>(1);
                    // focusedArmors[i].lights[1].center.x = prediction.at<float>(0) + focusedArmors[i].box.size.width / 2;
                    // focusedArmors[i].lights[1].center.y = prediction.at<float>(1);
                    // focusedArmors[i].box.vertices[0] = focusedArmors[i].lights[0].center - cv::Point2f(focusedArmors[i].box.size.width / 2, focusedArmors[i].box.size.height / 2);
                    // focusedArmors[i].box.vertices[3] = focusedArmors[i].lights[0].center + cv::Point2f(focusedArmors[i].box.size.width / 2, focusedArmors[i].box.size.height / 2);
                    // focusedArmors[i].box.vertices[1] = focusedArmors[i].lights[1].center - cv::Point2f(focusedArmors[i].box.size.width / 2, focusedArmors[i].box.size.height / 2);
                    // focusedArmors[i].box.vertices[2] = focusedArmors[i].lights[1].center + cv::Point2f(focusedArmors[i].box.size.width / 2, focusedArmors[i].box.size.height / 2);
                }
            }
            else
            {
                focusedArmors[i].is_detected = true;
            }
        }

        //将未匹配的检测到的装甲板加入跟踪列表
        for(size_t i = 0; i < detectedArmors.size(); i++)
        {
            if(!matched[i])
            {
                //初始化kalman滤波器
                cv::KalmanFilter kf(6,4,0);
                kf.transitionMatrix = transitionMatrix;
                kf.measurementMatrix = measurementMatrix;
                kf.processNoiseCov = processNoiseCov;
                kf.measurementNoiseCov = measurementNoiseCov;
                cv::Mat state = (cv::Mat_<float>(6,1) << detectedArmors[i].box.center.x, detectedArmors[i].box.center.y, 
                                 detectedArmors[i].box.size.width, detectedArmors[i].box.size.height, 0, 0);
                // 初始化statePost
                kf.statePost = state.clone();
                // 初始化后验协方差
                setIdentity(kf.errorCovPost, cv::Scalar::all(1));

                lostCount.push_back(0);
                kalmanFilters.push_back(kf);
                focusedArmors.push_back(detectedArmors[i]);
                // std::cout<<"new armor added, pos:"<<detectedArmors[i].box.center<<std::endl;
            }
        }
    }
    const std::vector<Armor>& getFocusedArmors() const
    {
        return focusedArmors;
    }
private:
    std::vector<Armor> focusedArmors; //当前跟踪的装甲板
    std::vector<cv::KalmanFilter> kalmanFilters; //对应的Kalman滤波器
    std::vector<int> lostCount; //丢失计数器
};

#endif