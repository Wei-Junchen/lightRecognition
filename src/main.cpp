#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <chrono>
#include "recognition.hpp"
#include "paramConfig.hpp"

int main(int argc,char** argv)
{
    if(argc<2)
    {
        std::cerr << "Usage: " << argv[0] << " <video_path>" << std::endl;
        return 1;
    }
    cv::VideoCapture cap(argv[1]);
    if(!cap.isOpened())
    {
        std::cerr << "Error opening video file: " << argv[1] << std::endl;
        return 1;
    }

    HsvParamManager hsvParamManager({"Red","Blue"});
    HsvParam hsv_param_blue, hsv_param_red;
    hsvParamManager.getParam(hsv_param_red, "Red");
    hsvParamManager.getParam(hsv_param_blue, "Blue");

#if PARAM_CONFIG_DEBUG
    //创建窗口和滑动条
    //hsvParamManager.createTrackbars("Red");
    hsvParamManager.createTrackbars("Blue");
#endif

    std::shared_ptr<cv::Mat> frame = std::make_shared<cv::Mat>();
    Armor::setFrame(frame);
    ArmorFilter armorFilter;
    auto blue_recong = Recognition::createRecognition(hsv_param_blue, frame, 0);
    auto red_recong = Recognition::createRecognition(hsv_param_red, frame, 1);

    auto startTime = std::chrono::high_resolution_clock::now();
    double fps = cap.get(cv::CAP_PROP_FPS);

    while(true)
    {
#if PARAM_CONFIG_DEBUG
        //检查是否有参数更新
        hsvParamManager.getParam(hsv_param_red, "Red");
        hsvParamManager.getParam(hsv_param_blue, "Blue");
        blue_recong->updateParam(hsv_param_blue);
        // red_recong->updateParam(hsv_param_red);
#endif
        cap >> *frame;
        if(frame->empty())
            break;

        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        startTime = currentTime;


        cv::Mat mask;
        Recognition::RecognitionAll();
        std::vector<Armor> armors;
        blue_recong->getArmor(armors);
        red_recong->getArmor(armors);
        armorFilter.updateArmor(armors);

        Armor::DrawArmor(*frame, armorFilter.getFocusedArmors());

        if (elapsed.count() > 0)
        {
            //add fps text to frame
            cv::putText(*frame, "True FPS: " + std::to_string(1.0 / elapsed.count()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        }

        cv::imshow("Frame", *frame);

        if(cv::waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
        {
            std::cout << "Esc key is pressed by user. Stopping the video" << std::endl;
            break;
        }
        // cv::waitKey(0);
    }
    return 0;
}
