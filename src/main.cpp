#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <chrono>
#include "recognition.hpp"
#include "paramConfig.hpp"
#include "videoLoader.hpp"
#include "filter.hpp"
#include "tracker.hpp"
#include "shooter.hpp"

int main(int argc,char** argv)
{
    if(argc<2)
    {
        std::cerr << "Usage: " << argv[0] << " <video_path>" << std::endl;
        return 1;
    }
    VideoLoader::VideoInput videoInput(VideoLoader::InputType::VIDEO, argv[1]);
    if(!videoInput.isOpened())
    {
        std::cerr << "Error opening video file: " << argv[1] << std::endl;
        return 1;
    }
    //load setting key-value pairs
    HsvParamManager hsvParamManager({"Red","Blue"});
    HsvParam hsv_param_blue, hsv_param_red;
    hsvParamManager.getParam(hsv_param_red, "Red");
    hsvParamManager.getParam(hsv_param_blue, "Blue");

#if PARAM_CONFIG_DEBUG
    //创建窗口和滑动条
    if(setting.isDebug())
    {
        if(setting.getValue("target").has_value())
        {
            auto target = setting.getValue("target").value();
            if(std::find(target.begin(), target.end(), "blue") != target.end())
                hsvParamManager.createTrackbars("Blue");
            if(std::find(target.begin(), target.end(), "red") != target.end())
                hsvParamManager.createTrackbars("Red");
        }
    }
#endif

    std::shared_ptr<cv::Mat> frame = std::make_shared<cv::Mat>();
    Armor::setFrame(frame);
    std::shared_ptr<Recognition> blue_recong, red_recong;
    if(setting.getValue("target").has_value())
    {
        auto target = setting.getValue("target").value();
        if(std::find(target.begin(), target.end(), "blue") != target.end())
            blue_recong = Recognition::createRecognition(hsv_param_blue, frame, 0);
        if(std::find(target.begin(), target.end(), "red") != target.end())
            red_recong = Recognition::createRecognition(hsv_param_red, frame, 1);
    }
    auto startTime = std::chrono::high_resolution_clock::now();
    double fps = videoInput.get(cv::CAP_PROP_FPS);

    while(true)
    {
#if PARAM_CONFIG_DEBUG
        //检查是否有参数更新
        if(setting.isDebug())
        {
            if(setting.getValue("target").has_value())
            {
                auto target = setting.getValue("target").value();
                if(std::find(target.begin(), target.end(), "blue") != target.end())
                {
                    hsvParamManager.getParam(hsv_param_blue, "Blue");
                    blue_recong->updateParam(hsv_param_blue);
                }
                if(std::find(target.begin(), target.end(), "red") != target.end())
                {
                    hsvParamManager.getParam(hsv_param_red, "Red");
                    red_recong->updateParam(hsv_param_red);
                }
            }
        }
#endif
        videoInput.getFrame(*frame);
        if(videoInput.isEnd())
            break;
        if(frame->empty())
            continue;

        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime - startTime;
        startTime = currentTime;


        cv::Mat mask;
        Recognition::RecognitionAll();
        std::vector<Armor> armors;

        if(setting.getValue("target").has_value())
        {
            auto target = setting.getValue("target").value();
            if(std::find(target.begin(), target.end(), "blue") != target.end())
                blue_recong->getArmor(armors);
            if(std::find(target.begin(), target.end(), "red") != target.end())
                red_recong->getArmor(armors);
        }
        //默认识别所有颜色
        else
        {
            blue_recong->getArmor(armors);
            red_recong->getArmor(armors);
        }
        
        ArmorTracker::TrackedArmor::HungarianMatch(armors);
        //predict位置更新，dt=0.5s
        Shooter::CapableTrajectory::computeTrajectory();
        for(auto & traj : Shooter::CapableTrajectory::trajectories)
        {
            ArmorTracker::trackedArmors[traj.getTrackedIndex()].predictArmors(traj.getDt());
        }
        ArmorTracker::getTrackedArmors(armors);
        
        Armor::DrawArmor(*frame, armors);

        if (elapsed.count() > 0)
        {
            //add fps text to frame
            cv::putText(*frame, "True FPS: " + std::to_string(1.0 / elapsed.count()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        }

        cv::imshow("Frame", *frame);

        if(setting.getValue("playmode").has_value() && setting.getValue("playmode").value()[0] == "normal")
        {
            uchar key = (uchar)cv::waitKey(1);
            if(key == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
            {
                std::cout << "Esc key is pressed by user. Stopping the video" << std::endl;
                break;
            }
            else if(key == 'p') //wait for 'p' key press for pause
            {
                cv::waitKey(0);
            }
        }
        else
            cv::waitKey(0);
    }
    return 0;
}
