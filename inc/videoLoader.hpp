#ifndef VIDEOLOADER_HPP
#define VIDEOLOADER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>


namespace VideoLoader
{
    constexpr int FRAME_BUFFER_MAX = 60; //最大缓存帧数

    enum class InputType
    {
        VIDEO,
        CAMERA
    };

    class VideoInput
    {
    public:
        VideoInput(InputType type, const std::string& source)
        {
            if(type == InputType::VIDEO)
            {
                cap.open(source);
                if(!cap.isOpened())
                {
                    throw std::runtime_error("Failed to open video file: " + source);
                }
            }
            else if(type == InputType::CAMERA)
            {
                int cam_index = std::stoi(source);
                cap.open(cam_index);
                if(!cap.isOpened())
                {
                    throw std::runtime_error("Failed to open camera index: " + source);
                }
            }
            reader_thread = std::thread(&VideoInput::Read2Buffer, this);
        }
        // Ensure reader thread is joined on destruction
        ~VideoInput()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                running = false;
            }
            cv_.notify_all();
            if(reader_thread.joinable())
                reader_thread.join();
        }
        bool isOpened()
        {
            return cap.isOpened();
        }
        bool getFrame(cv::Mat& frame)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if(frame_buffer.empty())
            {
                // no frame available right now
                frame = cv::Mat();
                return false;
            }
            frame = frame_buffer.front();
            frame_buffer.pop_front();
            // notify reader that there's room
            cv_.notify_one();
            return true;
        }
        bool getFrame(cv::Mat& frame, int index)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(index < static_cast<int>(frame_buffer.size()))
            {
                frame = frame_buffer[index];
                return true;
            }
            return false;
        }
        bool isEnd()
        {
            return !running && frame_buffer.empty();
        }
        void release()
        {
            cap.release();
        }
        double get(int propId)
        {
            return cap.get(propId);
        }
    private:
        void Read2Buffer()
        {
            while(true)
            {
                // read one frame
                cv::Mat frame;
                if(!cap.read(frame))
                {
                    // end of stream or read error
                    std::lock_guard<std::mutex> lock(mutex_);
                    running = false;
                    cv_.notify_all();
                    break;
                }

                // push into bounded buffer
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&]{ return frame_buffer.size() < FRAME_BUFFER_MAX || !running; });
                if(!running)
                {
                    break;
                }
                frame_buffer.push_back(frame);
                lock.unlock();
                cv_.notify_one();
            }
            std::cout<<"Video reading thread ended."<<std::endl;
        }
        bool running = true;
        cv::VideoCapture cap;
        std::deque<cv::Mat> frame_buffer;
    
        std::mutex mutex_;
        std::condition_variable cv_;
        std::thread reader_thread;
    };
};


#endif