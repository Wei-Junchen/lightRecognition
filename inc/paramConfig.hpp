#ifndef PARAMCONFIG_HPP
#define PARAMCONFIG_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <optional>

#define PARAM_CONFIG_DEBUG true
#define COLOR_NUM_MAX 2

struct HsvParam
{
    int hue_min,
        hue_max,
        saturation_min,
        saturation_max,
        value_min,
        value_max;
};

class Setting
{
public:
    Setting(const std::string& file = "../config/config.yml"): filename(file)
    {
        try
        {
            fs.open(filename, cv::FileStorage::READ);
            if(!fs.isOpened())
            {
                throw std::runtime_error("Failed to open config file");
            }
            //load all key-value pairs into map
            cv::FileNode root = fs.root();
            for(auto it = root.begin(); it != root.end(); ++it)
            {
                std::string key = (*it).name();
                cv::FileNode valueNode = *it;
                if(valueNode.type() == cv::FileNode::SEQ)
                {
                    std::vector<std::string> values;
                    for(auto vit = valueNode.begin(); vit != valueNode.end(); ++vit)
                    {
                        values.push_back((std::string)*vit);
                    }
                    configMap[key] = values;
                }
                else if(valueNode.type() == cv::FileNode::STRING || 
                        valueNode.type() == cv::FileNode::INT || 
                        valueNode.type() == cv::FileNode::REAL)
                {
                    // 处理标量值（字符串、整数、浮点数）
                    std::vector<std::string> values;
                    values.push_back((std::string)valueNode);
                    configMap[key] = values;
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cerr<<"Config file "<<filename<<" not found, creating new!"<<std::endl;
            fs.release();
            fs.open(filename, cv::FileStorage::WRITE);
            fs.release();
            fs.open(filename, cv::FileStorage::READ);
            if(!fs.isOpened())
            {
                std::cerr<<"Config file "<<filename<<" not found, creating new!"<<std::endl;
                fs.release();
                fs.open(filename, cv::FileStorage::WRITE);
                fs.release();
            }
        }
        //print all loaded key-value pairs
        for(const auto& [key, value] : configMap)
        {
            std::cout << "Loaded config: " << key << " : ";
            for(const auto& v : value)
            {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
    }
    std::optional<std::vector<std::string>> getValue(const std::string& key)
    {
        if(configMap.find(key) != configMap.end() && !configMap[key].empty())
        {
            return configMap[key];
        }
        return std::nullopt;
    }
    bool isDebug()
    {
        static bool debug = getValue("debug").value_or(std::vector<std::string>{"false"})[0] == "true";
        return debug;
    }
    bool isTerminalOutput()  
    {
        static bool terminal_output = getValue("terminal_output").value_or(std::vector<std::string>{"false"})[0] == "true";
        return terminal_output;
    }
private:
    cv::FileStorage fs;
    std::string filename;
    std::map<std::string,std::vector<std::string>> configMap;
};

inline Setting setting;

//Hsv参数管理类，可以读取和保存多个颜色的Hsv参数
class HsvParamManager
{
public:
    HsvParamManager(std::vector<std::string_view> colorRangeNames): filename("../config/hsv_params")
    {
        for(auto name : colorRangeNames)
        {
            getParam(std::string(name));
        }
    }
    void getParam(const std::string& index)
    {
        std::cout<<"Loading HsvParam from index "<<index<<std::endl;
        cv::FileStorage fs(filename + "_" +index + ".yml", cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            std::cerr<<"HsvParam file for index "<<index<<" not found, creating default!"<<std::endl;
            fs.release();
            //make default
            saveParam({0, 360, 0, 255, 0, 255}, index);
            return;
        }
        //load into vector
        HsvParam param;
        fs["hue_min"] >> param.hue_min;
        fs["hue_max"] >> param.hue_max;
        fs["saturation_min"] >> param.saturation_min;
        fs["saturation_max"] >> param.saturation_max;
        fs["value_min"] >> param.value_min;
        fs["value_max"] >> param.value_max;
        params[param_count++] = std::make_pair(index, param);
        fs.release();
    }
    void getParam(HsvParam& param, const std::string& index) const
    {
        for(size_t i = 0; i < param_count; i++)
        {
            if(params[i].first == index)
            {
                param = params[i].second;
                return;
            }
        }
        std::cerr<<"HsvParam for index "<<index<<" not found!"<<std::endl;
    }
    //保存参数到配置文件和vector中
    void saveParam(const HsvParam& param, const std::string& index)
    {
        std::cout<<"Saving HsvParam to index "<<index<<std::endl;
        cv::FileStorage fs(filename + "_" + index + ".yml", cv::FileStorage::WRITE);
        for(auto& p : params)
        {
            if(p.first == index)
            {
                p.second = param; //update existing
                fs << "hue_min" << param.hue_min;
                fs << "hue_max" << param.hue_max;
                fs << "saturation_min" << param.saturation_min;
                fs << "saturation_max" << param.saturation_max;
                fs << "value_min" << param.value_min;
                fs << "value_max" << param.value_max;
                fs.release();
                return;
            }
        }
        //not found, add new
        params[param_count++] = std::make_pair(index, param);
        fs << "hue_min" << param.hue_min;
        fs << "hue_max" << param.hue_max;
        fs << "saturation_min" << param.saturation_min;
        fs << "saturation_max" << param.saturation_max;
        fs << "value_min" << param.value_min;
        fs << "value_max" << param.value_max;
        fs.release();
    }

// Only compile the following code if PARAM_CONFIG_DEBUG is defined
#if PARAM_CONFIG_DEBUG
    static void paramSave(int pos, void* userdata)
    {
        HsvParamManager* manager = reinterpret_cast<HsvParamManager*>(userdata);
        if(manager == nullptr)
        {
            std::cerr<<"HsvParamManager pointer is null in paramSave!"<<std::endl;
            return;
        }
        if(manager->param_count == 0)
        {
            std::cerr<<"No parameters in HsvParamManager in paramSave!"<<std::endl;
            return;
        }
        //find which trackbar changed
        for(size_t i = 0; i < manager->param_count; i++)
        {
            //确定是哪个参数的滑动条
            if(!manager->param_debug[i] || !manager->param_debug_init[i])
                continue;
            std::string window_name = manager->params[i].first + "_Setting";
            int hue_min = cv::getTrackbarPos("hue_min", window_name);
            int hue_max = cv::getTrackbarPos("hue_max", window_name);
            int saturation_min = cv::getTrackbarPos("saturation_min", window_name);
            int saturation_max = cv::getTrackbarPos("saturation_max", window_name);
            int value_min = cv::getTrackbarPos("value_min", window_name);
            int value_max = cv::getTrackbarPos("value_max", window_name);
            HsvParam param = {hue_min, hue_max, saturation_min, saturation_max, value_min, value_max};
            manager->saveParam(param, manager->params[i].first);
            manager->pendingParamChange[i] = true;
        }
    }
    void createTrackbars(const std::string& index)
    {
        //find index's index
        size_t i;
        for(i = 0; i < param_count; i++)
        {
            if(params[i].first == index)
            {
                param_debug[i] = true;
                break;
            }
        }
        if(i == param_count)
        {
            std::cerr<<"HsvParam for index "<<index<<" not found, cannot create trackbars!"<<std::endl;
            return;
        }

        std::string window_name = index + "_Setting";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("hue_min", window_name, nullptr, 180, paramSave,this);
        cv::createTrackbar("hue_max", window_name, nullptr, 180, paramSave,this);
        cv::createTrackbar("saturation_min", window_name, nullptr, 255, paramSave,this);
        cv::createTrackbar("saturation_max", window_name, nullptr, 255, paramSave,this);
        cv::createTrackbar("value_min", window_name, nullptr, 255, paramSave,this);
        cv::createTrackbar("value_max", window_name, nullptr, 255, paramSave,this);

        //set initial values
        cv::setTrackbarPos("hue_min", window_name, params[i].second.hue_min);
        cv::setTrackbarPos("hue_max", window_name, params[i].second.hue_max);
        cv::setTrackbarPos("saturation_min", window_name, params[i].second.saturation_min);
        cv::setTrackbarPos("saturation_max", window_name, params[i].second.saturation_max);
        cv::setTrackbarPos("value_min", window_name, params[i].second.value_min);
        cv::setTrackbarPos("value_max", window_name, params[i].second.value_max);

        param_debug_init[i] = true;
        //create window
        cv::Mat empty = cv::Mat::zeros(1, 1, CV_8UC1);
        cv::imshow(window_name, empty);
    }

    bool pendingParamChange[COLOR_NUM_MAX] = {false};
#endif

private:
    const std::string filename;
    std::pair<std::string, HsvParam> params[COLOR_NUM_MAX];
#if PARAM_CONFIG_DEBUG
    bool param_debug[COLOR_NUM_MAX] = {false};
    bool param_debug_init[COLOR_NUM_MAX] = {false};
#endif
    size_t param_count = 0;
};

#endif