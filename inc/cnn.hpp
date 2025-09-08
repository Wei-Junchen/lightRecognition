#ifndef CNN_HPP
#define CNN_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define CNN_INPUT_SIZE 28
#define CLASS_NUM 10 // Number of classes in the model (10 digits + 1 for no digit)
#define ONNX_MODEL_PATH "../module/mnist_cnn.onnx"

namespace CNN
{
    cv::dnn::Net net = cv::dnn::readNetFromONNX(ONNX_MODEL_PATH);

    static int predict(const cv::Mat& input)
    {
        // Preprocess the input image
        cv::Mat blob = cv::dnn::blobFromImage(input, 1.0, cv::Size(CNN_INPUT_SIZE, CNN_INPUT_SIZE), cv::Scalar(104, 117, 123));
        net.setInput(blob);

        // Forward pass
        cv::Mat output = net.forward();

        // Get the predicted class
        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(output.reshape(1, 1), nullptr, &confidence, nullptr, &classIdPoint);
        int classId = classIdPoint.x;
        return classId;
    }

};

#endif