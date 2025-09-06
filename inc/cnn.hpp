#ifndef CNN_HPP
#define CNN_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define CNN_INPUT_SIZE 36
#define CLASS_NUM 11 // Number of classes in the model (10 digits + 1 for no digit)
#define ONNX_MODEL_PATH "model.onnx"

class CNN
{
public:
    CNN()
    {
        // Load the pre-trained model
        net = cv::dnn::readNetFromONNX(ONNX_MODEL_PATH);
    }

    int predict(const cv::Mat& input)
    {
        // Preprocess the input image
        cv::Mat blob = cv::dnn::blobFromImage(input, 1.0, cv::Size(CNN_INPUT_SIZE, CNN_INPUT_SIZE), cv::Scalar(104, 117, 123));
        net.setInput(blob);

        // Forward pass
        cv::Mat output = net.forward();

        // Get the predicted class
        // int classId = cv::dnn::NMSBoxes(output, std::vector<float>(), 0.5, 0.4).empty() ? -1 : cv::dnn::NMSBoxes(output, std::vector<float>(), 0.5, 0.4)[0];
        // return classId;
        return 0;
    }

private:
    cv::dnn::Net net;
};

#endif