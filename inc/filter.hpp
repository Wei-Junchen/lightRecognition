#ifndef FILTER_HPP
#define FILTER_HPP

#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace ArmorFilter
{

float dt = 0.04; //时间间隔s

//定义状态转移矩阵
cv::Mat transitionMatrix = (cv::Mat_<float>(6,6) <<
    1,0,0,dt,0,0,
    0,1,0,0,dt,0,
    0,0,1,0,0,dt,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1);

//定义测量矩阵
cv::Mat measurementMatrix = (cv::Mat_<float>(3,6) <<
    1,0,0,0,0,0,
    0,1,0,0,0,0,
    0,0,1,0,0,0);
cv::Mat measurementNoiseCov = (cv::Mat_<float>(3,3) <<
    1,0,0,
    0,1,0,
    0,0,1);

//定义协方差噪声矩阵
cv::Mat processNoiseCov = (cv::Mat_<float>(6,6) <<
    1,0,0,0,0,0,
    0,1,0,0,0,0,
    0,0,1,0,0,0,
    0,0,0,1,0,0,
    0,0,0,0,1,0,
    0,0,0,0,0,1);

}

class KalmanFilter
{
public:
    KalmanFilter(int const stateDim, int const measureDim,cv::Mat A, cv::Mat H,cv::Mat state,cv::Mat Q, cv::Mat R,cv::Mat P) : stateDim_(stateDim), measureDim_(measureDim)
    {
        if(stateDim <= 0 || measureDim <= 0 || stateDim < measureDim)
            throw std::invalid_argument("Invalid dimensions for Kalman Filter");
        A_ = A.empty() ? cv::Mat::eye(stateDim, stateDim, CV_32F) : A.clone();
        H_ = H.empty() ? cv::Mat::zeros(measureDim, stateDim, CV_32F) : H.clone();
        state_ = state.empty() ? cv::Mat::zeros(stateDim, 1, CV_32F) : state.clone();
        Q_ = Q.empty() ? cv::Mat::eye(stateDim, stateDim, CV_32F) * 0.01 : Q.clone();
        R_ = R.empty() ? cv::Mat::eye(measureDim, measureDim, CV_32F) * 0.1 : R.clone();
        P_ = P.empty() ? cv::Mat::eye(stateDim, stateDim, CV_32F) : P.clone();
    }
    KalmanFilter(const KalmanFilter&) = default;
    void UpdateA(const cv::Mat& A)
    {
        if(A.rows != stateDim_ || A.cols != stateDim_)
            throw std::invalid_argument("Invalid dimension for state transition matrix A");
        A_ = A.clone();
    }
    void UpdateP(const cv::Mat& P)
    {
        if(P.rows != stateDim_ || P.cols != stateDim_)
            throw std::invalid_argument("Invalid dimension for error covariance matrix P");
        P_ = P.clone();
    }
    void UpdateState(const cv::Mat& state)
    {
        if(state.rows != stateDim_ || state.cols != 1)
            throw std::invalid_argument("Invalid dimension for state vector");
        state_ = state.clone();
    }
    void getState(cv::Mat& state) const
    {
        state = state_.clone();
    }
    //仅仅进行预测，不进行更新
    cv::Mat PredictWithoutUpdate()
    {
        return A_*state_;
    }
    //无观测值修正，直接进行预测和更新
    cv::Mat PredictAndUpdate()
    {
        //预测
        cv::Mat pred = A_*state_;
        cv::Mat P_pre = A_*P_*A_.t() + Q_;
        //计算卡尔曼增益
        cv::Mat S = H_*P_*H_.t() + R_;
        cv::Mat K = P_*H_.t()*S.inv();
        //更新状态
        state_ = pred;
        //更新协方差
        cv::Mat I = cv::Mat::eye(state_.rows, state_.rows, CV_32F);
        P_ = (I - K*H_)*P_pre;
        return state_;
    }
    cv::Mat PredictAndUpdate(cv::Mat z)
    {
        //预测
        cv::Mat pred = A_*state_;
        cv::Mat P_pre = A_*P_*A_.t() + Q_;
        //计算卡尔曼增益
        cv::Mat S = H_*P_*H_.t() + R_;
        cv::Mat K = P_*H_.t()*S.inv();
        //更新状态
        state_ = pred + K*(z - H_*pred);
        //更新协方差
        cv::Mat I = cv::Mat::eye(state_.rows, state_.rows, CV_32F);
        P_ = (I - K*H_)*P_pre;
        return state_;
    }
private:
    int stateDim_;
    int measureDim_;
    //认为可更新的量：
    cv::Mat state_;
    cv::Mat A_;
    cv::Mat P_;
    //认为不可更新的量：
    cv::Mat H_;
    cv::Mat Q_;
    cv::Mat R_;
};

#endif