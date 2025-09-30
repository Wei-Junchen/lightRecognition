#ifndef FILTER_HPP
#define FILTER_HPP

#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <functional>
#include <Eigen/Dense>

namespace ArmorFilter
{
    double dt = 0.02; //时间间隔s

    //定义状态转移矩阵
    cv::Mat transitionMatrix = (cv::Mat_<double>(6,6) <<
        1,0,0,dt,0,0,
        0,1,0,0,dt,0,
        0,0,1,0,0,dt,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1);

    //定义测量矩阵
    cv::Mat measurementMatrix = (cv::Mat_<double>(3,6) <<
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0);

    cv::Mat measurementNoiseCov = (cv::Mat_<double>(3,3) <<
        10,0,0,
        0,10,0,
        0,0,25);

    //定义协方差噪声矩阵
    cv::Mat processNoiseCov = (cv::Mat_<double>(6,6) <<
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,100,0,0,0,
        0,0,0,10,0,0,
        0,0,0,0,10,0,
        0,0,0,0,0,100);

    // 示例：构造 6x6 加速度驱动 Q（double 计算然后转为 double）
    cv::Mat makeProcessNoiseCov6(double dt,double sigma_a_mm_s2) {
        cv::Mat Q = cv::Mat::zeros(6,6, CV_64F );
        double q11 = pow(dt,4) / 4.0;
        double q12 = pow(dt,3) / 2.0;
        double q22 = pow(dt,2);
        for (int ax=0; ax<3; ++ax) {
            Q.at<double>(ax, ax)       = q11 * sigma_a_mm_s2 * sigma_a_mm_s2;
            Q.at<double>(ax, ax+3)     = q12 * sigma_a_mm_s2 * sigma_a_mm_s2;
            Q.at<double>(ax+3, ax)     = q12 * sigma_a_mm_s2 * sigma_a_mm_s2;
            Q.at<double>(ax+3, ax+3)   = q22 * sigma_a_mm_s2 * sigma_a_mm_s2;
        }
        // convert to double for OpenCV KalmanFilter (if needed)
        cv::Mat Qf;
        Q.convertTo(Qf, CV_32F);
        return Qf;
    }

    namespace EKF
    {
        //state: [px,py,pz,vx,vy,vz,theta,omega,radius]^T
        Eigen::Vector<double,9> f(Eigen::Vector<double,9> state , double delta_t = dt)
        {
            double px = state(0);
            double py = state(1);
            double pz = state(2);
            double vx = state(3);
            double vy = state(4);
            double vz = state(5);
            double th = state(6);
            double w = state(7);
            double r = state(8);
            return (Eigen::Vector<double,9>()<<
                px + vx*delta_t + r*w*cos(th)*delta_t,
                py + vy*delta_t + r*w*sin(th)*delta_t,
                pz + vz*delta_t,
                vx + r*w*cos(th),
                vy + r*w*sin(th),
                vz,
                th + w*delta_t,
                w,
                r).finished();
        }

        //state: [px,py,pz,vx,vy,vz,theta,omega,radius]^T
        Eigen::Matrix<double,9,9> f_jacobian(Eigen::Vector<double,9> state , double delta_t = dt)
        {
            double px = state(0);
            double py = state(1);
            double pz = state(2);
            double vx = state(3);
            double vy = state(4);
            double vz = state(5);
            double th = state(6);
            double w = state(7);
            double r = state(8);
            return (Eigen::Matrix<double,9,9>()<<
                    1,0,0,delta_t,0,0,(-r*w*sin(th))*delta_t, (r*cos(th))*delta_t, (w*cos(th))*delta_t,
                    0,1,0,0,delta_t,0,( r*w*cos(th))*delta_t, (r*sin(th))*delta_t, (w*sin(th))*delta_t,
                    0,0,1,0,0,delta_t,0,0,0,
                    0,0,0,1,0,0,(-r*w*sin(th)), (r*cos(th)), (w*cos(th)),
                    0,0,0,0,1,0,( r*w*cos(th)), (r*sin(th)), (w*sin(th)),
                    0,0,0,0,0,1,0,0,0,
                    0,0,0,0,0,0,1,delta_t,0,
                    0,0,0,0,0,0,0,1,0,
                    0,0,0,0,0,0,0,0,1).finished();
        }

        //measurement: [px,py,pz,theta]^T
        Eigen::Matrix<double,4,9> measurementMatrix = (Eigen::Matrix<double,4,9>() <<
            1,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,0,
            0,0,0,0,0,0,1,0,0).finished();

        Eigen::Matrix<double,4,4> measurementNoiseCov = (Eigen::Matrix<double,4,4>() <<
            10,0,0,0,
            0,10,0,0,
            0,0,25,0,
            0,0,0,(2.0f/180.0f)*CV_PI).finished(); //角度标准差2度

        Eigen::Matrix<double,9,9> processNoiseCov = (Eigen::Matrix<double,9,9>() <<
            1,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,
            0,0,100,0,0,0,0,0,0,
            0,0,0,10,0,0,0,0,0,
            0,0,0,0,10,0,0,0,0,
            0,0,0,0,0,100,0,0,0,
            0,0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,0,50).finished();
    }

}

//N为状态维度，M为观测维度
template<size_t N, size_t M>
class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(Eigen::Vector<double,N> init_state = Eigen::Vector<double,N>::Zero(),
                         Eigen::Matrix<double,N,N> P = Eigen::Matrix<double,N,N>::Identity(),
                         Eigen::Matrix<double,M,N> H = ArmorFilter::EKF::measurementMatrix,
                         Eigen::Matrix<double,N,N> Q = ArmorFilter::EKF::processNoiseCov,
                         Eigen::Matrix<double,M,M> R = ArmorFilter::EKF::measurementNoiseCov,
                         std::function<Eigen::Vector<double,N>(Eigen::Vector<double,N>,double)> f = ArmorFilter::EKF::f,
                         std::function<Eigen::Matrix<double,N,N>(Eigen::Vector<double,N>,double)> F = ArmorFilter::EKF::f_jacobian)
        : state_(init_state), H_(H), P_(P), Q_(Q), R_(R), f_(f), F_(F) {}


    Eigen::Vector<double,N> PredictAndUpdate(Eigen::Vector<double,M> const& z,double dt)
    {
        //计算state先验值
        Eigen::Vector<double,N> state_pre = f_(state_,dt);
        Eigen::Matrix<double,N,N> F = F_(state_,dt);
        //计算协方差P的先验值
        Eigen::Matrix<double,N,N> P_pre = F*P_*F.transpose() + Q_;
        // 计算新息 (Innovation) y = z_k - H * x_k|k-1
        Eigen::Vector<double,M> y = z - H_ * state_pre;
        // 计算新息协方差 S = H * P_k|k-1 * H^T + R
        Eigen::Matrix<double,M,M> S = H_ * P_pre * H_.transpose() + R_;
        //计算卡尔曼增益 K = P_k|k-1 * H^T * S^-1
        Eigen::Matrix<double,N,M> K = P_pre * H_.transpose() * S.inverse();
        //计算后验状态（修正后的状态）x_k|k = x_k|k-1 + K * y
        state_ = state_pre + K * y;
        // 更新后验协方差 P_k|k
        Eigen::Matrix<double,N,N> I = Eigen::Matrix<double,N,N>::Identity();
        // 使用 Joseph 形式更新 P，数值稳定性更好
        P_ = (I - K * H_) * P_pre * (I - K * H_).transpose() + K * R_ * K.transpose();

        // std::cout<<"observation: "<<z.transpose()<<std::endl;
        // std::cout<<"predicted state: "<<state_pre.transpose()<<std::endl;
        // std::cout<<"updated state: "<<state_.transpose()<<std::endl;
        return state_;
    }

    // 整体重置
    void resetState(Eigen::Vector<double,N> const& init_state,
                    Eigen::Matrix<double,N,N> const& P_new = Eigen::Matrix<double,N,N>::Identity())
    {
        state_ = init_state;
        P_ = P_new;
    }

    // 部分重置
    void resetPartialState(const std::vector<size_t>& indices,
                           const Eigen::VectorXd& values,
                           const Eigen::VectorXd& variances)
    {
        assert(indices.size() == values.size());
        assert(indices.size() == variances.size());

        for (size_t k = 0; k < indices.size(); ++k)
        {
            size_t i = indices[k];
            state_(i) = values(k);

            // 协方差矩阵更新：设置新的方差，同时清除相关性
            P_.row(i).setZero();
            P_.col(i).setZero();
            P_(i,i) = variances(k);
        }
    }
private:
    Eigen::Vector<double,N> state_;
    //观测矩阵
    Eigen::Matrix<double,M,N> H_;
    //协方差矩阵
    Eigen::Matrix<double,N,N> P_;
    //过程噪声协方差矩阵
    Eigen::Matrix<double,N,N> Q_;
    //测量噪声协方差矩阵
    Eigen::Matrix<double,M,M> R_;

    //状态更新非线性函数f
    std::function<Eigen::Vector<double,N>(Eigen::Vector<double,N>,double)> f_;
    //状态更新函数f的雅可比矩阵
    std::function<Eigen::Matrix<double,N,N>(Eigen::Vector<double,N>,double)> F_;
};

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
    KalmanFilter(const KalmanFilter& other) : stateDim_(other.stateDim_), measureDim_(other.measureDim_)
    {
        A_ = other.A_.clone();
        H_ = other.H_.clone();
        state_ = other.state_.clone();
        Q_ = other.Q_.clone();
        R_ = other.R_.clone();
        P_ = other.P_.clone();
    }
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
    //矩阵对称钳位函数，用于确保协方差矩阵的对称性和数值稳定性
    static cv::Mat symClamp(const cv::Mat& M_in, double eig_min = 1e-9, double eig_max = 1e12) {
        cv::Mat M;
        M_in.convertTo(M, CV_64F);
        // symmetry
        cv::Mat Mt;
        cv::transpose(M, Mt);
        M = 0.5 * (M + Mt);

        // eigen-decomposition (cv::eigen returns eigenvalues in descending order)
        cv::Mat eigvals, eigvecs;
        cv::eigen(M, eigvals, eigvecs); // eigvals: row vector
        // clamp
        for (int i = 0; i < eigvals.cols; ++i) {
            double v = eigvals.at<double>(0, i);
            if (v < eig_min) eigvals.at<double>(0, i) = eig_min;
            if (v > eig_max) eigvals.at<double>(0, i) = eig_max;
        }
        // reconstruct: M = V^T * diag(eigvals) * V  (OpenCV eigen returns V such that M = V^T diag(eig) V)
        // eigvecs rows are eigenvectors, we need to transpose
        cv::Mat V = eigvecs.t();
        cv::Mat D = cv::Mat::zeros(V.cols, V.cols, CV_64F);
        for (int i = 0; i < eigvals.cols; ++i) D.at<double>(i, i) = eigvals.at<double>(0, i);
        cv::Mat M2 = V * D * V.t();
        return M2;
    }

    //无观测值修正，直接进行预测和更新
    cv::Mat PredictAndUpdate() {
        //转换为double进行计算，确保精度
        cv::Mat A64, state64, P64, Q64;
        A_.convertTo(A64, CV_64F);
        state_.convertTo(state64, CV_64F);
        P_.convertTo(P64, CV_64F);
        Q_.convertTo(Q64, CV_64F);

        cv::Mat pred = A64 * state64;
        cv::Mat P_pre = A64 * P64 * A64.t() + Q64;

        P_pre = symClamp(P_pre); // sym + clamp evalues
        // write back (convert to original type)
        P_pre.convertTo(P_, state_.type());
        pred.convertTo(state_, state_.type());

        return state_.clone();
    }

    //仅仅进行预测，不进行更新
    cv::Mat PredictWithoutUpdate() {
        cv::Mat A64, state64;
        A_.convertTo(A64, CV_64F);
        state_.convertTo(state64, CV_64F);

        cv::Mat pred = A64 * state64;
        // return prediction as same type as state_
        pred.convertTo(pred, state_.type());
        return pred.clone();
    }

    // Predict and update in one (do both commit)
    cv::Mat PredictAndUpdate(const cv::Mat& z) {
        // Predict
        cv::Mat A64, state64, P64, Q64;
        A_.convertTo(A64, CV_64F);
        state_.convertTo(state64, CV_64F);
        P_.convertTo(P64, CV_64F);
        Q_.convertTo(Q64, CV_64F);

        cv::Mat pred = A64 * state64;
        cv::Mat P_pre = A64 * P64 * A64.t() + Q64;

        // Update using P_pre
        cv::Mat H64, R64, z64;
        H_.convertTo(H64, CV_64F);
        R_.convertTo(R64, CV_64F);
        z.convertTo(z64, CV_64F);

        cv::Mat S = H64 * P_pre * H64.t() + R64;
        // regularize S slightly
        for (int i = 0; i < S.rows; ++i) S.at<double>(i,i) += 1e-9;

        cv::Mat Sinv;
        bool ok = cv::invert(S, Sinv, cv::DECOMP_SVD);
        if (!ok) {
            for (int i = 0; i < S.rows; ++i) S.at<double>(i,i) += 1e-6;
            cv::invert(S, Sinv, cv::DECOMP_SVD);
        }
        cv::Mat K = P_pre * H64.t() * Sinv;

        // state update
        cv::Mat y = z64 - H64 * pred;
        state64 = pred + K * y;

        // Joseph form P update
        cv::Mat I = cv::Mat::eye(stateDim_, stateDim_, CV_64F);
        cv::Mat Pnew = (I - K * H64) * P_pre * (I - K * H64).t() + K * R64 * K.t();

        // sym + clamp
        cv::Mat Pclamped = symClamp(Pnew);

        // commit
        Pclamped.convertTo(P_, state_.type());
        state64.convertTo(state_, state_.type());

        return state_.clone();
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