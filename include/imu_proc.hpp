/*
 * Copyright (C) 2023 SemiDrive Technology. All rights reserved.
 *
 *  Author: Zhongkang Lin <zhongkang.lin@semidrive.com>
 *
 * This program is released under the terms of the GPL v2 license ("License").
 * You must comply with the terms of the License in order to use this program.
 * You can obtain a copy of the License at:
 *
 *      https://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is provided "as is" without any warranty or conditions,
 * express or implied, including but not limited to the warranties or conditions
 * of merchantability, fitness for a particular purpose, or non-infringement.
 * You bear the risk of using or distributing this program.
 *
 * SemiDrive Technology and other contributors shall not be liable for any direct, indirect,
 * special, incidental, or consequential damages arising out of the use or inability
 * to use this program, even if advised of the possibility of such damages.
 */
#ifndef SEMIDRIVE_GOINS_IMU_PROC_HPP
#define SEMIDRIVE_GOINS_IMU_PROC_HPP
#include <glog/logging.h>
#include <cassert>
#include <cmath>

#include <fstream>
#include "data_frame.hpp"
#include "use-ikfom.hpp"
#include "utils.hpp"

namespace unidrive {

constexpr int MAX_INI_COUNT = 20;

class IMUProc {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUProc();
    ~IMUProc();

    inline void Reset(bool re_init = false);
    inline void ResetPQ(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
    inline void SetExtrinsic_ODOM(const unidrive::V3D &transl, const unidrive::M3D &rot);
    inline void SetExtrinsic_GNSS(const unidrive::V3D &transl, const unidrive::M3D &rot);
    inline void SetGyrCov(const unidrive::V3D &scaler);
    inline void SetAccCov(const unidrive::V3D &scaler);
    inline void SetGyrBiasCov(const unidrive::V3D &b_g);
    inline void SetAccBiasCov(const unidrive::V3D &b_a);
    void Process(const unidrive::FusionDataFrame &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
    inline std::vector<unidrive::ImuPoseFrame> getPose() { return IMUpose_; }

    std::ofstream fout_imu_;
    Eigen::Matrix<double, 12, 12> Q_;
    unidrive::V3D cov_acc_;
    unidrive::V3D cov_gyr_;
    unidrive::V3D cov_acc_scale_;
    unidrive::V3D cov_gyr_scale_;
    unidrive::V3D cov_bias_gyr_;
    unidrive::V3D cov_bias_acc_;

   private:
    void IMUInit(const unidrive::FusionDataFrame &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    // void UndistortPcl(const unidrive::ImuMeasureStack &meas, esekfom::esekf<state_ikfom, 12, input_ikfom>
    // &kf_state);

    std::shared_ptr<ImuData> last_imu_;
    std::deque<std::shared_ptr<ImuData>> v_imu_;
    std::vector<unidrive::ImuPoseFrame> IMUpose_;
    std::vector<unidrive::M3D> v_rot_pcl_;
    unidrive::M3D Car_R_wrt_IMU_;
    unidrive::V3D Car_T_wrt_IMU_;

    unidrive::M3D GNSS_R_wrt_IMU_;  // GNSS init pose w.r.t IMU refrence cordinate
    unidrive::V3D GNSS_T_wrt_IMU_;

    unidrive::V3D mean_acc_;
    unidrive::V3D mean_gyr_;
    unidrive::V3D angvel_last_;
    unidrive::V3D acc_s_last_;
    double last_sensor_end_time_ = -1.0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

inline IMUProc::IMUProc() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_ = process_noise_cov();
    cov_acc_ = unidrive::V3D(0.1, 0.1, 0.1);
    cov_gyr_ = unidrive::V3D(0.1, 0.1, 0.1);
    cov_bias_gyr_ = unidrive::V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = unidrive::V3D(0.0001, 0.0001, 0.0001);
    mean_acc_ = unidrive::V3D(0, 0, -1.0);
    mean_gyr_ = unidrive::V3D(0, 0, 0);
    angvel_last_ = unidrive::Zero3d;
    Car_T_wrt_IMU_ = unidrive::Zero3d;
    Car_R_wrt_IMU_ = unidrive::Eye3d;
    last_imu_.reset(new ImuData);
}

inline IMUProc::~IMUProc() {}

void IMUProc::SetExtrinsic_ODOM(const unidrive::V3D &transl, const unidrive::M3D &rot) {
    Car_T_wrt_IMU_ = transl;
    Car_R_wrt_IMU_ = rot;
}

void IMUProc::SetExtrinsic_GNSS(const unidrive::V3D &transl, const unidrive::M3D &rot) {
    GNSS_T_wrt_IMU_ = transl;
    GNSS_R_wrt_IMU_ = rot;
}

void IMUProc::Reset(bool re_init) {
    if (re_init) {
        mean_acc_ = unidrive::V3D(0, 0, -1.0);
        mean_gyr_ = unidrive::V3D(0, 0, 0);
        imu_need_init_ = true;
    }
    angvel_last_ = unidrive::Zero3d;
    init_iter_num_ = 1;
    v_imu_.clear();
    IMUpose_.clear();
    last_imu_.reset(new ImuData);
    last_sensor_end_time_ = -1.0;
}

void IMUProc::SetGyrCov(const unidrive::V3D &scaler) { cov_gyr_scale_ = scaler; }

void IMUProc::SetAccCov(const unidrive::V3D &scaler) { cov_acc_scale_ = scaler; }

void IMUProc::SetGyrBiasCov(const unidrive::V3D &b_g) { cov_bias_gyr_ = b_g; }

void IMUProc::SetAccBiasCov(const unidrive::V3D &b_a) { cov_bias_acc_ = b_a; }

inline void IMUProc::IMUInit(const unidrive::FusionDataFrame &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                             int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_
     ** from faster-lio
     **/
    unidrive::V3D cur_acc, cur_gyr;
    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu_set.front()->acc;
        const auto &gyr_acc = meas.imu_set.front()->gyr;
        mean_acc_ << imu_acc.x(), imu_acc.y(), imu_acc.z();
        mean_gyr_ << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();
    }
    for (const auto &imu : meas.imu_set) {
        const auto &imu_acc = imu->acc;
        const auto &gyr_acc = imu->gyr;
        cur_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
        cur_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);
        N++;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * unidrive::G_m_s2);

    init_state.bg = mean_gyr_;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P *= 0.0001;
    // init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;       // cov w.r.t offset_R
    // init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;   // cov w.r.t offset_t
    // init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;  // cov w.r.t bg
    // init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;   // cov w.r.t ba
    // init_P(21, 21) = init_P(22, 22) = 0.00001;                  // cov w.r.t grav

    kf_state.change_P(init_P);
    last_imu_ = meas.imu_set.back();
}

inline void IMUProc::ResetPQ(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state) {
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P *= 0.0001;
    // init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;       // cov w.r.t offset_R
    // init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;   // cov w.r.t offset_t
    // init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;  // cov w.r.t bg
    // init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;   // cov w.r.t ba
    // init_P(21, 21) = init_P(22, 22) = 0.00001;                  // cov w.r.t grav

    kf_state.change_P(init_P);

    Q_ = process_noise_cov();
    Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
}

inline void IMUProc::Process(const unidrive::FusionDataFrame &meas,
                             esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state) {
    if (meas.imu_set.empty()) return;
    // assert()
    if (imu_need_init_) {
        IMUInit(meas, kf_state, init_iter_num_);
        last_imu_ = meas.imu_set.back();
        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num_ > MAX_INI_COUNT) {
            cov_acc_ *= pow(unidrive::G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;
            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            LOG(INFO) << "IMU Init Done";
        }
        return;
    }
    /*** add the imu_ of the last frame-tail to the of current frame-head ***/
    auto current_IMU_meas = meas.imu_set;
    current_IMU_meas.push_front(last_imu_);
    const double &imu_beg_time = current_IMU_meas.front()->measure_time;
    const double &imu_end_time = current_IMU_meas.back()->measure_time;
    const double &sensor_measur_time = meas.sensor_measure_time;
    if (last_sensor_end_time_ < 0) last_sensor_end_time_ = sensor_measur_time;  // first input data

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose_.clear();
    IMUpose_.push_back(ImuPoseFrame(last_sensor_end_time_, 0.0, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                    imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu_ point ***/
    unidrive::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    unidrive::M3D R_imu;

    double dt = 0;
    input_ikfom in;
    for (auto it_imu = current_IMU_meas.begin(); it_imu < (current_IMU_meas.end() - 1); it_imu++) {
        auto &&head = *it_imu;  // T && means rvalue references
        auto &&tail = *(it_imu + 1);

        if (tail->measure_time < last_sensor_end_time_) {
            continue;
        }

        angvel_avr << 0.5 * (head->gyr.x() + tail->gyr.x()), 0.5 * (head->gyr.y() + tail->gyr.y()),
            0.5 * (head->gyr.z() + tail->gyr.z());

        acc_avr << 0.5 * (head->acc.x() + tail->acc.x()), 0.5 * (head->acc.y() + tail->acc.y()),
            0.5 * (head->acc.z() + tail->acc.z());

        acc_avr = acc_avr * unidrive::G_m_s2 / mean_acc_.norm();
        if (head->measure_time < last_sensor_end_time_) {
            dt = tail->measure_time - last_sensor_end_time_;
        } else {
            dt = tail->measure_time - head->measure_time;
        }
        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
        kf_state.predict(dt, Q_, in);

        /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();
        angvel_avr = angvel_avr - imu_state.bg;
        acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav[i];
        }
        double &&offs_t = tail->measure_time - last_sensor_end_time_;
        IMUpose_.emplace_back(ImuPoseFrame(tail->measure_time, offs_t, acc_s_last_, angvel_last_, imu_state.vel,
                                           imu_state.pos, imu_state.rot.toRotationMatrix()));
    }
    // 外推到dataframe接收时刻
    double note = sensor_measur_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (sensor_measur_time - imu_end_time);
    kf_state.predict(dt, Q_, in);

    // imu_state = kf_state.get_x();
    last_imu_ = meas.imu_set.back();
    last_sensor_end_time_ = sensor_measur_time;
}

}  // namespace unidrive

#endif