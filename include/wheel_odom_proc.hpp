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
#ifndef SEMIDRIVE_GOINS_WHEEL_ODOM_HPP
#define SEMIDRIVE_GOINS_WHEEL_ODOM_HPP

#include <glog/logging.h>
#include <math.h>
#include <cmath>
#include <map>
#include <memory>
#include "data_frame.hpp"
#include "use-ikfom.hpp"
#include "utils.hpp"

namespace unidrive {
constexpr double unidrive_PI = 3.14159265359;
class WheelOdomProc {
   public:
    WheelOdomProc(){};
    ~WheelOdomProc() { last_wheel_data = nullptr; };

    enum InputType { wheelspeed_vlvr = 0, wheelspeed_wlwr = 1, wheelspeed_vxwx = 2 };
    std::map<int, std::string> input_map = {{0, "wheelspeed_vlvr"}, {1, "wheelspeed_wlwr"}, {2, "wheelspeed_vxwx"}};

    inline void setIntrincJ(double wheel_radius_l, double wheel_radius_r, double baseline) {
        J << 0.5 * wheel_radius_l, 0.5 * wheel_radius_r, -wheel_radius_l / baseline, wheel_radius_r / baseline;
        L = baseline;
        J_Assigned = true;
    }

    inline void setInputType(InputType tp) {
        type = tp;
        LOG(INFO) << "Setting inut type to: " << type << " " << input_map[type];
    }

    void dead_reckoning(const unidrive::FusionDataFrame &dataframe, OdomMeasurment &out_measure,
                        double calib_v_factor = 1.0);
    inline void odom_reset() {
        last_yaw = 0.0;
        last_pose = unidrive::V2D::Zero();
    }
    /*
     * @description update last pose from outside
     * @param pose
     */
    inline void updateState(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,bool update_yaw = false) {
        auto state = kf_state.get_x();
        const unidrive::M3D Rci = state.offset_R_C_I.toRotationMatrix().transpose().cast<double>();
        const unidrive::V3D tci = -Rci * state.offset_T_C_I.cast<double>();
        auto now_pose = (Rci * state.pos + tci);
        last_pose = now_pose.block<2, 1>(0, 0);
        if(update_yaw)
            last_yaw = SO3::log(state.rot).z();
    }

    inline void resetOdom() {
        last_yaw = 0;
        last_integrate_time = -1.0;
        last_pose = unidrive::V2D::Zero();
        last_wheel_data = nullptr;
    }

   private:
    unidrive::M2D J;
    bool J_Assigned = false;
    InputType type = InputType::wheelspeed_vlvr;
    double L;
    unidrive::V2D last_pose = unidrive::V2D::Zero();
    std::shared_ptr<WheelData> last_wheel_data = nullptr;
    double last_yaw = 0;
    double last_integrate_time = -1.0;
};
}  // namespace unidrive

inline void unidrive::WheelOdomProc::dead_reckoning(const unidrive::FusionDataFrame &dataframe, OdomMeasurment &out_measure,
                                                 double calib_v_factor) {
    auto odom_dataset = dataframe.wheel_data_stack;
    if (last_wheel_data != nullptr) odom_dataset.push_front(last_wheel_data);
    double integrate_time = dataframe.sensor_measure_time;
    const double &odom_beg_time = dataframe.wheel_data_stack.front()->measure_time;
    const double &odom_end_time = dataframe.wheel_data_stack.back()->measure_time;
    double dt = 0;
    double car_v_avg;
    double car_w_avg;
    if (last_integrate_time < 0) last_integrate_time = odom_beg_time;
    for (auto it_wheel = odom_dataset.begin(); it_wheel < (odom_dataset.end() - 1); it_wheel++) {
        auto &&head = *(it_wheel);
        auto &&tail = *(it_wheel + 1);
        if (tail->measure_time < last_integrate_time) continue;
        double car_v_head, car_v_tail, car_w_head, car_w_tail;
        if (type == InputType::wheelspeed_vlvr) {
            unidrive::V2D car_speed_head = head->wheel_speed;
            car_v_head = (car_speed_head(0) + car_speed_head(1)) / 2;
            car_w_head = (car_speed_head(1) - car_speed_head(0)) / L;

            unidrive::V2D car_speed_tail = tail->wheel_speed;
            car_v_tail = (car_speed_tail(0) + car_speed_tail(1)) / 2;
            car_w_tail = (car_speed_tail(1) - car_speed_tail(0)) / L;
        } else if (type == InputType::wheelspeed_vxwx) {
            unidrive::V2D car_speed_head = head->wheel_speed;
            car_v_head = car_speed_head(0);
            car_w_head = car_speed_head(1);

            unidrive::V2D car_speed_tail = tail->wheel_speed;
            car_v_tail = car_speed_tail(0);
            car_w_tail = car_speed_tail(1);
        } else {
            unidrive::V2D car_speed_head = J * head->wheel_speed;
            car_v_head = car_speed_head(0);
            car_w_head = car_speed_head(1);

            unidrive::V2D car_speed_tail = J * tail->wheel_speed;
            car_v_tail = car_speed_tail(0);
            car_w_tail = car_speed_tail(1);
        }
        car_w_avg = (car_w_head + car_w_tail) * 0.5;
        car_v_avg = (car_v_head + car_v_tail) * 0.5 * calib_v_factor;
        if (head->measure_time < last_integrate_time) dt = tail->measure_time - last_integrate_time;
        else
            dt = tail->measure_time - head->measure_time;
        unidrive::V3D last_state;
        last_state << last_pose.x(), last_pose.y(), last_yaw;

        double dyaw = car_w_avg * dt;                 // rad
        double dx = -car_v_avg * dt * sin(last_yaw);  // rad car cordinate is RFU
        double dy = car_v_avg * dt * cos(last_yaw);

        unidrive::V3D dPose;
        dPose << dx, dy, dyaw;
        unidrive::V3D current_state;
        current_state = last_state + dPose;
        last_pose << current_state.block<2, 1>(0, 0);
        last_yaw = current_state(2);
        // if (last_yaw > 2 * unidrive_PI) {
        //     last_yaw = last_yaw - 2 * unidrive_PI;
        // } else if (last_yaw < -2 * unidrive_PI) {
        //     last_yaw = last_yaw + 2 * unidrive_PI;
        // }
    }
    /*** calculated the pos and attitude prediction at the frame-end ***/
    // 轨迹外推
    double note = integrate_time > odom_end_time ? 1.0 : -1.0;
    dt = note * (integrate_time - odom_end_time);
    double dyaw = car_w_avg * dt;                 // rad
    double dx = -car_v_avg * dt * sin(last_yaw);  // rad car cordinate is RFU
    double dy = car_v_avg * dt * cos(last_yaw);

    unidrive::V3D dPose;
    dPose << dx, dy, dyaw;
    unidrive::V3D last_state;
    last_state << last_pose.x(), last_pose.y(), last_yaw;
    last_state = last_state + dPose;
    last_pose << last_state.block<2, 1>(0, 0);
    last_yaw = last_state(2);
    // if (last_yaw > 2 * unidrive_PI) {
    //     last_yaw = last_yaw - 2 * unidrive_PI;
    // } else if (last_yaw < -2 * unidrive_PI) {
    //     last_yaw = last_yaw + 2 * unidrive_PI;
    // }

    last_wheel_data = dataframe.wheel_data_stack.back();
    last_integrate_time = integrate_time;
    out_measure.current_pose = last_pose;
    out_measure.current_yaw = last_yaw;
    out_measure.current_vel << -car_v_avg * sin(last_yaw), car_v_avg * cos(last_yaw);
}

#endif