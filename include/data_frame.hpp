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
#ifndef SEMIDRIVE_GOINS_DATA_FRAME_HPP
#define SEMIDRIVE_GOINS_DATA_FRAME_HPP
#include <deque>
#include <memory>
#include "utils.hpp"

namespace unidrive {

struct ImuData {
    ImuData() {}
    unidrive::V3D acc;
    unidrive::V3D gyr;
    double measure_time;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::deque<std::shared_ptr<ImuData>> ImuMeasureStack;

// the preintegrated measurment sensor states at the time of IMU measurements in a frame
struct ImuPoseFrame {
    ImuPoseFrame() {}

    ImuPoseFrame(double t,double t_off, unidrive::V3D a, unidrive::V3D g, unidrive::V3D v, unidrive::V3D p, unidrive::M3D R) {
        timestamp = t;
        offset_time = t_off;
        acc = a;
        gyr = g;
        vel = v;
        pose = p;
        rot = R;
    }

    inline void changeData(double t,double t_off, unidrive::V3D a, unidrive::V3D g, unidrive::V3D v, unidrive::V3D p,
                           unidrive::M3D R) {
        timestamp = t;
        offset_time = t_off;
        acc = a;
        gyr = g;
        vel = v;
        pose = p;
        rot = R;
    }
    double timestamp;
    double offset_time;   // the offset time of IMU measurement w.r.t the first measurment sensor point
    unidrive::V3D acc;   // the preintegrated total acceleration (global frame) at the measurment sensor origin
    unidrive::V3D gyr;   // the unbiased angular velocity (body frame) at the measurment sensor origin
    unidrive::V3D vel;   // the preintegrated velocity (global frame) at the measurment sensor origin
    unidrive::V3D pose;  // the preintegrated position (global frame) at the measurment sensor origin
    unidrive::M3D rot;   // the preintegrated rotation (global frame) at the measurment sensor origin
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum WheelState
{
    FORWARD = 0,
    BACKWARD = 1,
    STANDSTILL = 2,
    INVALID = 3
};

struct WheelData
{
    unidrive::V2D wheel_speed;
    double measure_time;
    WheelState state_right = WheelState::FORWARD;
    WheelState state_left = WheelState::FORWARD;
};

struct OdomMeasurment
{
    unidrive::V2D current_vel;
    unidrive::V2D current_pose;
    double current_yaw;
};

struct Path
{
    double timestamp;
    unidrive::V3D pose;
    unidrive::M3D Rotation;
};

struct GNSSData
{
    double measure_time;
    double lat;
    double lon;
    double h;
};


enum FusionSensorType
{
    no_measurement = 0,
    wheel_odom = 1,
    gnss = 2
};

struct FusionDataFrame {
    FusionDataFrame() {}

    ImuMeasureStack imu_set;
    std::deque<std::shared_ptr<WheelData>> wheel_data_stack;
    std::shared_ptr<GNSSData> gnss_measure_ptr;
    double sensor_measure_time = 0;
    FusionSensorType measure_sensor_type = FusionSensorType::no_measurement;

};

}  // namespace unidrive

#endif