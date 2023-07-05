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
#ifndef unidrive_goins_HPP
#define unidrive_goins_HPP

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <deque>
// #include <execution>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "wheel_odom_proc.hpp"

#include "data_frame.hpp"
#include "imu_proc.hpp"
#include "options.h"
#include "use-ikfom.hpp"
#include "utils.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include <ios>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"

#define IMU_TIME_INTERV 0.01

namespace unidrive {
class unidrive_goins {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    unidrive_goins(bool print_debug_info = false);
    ~unidrive_goins() { LOG(INFO) << "unidrive_goins deconstructed!"; }
    bool initFromFile(const std::string &config_yaml);
    void Run();
    // sync sensor with imu
    bool SyncPackages();
    /// interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    void Savetrajectory(const std::string &traj_file);
    void Finish();
    void inputIMUData(ImuData &imu_data);
    void inputOdomData(WheelData &odom_data);
    void inutGNSSData(GNSSData &gnss_data);
    void handleOdomIntegrate(double odom_data_measure_time);
    void handleGNSSMeasure(double gnss_data_measure_time);
    inline bool isImuNeedAppend() { return imu_data_need_append; }
    inline void setZUPT(bool zupt){ZUPT = zupt;}
    inline void setTrajBufferLength(size_t len){ traj_buffer_length = len;}
    inline void setDebugInfo(bool debug_info){print_debug_info_ = debug_info;}
    void resetGOINS(bool need_re_init = false);
    std::deque<Path> getTrajc(){return trajtory;}
    std::deque<Path> getTrajcFULL(){return trajtory_all;}
    Path getPose(){return trajtory.back();}
    void savePose();

   private:
    template <typename T>
    void SetPosestamp(T &out);
    bool LoadParamsFromYAML(const std::string &yaml);
    void PrintState(const state_ikfom &s);

   private:
    // modules
    std::shared_ptr<IMUProc> p_imu_ = nullptr;
    std::shared_ptr<WheelOdomProc> p_odom_ = nullptr;
    GeographicLib::LocalCartesian geo_converter;
    // params
    std::vector<double> extrinT_odom{3, 0.0};  // car w.r.t imu
    std::vector<double> extrinR_odom{9, 0.0};
    std::vector<double> extrinT_gnss{3, 0.0};  // gnss w.r.t imu
    std::vector<double> extrinR_gnss{9, 0.0};
    OdomMeasurment odom_measure;
    Eigen::MatrixXd odom_R = Eigen::MatrixXd::Identity(9, 9);
    unidrive::V3D gnss_local_cord;
    Eigen::MatrixXd gnss_R = Eigen::MatrixXd::Identity(3, 3);
    // buffrs and mutex
    size_t traj_buffer_length = 100;
    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<ImuData> imu_buffer_;
    std::deque<WheelData> odom_buffer_;
    std::deque<GNSSData> gnss_buffer_;
    /// options
    bool time_sync_en_ = false;
    double timediff_odom_wrt_imu_ = 0.0;
    double timediff_gnss_wrt_imu_ = 0.0;
    double last_timestamp_gnss_ = 0.0;
    double last_timestamp_odom_ = 0.0;
    double odom_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double last_pose_update_time = 0.0;
    double first_odom_time_ = 0.0;
    double last_odom_integrate_time = 0.0;
    bool odom_pushed_ = false;
    /// statistics and flags ///
    bool syncdata_sucess_odom = false;
    bool imu_data_need_append = false;
    bool gnss_need_reset = true;
    bool is_gnss_yaw_estimation_ready = false;
    bool is_yaw_ready_to_use = false;
    bool save_imu_pose = false;
    int publish_count_ = 0;
    bool flg_EKF_inited_ = false;
    bool timediff_set_flg_ = false;
    double odom_integrate_gap = 0.5;  // unit: second
    int effect_feat_num_ = 0, frame_num_ = 0;
    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    unidrive::FusionDataFrame measures_;                  // sync IMU and other sensor scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf
    state_ikfom state_point_;                          // ekf current state
    vect3 pos_;                                        // position after eskf update
    unidrive::V3D euler_cur_ = unidrive::V3D::Zero();        // rotation in euler angles
    bool extrinsic_est_en_ = true;
    FusionSensorType now_fusision_type = FusionSensorType::no_measurement;
    unidrive::V3D last_gnss_pose = unidrive::V3D::Zero();
    std::deque<Path> trajtory_all;
    std::deque<Path> trajtory;
    /////////////////////////  debug show / save /////////////////////////////////////////////////////////
    bool run_in_offline_ = false;
    bool path_pub_en_ = true;
    bool runtime_pos_log_ = true;
    bool path_save_en_ = false;
    bool ZUPT = false;
    bool print_debug_info_ = false;
    double calib_scale_factor = 1.0;
    std::string dataset_;
};

}  // namespace unidrive

#endif