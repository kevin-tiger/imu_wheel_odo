#include "unidrive_goins.h"
#include <glog/logging.h>

namespace unidrive {
constexpr double time_minimal_gap = IMU_TIME_INTERV;
unidrive_goins::unidrive_goins(bool print_debug_info) : print_debug_info_(print_debug_info) {
    p_imu_.reset(new IMUProc());
    p_odom_.reset(new WheelOdomProc());
}

bool unidrive_goins::initFromFile(const std::string &config_yaml) {
    LOG(INFO) << "init from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }
    // esekf init
    std::vector<double> epsi(30, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());
    return true;
}

bool unidrive_goins::LoadParamsFromYAML(const std::string &yaml_file) {
    auto yaml = YAML::LoadFile(yaml_file);
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    unidrive::V3D Car_T_wrt_IMU;
    unidrive::M3D Car_R_wrt_IMU;  // IMU安装杆臂

    unidrive::V3D GNSS_T_wrt_IMU;  // GNSS天线杆臂
    unidrive::M3D GNSS_R_wrt_IMU;

    try {
        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        save_imu_pose = yaml["save_imu_pose"].as<bool>();
        gyr_cov = yaml["IMU_Params"]["gyr_cov"].as<double>();
        acc_cov = yaml["IMU_Params"]["acc_cov"].as<double>();
        b_acc_cov = yaml["IMU_Params"]["bias_acc_cov"].as<double>();
        b_gyr_cov = yaml["IMU_Params"]["bias_gyr_cov"].as<double>();
        extrinT_odom = yaml["offset"]["offset_T_car_wrt_imu"].as<std::vector<double>>();
        extrinR_odom = yaml["offset"]["offset_R_car_wrt_imu"].as<std::vector<double>>();
        extrinT_gnss = yaml["offset"]["offset_T_gnss_wrt_imu"].as<std::vector<double>>();
        extrinR_gnss = yaml["offset"]["offset_R_gnss_wrt_imu"].as<std::vector<double>>();
        int type_int = yaml["odom"]["input_type"].as<int>();
        WheelOdomProc::InputType odom_type = static_cast<WheelOdomProc::InputType>(type_int);
        odom_integrate_gap = yaml["odom"]["odom_integrate_gap"].as<double>();
        calib_scale_factor = yaml["calib_scale_factor"].as<double>();
        p_odom_->setInputType(odom_type);

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }
    Car_T_wrt_IMU = unidrive::VecFromArray<double>(extrinT_odom);
    Car_R_wrt_IMU = unidrive::MatFromArray<double>(extrinR_odom);

    GNSS_T_wrt_IMU = unidrive::VecFromArray<double>(extrinT_gnss);
    GNSS_R_wrt_IMU = unidrive::MatFromArray<double>(extrinR_gnss);

    p_imu_->SetExtrinsic_ODOM(Car_T_wrt_IMU, Car_R_wrt_IMU);
    p_imu_->SetExtrinsic_GNSS(GNSS_T_wrt_IMU, GNSS_R_wrt_IMU);
    p_imu_->SetGyrCov(unidrive::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetGyrBiasCov(unidrive::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccCov(unidrive::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetAccBiasCov(unidrive::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    run_in_offline_ = true;
    return true;
}

void unidrive_goins::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }
    // ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;

    for (const auto &p : trajtory_all) {
        Eigen::Quaterniond quatern(p.Rotation);
        ofs << std::fixed << std::setprecision(6) << p.timestamp << " " << std::setprecision(15) << p.pose.x() << " "
            << p.pose.y() << " " << p.pose.z() << " " << quatern.x() << " " << quatern.y() << " " << quatern.z() << " "
            << quatern.w() << std::endl;
    }

    ofs.close();
}

void unidrive_goins::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    ekfom_data.h_x = Eigen::MatrixXd::Zero(9, 29);
    ekfom_data.h_v = Eigen::MatrixXd::Zero(9, 9);
    ekfom_data.R = odom_R;
    ekfom_data.z.resize(9);
    ekfom_data.h.resize(9);
    if (measures_.measure_sensor_type == FusionSensorType::wheel_odom) {
        ekfom_data.R(2, 2) *= 10;
        ekfom_data.R(8, 8) *= 10;
        unidrive::V2D current_pose = odom_measure.current_pose;
        double current_yaw = odom_measure.current_yaw;
        unidrive::V2D current_vel = odom_measure.current_vel;
        if (print_debug_info_) {
            LOG(INFO) << "Into ObsModel odom";
            LOG(INFO) << "[Odom obsmodel] current odom pose: " << odom_measure.current_pose.transpose()
                      << " system pose: " << s.pos.transpose();
            LOG(INFO) << "[Odom obsmodel] current odom vel: " << odom_measure.current_vel.transpose()
                      << " system vel: " << s.vel.transpose();
            LOG(INFO) << "[Odom obsmodel] current odom yaw: " << odom_measure.current_yaw
                      << " system yaw: " << SO3::log(s.rot).z();
        }

        unidrive::M3D Rci = s.offset_R_C_I.toRotationMatrix().transpose().cast<double>();
        unidrive::V3D tci = -Rci * s.offset_T_C_I.cast<double>();
        ekfom_data.z.block<2, 1>(0, 0) = current_pose;
        ekfom_data.z(2) = 0;
        ekfom_data.h.block<3, 1>(0, 0) = (Rci * s.pos + tci);
        ekfom_data.h_x.block<3, 3>(0, 0) = Rci;  //(h(x_true_p)-h(x_state_p)) w.r.t error_p
        // ekfom_data.h_x.block<3, 3>(0, 18) = Eigen::MatrixXd::Zero(3, 3);  //(h(x_true_p)-h(x_state_p)) w.r.t
        // error_rot
        // todo add offset factor to adjust the extrinsic param RIC and tic

        Eigen::AngleAxisd yawAngle(current_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond euler_rot(yawAngle);
        SO3 new_rot(euler_rot);
        vect3 res_rot = SO3::log(new_rot);
        // LOG(INFO) << "new_rot:\n" << new_rot;
        // LOG(INFO) << "res_rot:\n" << res_rot;
        // LOG(INFO) << "sys_rot:\n" << SO3::log(s.rot);
        ekfom_data.z.block<3, 1>(3, 0) = res_rot;
        ekfom_data.h.block<3, 1>(3, 0) = SO3::log(s.rot);
        ekfom_data.h_x.block<3, 3>(3, s.rot.IDX) =
            Eigen::MatrixXd::Identity(3, 3);  //(h(x_true_rot)-h(x_state_rot)) w.r.t error_rot
        ekfom_data.h_x.block<2, 2>(3, s.rot.IDX) = 20 * ekfom_data.h_x.block<2, 2>(3, 3);

        ekfom_data.z.block<2, 1>(6, 0) = current_vel;
        ekfom_data.z(8) = 0;  // NHC Constraint
        ekfom_data.h.block<3, 1>(6, 0) = Rci * s.vel;
        ekfom_data.h_x.block<3, 3>(6, s.vel.IDX) = Rci;  //(h(x_true_v)-h(x_state_v)) w.r.t error_v
        // ekfom_data.h_x.block<3, 3>(6, 18) = Eigen::MatrixXd::Zero(3, 3);  //(h(x_true_v)-h(x_state_v)) w.r.t
        // error_rot

    } else if (measures_.measure_sensor_type == FusionSensorType::gnss) {
        if (is_yaw_ready_to_use) {
            if (print_debug_info_) LOG(INFO) << "Into ObsModel GNSS";
            const unidrive::M3D RGI = s.offset_R_GNSS_I.toRotationMatrix().transpose().cast<double>();
            ekfom_data.z.block<3, 1>(0, 0) = gnss_local_cord;
            ekfom_data.h.block<3, 1>(0, 0) = RGI * s.pos;
            ekfom_data.h_x.block<3, 3>(0, s.pos.IDX) = RGI;  //(h(x_true_p)-h(x_state_p)) w.r.t error_p
            // stay the same as before, only update position
            ekfom_data.z.block<3, 1>(3, 0) = SO3::log(s.rot);
            ekfom_data.h.block<3, 1>(3, 0) = SO3::log(s.rot);
            ekfom_data.h_x.block<3, 3>(3, s.rot.IDX) =
                Eigen::MatrixXd::Identity(3, 3);  //(h(x_true_rot)-h(x_state_rot)) w.r.t error_rot

            unidrive::M3D Rci = s.offset_R_C_I.toRotationMatrix().transpose().cast<double>();
            unidrive::V3D tci = -Rci * s.offset_T_C_I.cast<double>();
            ekfom_data.z.block<3, 1>(6, 0) = Rci * s.vel;
            ekfom_data.z(8) = 0;  // NHC Constraint
            ekfom_data.h.block<3, 1>(6, 0) = Rci * s.vel;
            ekfom_data.h_x.block<3, 3>(6, s.vel.IDX) = Rci;  //(h(x_true_v)-h(x_state_v)) w.r.t error_v
            // ekfom_data.h_x.block<3, 3>(6, 18) = Eigen::MatrixXd::Zero(3, 3);  //(h(x_true_v)-h(x_state_v)) w.r.t
        } else {
            LOG(WARNING) << "gnss yaw estimation is not ready";
            ekfom_data.valid = false;
            return;
        }
    }

    // ekfom_data.h(0) = -(current_pose(0) - s.pos(0));
}

void unidrive_goins::handleOdomIntegrate(double odom_data_measure_time) {
    if (print_debug_info_) LOG(INFO) << "into integrate step";
    measures_.measure_sensor_type = FusionSensorType::wheel_odom;
    measures_.sensor_measure_time = odom_data_measure_time;
    Run();
}

void unidrive_goins::handleGNSSMeasure(double gnss_data_measure_time) {
    if (print_debug_info_) LOG(INFO) << "into gnss step";
    measures_.measure_sensor_type = FusionSensorType::gnss;
    measures_.sensor_measure_time = gnss_data_measure_time;
    SyncPackages();
    double lat, lon, h;
    lat = measures_.gnss_measure_ptr->lat;
    lon = measures_.gnss_measure_ptr->lon;
    h = measures_.gnss_measure_ptr->h;
    if (gnss_need_reset) {
        gnss_need_reset = false;
        is_gnss_yaw_estimation_ready = false;
        geo_converter.Reset(lat, lon, h);
        last_gnss_pose = Eigen::Vector3d::Zero();
        // todo to get init yaw from RFU2ENU and set as offset_GNSS_I
    }
    // todo handle GNSS data
    double px, py, pz;
    geo_converter.Forward(lat, lon, h, px, py, pz);
    gnss_local_cord << px, py, 0;
    // need to run at least 10m to avoid unstable init
    if (gnss_local_cord.norm() > 10 && !is_gnss_yaw_estimation_ready) {
        is_gnss_yaw_estimation_ready = true;
        LOG(INFO) << "gnss yaw estimation is ready";
    }
    LOG(INFO) << "gnss_local_cord: " << gnss_local_cord.transpose();
    if (is_gnss_yaw_estimation_ready || is_yaw_ready_to_use) {
        if (flg_EKF_inited_) {
            // double yaw_measure =
            auto current_state = kf_.get_x();
            auto current_pose = kf_.get_x().pos;
            Eigen::Quaterniond QIG = Eigen::Quaterniond::FromTwoVectors(gnss_local_cord.normalized(),
                                                                        current_pose.cast<double>().normalized());
            QIG.normalize();
            SO3 SO3_QIG(QIG);
            const unidrive::M3D RIG = SO3_QIG.toRotationMatrix().cast<double>();
            unidrive::V3D gnss_pos = RIG * gnss_local_cord;
            LOG(INFO) << "current_pose: " << current_pose.transpose() << " gnss_pos: " << gnss_pos.transpose();
            auto diff_error = gnss_pos - current_pose;
            LOG(INFO) << "gnss diff error: " << diff_error.transpose();
            if (diff_error.squaredNorm() < 5e-1) {
                is_yaw_ready_to_use = true;
                current_state.offset_R_GNSS_I = SO3_QIG;
                kf_.change_x(current_state);
                LOG(INFO) << "gnss yaw is ready to use";
            }
            double two_gnss_data_dist =
                (gnss_pos - last_gnss_pose).norm();  // need engouth distance to get robust estimate
            if (is_yaw_ready_to_use && two_gnss_data_dist > 10) {
                double ratio = 1 / (1 + exp(-diff_error.squaredNorm()));
                // double gap_time = measures_.sensor_measure_time - last_pose_update_time;
                // auto midian_vel = (gnss_pos - current_pose) / gap_time;
                // auto new_vel = midian_vel + current_state.vel;
                // double yaw_measure = atan2(-new_vel(0), new_vel(1));
                // Eigen::AngleAxisd yawAngle(yaw_measure, Eigen::Vector3d::UnitZ());
                // Eigen::Quaterniond euler_rot(yawAngle);
                // SO3 new_rot(euler_rot);
                current_state.pos = gnss_pos * (1 - ratio) + current_pose * ratio;
                // current_state.vel = new_vel;
                // current_state.rot = new_rot;
                kf_.change_x(current_state);
                // p_odom_->updateState(kf_,true);
                p_odom_->updateState(kf_);
                last_gnss_pose = gnss_pos;
            }
        }
    }
}

bool unidrive_goins::SyncPackages() {
    if ((gnss_buffer_.empty() && odom_buffer_.empty() && measures_.wheel_data_stack.empty()) ||
        (imu_buffer_.empty() && gnss_buffer_.empty())) {
        LOG(WARNING) << "buffer empty! " << gnss_buffer_.empty() << imu_buffer_.empty() << odom_buffer_.empty();
        return false;
    }
    double measure_time = 0.0;
    measure_time = measures_.sensor_measure_time;
    if (measures_.measure_sensor_type == FusionSensorType::wheel_odom) {
        double wheel_data_time = odom_buffer_.front().measure_time;
        if (syncdata_sucess_odom) {
            syncdata_sucess_odom = false;
            measures_.wheel_data_stack.clear();
        }

        // while ((!odom_buffer_.empty()) && (wheel_data_time - measure_time <= time_minimal_gap)) {
        //     wheel_data_time = odom_buffer_.front().measure_time;
        //     if (wheel_data_time - measure_time > time_minimal_gap) break;
        //     measures_.wheel_data_stack.push_back(std::make_shared<WheelData>(odom_buffer_.front()));
        //     odom_buffer_.pop_front();
        // }
        if ((!odom_buffer_.empty())) {
            measures_.wheel_data_stack.push_back(std::make_shared<WheelData>(odom_buffer_.front()));
            odom_buffer_.pop_front();
        }
        if(print_debug_info_)
            LOG(INFO) << "wheel_data_stack size: " << measures_.wheel_data_stack.size();
    } else if (measures_.measure_sensor_type == FusionSensorType::gnss) {
        auto data = gnss_buffer_.front();
        measures_.gnss_measure_ptr = std::make_shared<GNSSData>(data);
        gnss_buffer_.pop_front();
        // gnss not need to sync, when the data is ready, it will be used
        return true;
    } else {
        return false;
    }

    /*
    这里的last_timestamp_imu_指的是上一次读取的 IMU 时间戳，如果 last_timestamp_imu_ 不小于
    measure_time，说明在当前GNSS或Odom数据包的结束时间之前的 IMU 数据已经被全部记录到 measures_ 中，可以进行处理。如果
    last_timestamp_imu_ 小于 measure_time，说明在当前GNSS或Odom数据包的结束时间之前的 IMU 数据还没有被完全记录到
    measures_ 中，此时应该继续等待，直到 last_timestamp_imu_ 不小于
    measure_time。这样，我们才能保证在当前GNSS或Odom数据包的结束时间，全部的惯性运动信息被记录到，防止运动信息出现丢失。
    */
    // if (last_timestamp_imu_ < measure_time) {
    //     LOG(WARNING) << "imu data not ready!  " << last_timestamp_imu_ << " < " << measure_time;
    //     imu_data_need_append = true;
    //     return false;
    // }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front().measure_time;
    measures_.imu_set.clear();
    // while ((!imu_buffer_.empty()) && (imu_time - measure_time <= time_minimal_gap)) {
    //     imu_time = imu_buffer_.front().measure_time;
    //     if (imu_time > measure_time) break;
    //     measures_.imu_set.push_back(std::make_shared<ImuData>(imu_buffer_.front()));
    //     imu_buffer_.pop_front();
    // }
    if ((!imu_buffer_.empty())) {
        measures_.imu_set.push_back(std::make_shared<ImuData>(imu_buffer_.front()));
        imu_buffer_.pop_front();
    }
    if(print_debug_info_)
    {
        LOG(INFO) << "imu set size: " << measures_.imu_set.size();
        LOG(WARNING) << "SyncPackages ready!  ";
    }
    assert(measures_.wheel_data_stack.size() == measures_.imu_set.size());
    imu_data_need_append = false;
    if (measures_.measure_sensor_type == FusionSensorType::wheel_odom) {
        last_odom_integrate_time = measure_time;
        syncdata_sucess_odom = true;
    }
    return true;
}

void unidrive_goins::resetGOINS(bool need_re_init) {
    p_imu_->Reset(need_re_init);
    if (p_odom_) p_odom_->resetOdom();
    first_odom_time_ = -1.0;
    gnss_need_reset = true;
}

void unidrive_goins::Run() {
    if (!SyncPackages()) return;
    auto last_kf_ = kf_;
    auto last_state = last_kf_.get_x();
    // IMU process, kf prediction
    p_imu_->Process(measures_, kf_);
    if (first_odom_time_ < 0) first_odom_time_ = measures_.sensor_measure_time;
    flg_EKF_inited_ = (measures_.sensor_measure_time - first_odom_time_) >= options::INIT_TIME;
    // iterated Kalman filter update
    //  iterated state estimation
    // update the observation model, will call nn and point-to-plane residual computation
    if (measures_.measure_sensor_type == FusionSensorType::wheel_odom) {
        p_odom_->dead_reckoning(measures_, odom_measure, calib_scale_factor);
    }
    if (flg_EKF_inited_) {
        kf_.update_iterated_dyn_share();
        // Error State Check
        {
            auto current_state = kf_.get_x();
            auto LP = last_state.pos;
            auto CP = current_state.pos;
            if (print_debug_info_) {
                LOG(WARNING) << "Before update:\npos:" << last_state.pos << "\nrot:" << last_state.rot
                             << "\nvel:" << last_state.vel << "\nbg:" << last_state.bg << "\nba:" << last_state.ba;
                LOG(WARNING) << "After update:\npos:" << current_state.pos << "\nrot:" << current_state.rot
                             << "\nvel:" << current_state.vel << "\nbg:" << current_state.bg
                             << "\nba:" << current_state.ba;
            }
            // // MAX_CAR_SPPED km/h -> m/s
            constexpr double MAX_CAR_SPPED = 160 / 3.6;
            double gap_time = measures_.sensor_measure_time - last_pose_update_time;
            double MAX_DIST_TWO_FARME = MAX_CAR_SPPED * gap_time;
            constexpr double MAX_Z_DIFF = 0.5;
            double diff_x = CP.x() - LP.x();
            double diff_y = CP.y() - LP.y();
            double diff_z = abs(CP.z() - LP.z());
            double EKF_DIST = sqrt(diff_x * diff_x + diff_y * diff_y);
            if (EKF_DIST > MAX_DIST_TWO_FARME || diff_z > MAX_Z_DIFF || current_state.ba.norm() > 1e3 ||
                current_state.bg.norm() > 1e3) {
                LOG(WARNING) << "Detect Error State! EKF_DIST:" << EKF_DIST << " ,MAX_DIST_TWO_FARME"
                             << MAX_DIST_TWO_FARME << " .diff_z:" << diff_z << " ,gap_time" << gap_time;
                if (measures_.measure_sensor_type == FusionSensorType::wheel_odom) {
                    LOG(WARNING) << "Using Last Pose to Correct!";
                    LOG(WARNING) << "Odom pose:\n" << odom_measure.current_pose;
                    LOG(WARNING) << "Before Correct:\npos:" << current_state.pos << "\nrot:" << current_state.rot
                                 << "\nvel:" << current_state.vel << "\nbg:" << current_state.bg
                                 << "\nba:" << current_state.ba;
                    // 回滚到上一帧且重置vel ba bg
                    last_state.pos = last_state.pos.squaredNorm() > current_state.pos.squaredNorm() ? current_state.pos
                                                                                                    : last_state.pos;
                    last_state.vel = last_state.vel.squaredNorm() > current_state.vel.squaredNorm() ? current_state.vel
                                                                                                    : last_state.vel;
                    last_state.ba = last_state.ba / 2;
                    last_state.bg = last_state.bg / 2;
                    last_kf_.change_x(last_state);
                    kf_ = last_kf_;
                    p_imu_->ResetPQ(kf_);
                    LOG(WARNING) << "After Correct:\npos:" << kf_.get_x().pos << "\nrot:" << kf_.get_x().rot
                                 << "\nvel:" << kf_.get_x().vel << "\nbg:" << kf_.get_x().bg
                                 << "\nba:" << kf_.get_x().ba;
                }
            } else if (std::isnan(current_state.pos.x()) || std::isnan(current_state.pos.y()) ||
                       std::isnan(current_state.pos.z())) {
                LOG(WARNING) << "Detect NAN State! EKF_DIST: x" << current_state.pos.x() << " y"
                             << current_state.pos.y() << " z" << current_state.pos.z();
                kf_ = last_kf_;
                LOG(WARNING) << "After Correct:\npos:" << kf_.get_x().pos << "\nrot:" << kf_.get_x().rot
                             << "\nvel:" << kf_.get_x().vel << "\nbg:" << kf_.get_x().bg << "\nba:" << kf_.get_x().ba;
            }
            p_odom_->updateState(kf_);
        }
        last_pose_update_time = measures_.sensor_measure_time;
    }
}

void unidrive_goins::savePose() {
    // save the state
    auto imu_pose = p_imu_->getPose();
    state_point_ = kf_.get_x();
    euler_cur_ = SO3ToEuler(state_point_.rot);
    pos_ = state_point_.pos;
    if(print_debug_info_)
    {
        LOG(INFO) << "Saving pose!";
        LOG(INFO) << "Current pose: " << pos_.transpose();
    }
    if (save_imu_pose) {
        for (auto p : imu_pose) {
            Path temp_trans;
            temp_trans.timestamp = p.timestamp;
            temp_trans.pose = p.pose;
            temp_trans.Rotation = p.rot;
            trajtory.emplace_back(temp_trans);
            trajtory_all.emplace_back(temp_trans);
        }
    }
    Path temp_path;
    temp_path.timestamp = measures_.sensor_measure_time;
    temp_path.pose = pos_;
    double current_yaw = odom_measure.current_yaw;
    Eigen::AngleAxisd yawAngle(current_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond euler_rot(yawAngle);
    SO3 new_rot(euler_rot);
    temp_path.Rotation = new_rot.toRotationMatrix();
    // temp_path.Rotation = state_point_.rot.toRotationMatrix();
    if (trajtory.size() >= traj_buffer_length) trajtory.pop_front();
    trajtory.emplace_back(temp_path);
    trajtory_all.emplace_back(temp_path);
}

void unidrive_goins::Finish() {
    LOG(INFO) << "finish done";
    LOG(INFO) << "offset RIC:\n"
              << state_point_.offset_R_C_I.toRotationMatrix() << "\ntic:\n"
              << state_point_.offset_T_C_I;
    LOG(INFO) << "offset RIG:\n"
              << state_point_.offset_R_GNSS_I.toRotationMatrix() << "\ntig:\n"
              << state_point_.offset_T_GNSS_I;
}

void unidrive_goins::inputIMUData(ImuData &imu_data) {
    mtx_buffer_.lock();
    if (imu_data.measure_time < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }
    // LOG(INFO) << "IMU buffer size: " << imu_buffer_.size() << " measure time: " << imu_data.measure_time;
    last_timestamp_imu_ = imu_data.measure_time;
    imu_buffer_.emplace_back(imu_data);
    mtx_buffer_.unlock();
}
void unidrive_goins::inputOdomData(WheelData &odom_data) {
    mtx_buffer_.lock();
    if (odom_data.measure_time < last_timestamp_odom_) {
        LOG(WARNING) << "odom loop back, clear buffer";
        odom_buffer_.clear();
    }
    last_timestamp_odom_ = odom_data.measure_time;
    odom_buffer_.emplace_back(odom_data);
    // LOG(INFO) << "odom buffer size: " << odom_buffer_.size();
    mtx_buffer_.unlock();
}
void unidrive_goins::inutGNSSData(GNSSData &gnss_data) {
    mtx_buffer_.lock();
    if (gnss_data.measure_time < last_timestamp_gnss_) {
        LOG(WARNING) << "gnss loop back, clear buffer";
        gnss_buffer_.clear();
    }
    last_timestamp_gnss_ = gnss_data.measure_time;
    gnss_buffer_.emplace_back(gnss_data);
    // LOG(INFO) << "gnss buffer size: " << gnss_buffer_.size();
    mtx_buffer_.unlock();
}

}  // namespace unidrive