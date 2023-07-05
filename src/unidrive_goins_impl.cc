#include "unidrive_goins_impl.h"

namespace unidrive {

unidrive_goins_impl::unidrive_goins_impl(bool print_debug_info):IsPrintDebugInfo_(print_debug_info) {
    // std::cout << "unidrive_goins_impl::unidrive_goins_impl()" << std::endl;
    LastWheelData_.measure_time = 0.0;
    LastWheelData_.wheel_speed << 0.0, 0.0;
}

unidrive_goins_impl::~unidrive_goins_impl() {
    // std::cout << "unidrive_goins_impl::~unidrive_goins_impl()" << std::endl;
}

bool unidrive_goins_impl::Run() {
    unidrive::WheelData odom_input = LastWheelData_;
    if (!IsProcessFinished) {
        if (IsRunningWithFile_ && (ImuDatas_.empty() || WheelDatas_.empty())) {
            LOG(INFO) << "finished";
            goins_->Finish();
            IsProcessFinished = true;
            if (IsSaveTraj_) {
                LOG(INFO) << "save trajectory to: " << OutputTrajPath_;
                goins_->Savetrajectory(OutputTrajPath_);
            }
            return false;
        } else {
            bool is_vec_empty_ = true;
            MtxBuffer_.lock();
            is_vec_empty_ = ImuDatas_.empty();
            auto imu_data = ImuDatas_.front();
            MtxBuffer_.unlock();
            // LOG(INFO) << "mVecImuDatas empty?: " << is_vec_empty_;
            if (!is_vec_empty_) {
                if (imu_data.measure_time < InitTime_) InitTime_ = imu_data.measure_time;
                imu_data.measure_time = imu_data.measure_time - InitTime_;
                odom_input.measure_time = imu_data.measure_time;
                bool ZUPT = false;
                if (abs(imu_data.acc.x()) < 0.1 && abs(imu_data.acc.y()) < 0.1) {
                    ZUPT = true;
                    imu_data.gyr << 1e-6, 1e-6, 1e-6;
                }
                goins_->inputIMUData(imu_data);
                MtxBuffer_.lock();
                bool is_next_imu_empty = ImuDatas_.empty();
                bool is_odom_empty = WheelDatas_.empty();
                unidrive::WheelData now_odom_data;
                unidrive::ImuData now_imu_data;
                if (!is_next_imu_empty && !is_odom_empty) {
                    now_odom_data = WheelDatas_.front();
                    now_imu_data = ImuDatas_.front();
                }
                // if (IsPubReceived_) {
                //   std::shared_ptr<Localization_proto> proto_localization_msg =
                //       std::make_shared<Localization_proto>(
                //           received_rtk_messages_.front());
                //   IMU_RTK_writer_->Write(proto_localization_msg);
                //   received_rtk_messages_.pop_front();
                // }
                MtxBuffer_.unlock();
                if (!is_next_imu_empty && !is_odom_empty && now_imu_data.measure_time > now_odom_data.measure_time) {
                    double speed = ((now_odom_data.wheel_speed.x() + now_odom_data.wheel_speed.y()) * 0.5) /
                                   3.6;  // 3.6 km/h --> m/s
                    double omega = now_imu_data.gyr.z();
                    if (now_odom_data.state_left == unidrive::STANDSTILL &&
                        now_odom_data.state_right == unidrive::STANDSTILL) {
                        ZUPT = true;
                        omega = 1e-6;
                    } else if (abs(speed) < 0.01) {
                        // the sensor output is zero,but car has low speed
                        LOG(WARNING) << "The sensor output is zero,but car has low speed!\n";
                        int sign_direct = now_odom_data.state_left == unidrive::BACKWARD ? -1 : 1;
                        speed = sign_direct * CutedMeanSpeed_ / 3.6;  // assume mean speed is CutedMeanSpeed (km/h)
                        LOG(WARNING) << "Setting speed to:" << speed << std::endl;
                        omega = 1e-6;
                    }
                    if (now_odom_data.measure_time < InitTime_) InitTime_ = now_odom_data.measure_time;
                    odom_input.measure_time = now_odom_data.measure_time - InitTime_;
                    odom_input.wheel_speed << speed, omega;
                    MtxBuffer_.lock();
                    WheelDatas_.pop_front();
                    //   if (IsPubReceived_) {
                    //     std::shared_ptr<Chassis_proto> proto_chassis_msg =
                    //         std::make_shared<Chassis_proto>(
                    //             received_chassis_messages_.front());
                    //     chassis_writer_->Write(proto_chassis_msg);
                    //     received_chassis_messages_.pop_front();
                    //   }
                    MtxBuffer_.unlock();
                }
                MtxBuffer_.lock();
                ImuDatas_.pop_front();
                MtxBuffer_.unlock();
                unidrive::WheelData odom_input_copy = odom_input;
                LastWheelData_ = odom_input_copy;
                goins_->inputOdomData(odom_input_copy);
                goins_->setZUPT(ZUPT);
                goins_->handleOdomIntegrate(odom_input_copy.measure_time);
                // handle gnss
                MtxBuffer_.lock();
                bool is_gnss_buffer_empty = GNSSDatas_.empty();
                unidrive::GNSSData gnss_data;
                if (!is_gnss_buffer_empty) {
                    gnss_data = GNSSDatas_.front();
                }
                MtxBuffer_.unlock();
                if (!is_gnss_buffer_empty && !is_next_imu_empty && gnss_data.measure_time < now_imu_data.measure_time && IsGNSSFusion_) {
                    if (!ZUPT) {
                        goins_->inutGNSSData(gnss_data);
                        goins_->handleGNSSMeasure(gnss_data.measure_time);
                    }
                    MtxBuffer_.lock();
                    GNSSDatas_.pop_front();
                    MtxBuffer_.unlock();
                }
                goins_->savePose();
                // publish traj
                // {
                //   auto current_pose = goins_->getPose();

                //   current_pose.timestamp =
                //       current_pose.timestamp + InitTime_; // return the real time
                //   LocalizationEstimate odom_msg;
                //   odom_msg.set_measurement_time(current_pose.timestamp);
                //   auto header = new unidrive::proto::common::Header();
                //   header->set_timestamp_sec(current_pose.timestamp);
                //   odom_msg.set_allocated_header(header);
                //   auto history_trajc = goins_->getTrajc();
                //   // push pose data
                //   for (auto pose : history_trajc) {
                //     unidrive::proto::localization::OdometryPose *addOdom =
                //         odom_msg.add_odometry();
                //     double time = pose.timestamp + InitTime_;
                //     addOdom->set_measurement_time(time);
                //     auto addPose = addOdom->mutable_pose();
                //     auto addTrans = addPose->mutable_position();
                //     addTrans->set_x(pose.pose.x());
                //     addTrans->set_y(pose.pose.y());
                //     addTrans->set_z(pose.pose.z());
                //     auto addQuatern = addPose->mutable_orientation();
                //     Eigen::Quaterniond quatern(pose.Rotation);
                //     addQuatern->set_qx(quatern.x());
                //     addQuatern->set_qy(quatern.y());
                //     addQuatern->set_qz(quatern.z());
                //     addQuatern->set_qw(quatern.w());
                //   }
                //   std::shared_ptr<Localization_proto> proto_msg =
                //       std::make_shared<Localization_proto>(odom_msg);
                //   odom_writer_->Write(proto_msg);
                // }
                return true;
            } else
                return false;
        }
    }
}

void unidrive_goins_impl::Stop() {
    if (!IsRunningWithFile_) {
        LOG(INFO) << "goins_ finished";
        goins_->Finish();
        IsProcessFinished = true;
        if (IsSaveTraj_) {
            LOG(INFO) << "save trajectory to: " << OutputTrajPath_;
            goins_->Savetrajectory(OutputTrajPath_);
            LOG(INFO) << "save trajectory finished";
        }
    }
}

void unidrive_goins_impl::Reset() {}

void unidrive_goins_impl::Init(boost::property_tree::ptree &module_config,bool read_gnss_data) {
    IsGNSSFusion_ = read_gnss_data;
    goins_ = std::make_shared<unidrive::unidrive_goins>(IsPrintDebugInfo_);
    std::string goins_cfg = module_config.get<std::string>("params");
    goins_->initFromFile(goins_cfg);
    goins_->resetGOINS(true);
    setDebugInfo(module_config.get<bool>("PrintDebugInfo"));
    IsRunningWithFile_ = module_config.get<bool>("RunOffLineWithFIle");
    IsPubReceived_ = module_config.get<bool>("IsPubReceived");
    IsSaveTraj_ = module_config.get<bool>("TrajOutput");
    OutputTrajPath_ = module_config.get<std::string>("TrajOutput_Path");
    CutedMeanSpeed_ = module_config.get<double>("CutedMeanSpeed");
    LOG(INFO) << "Readed CutedMeanSpeed: " << CutedMeanSpeed_ << std::endl;
    bool is_odom_data_have_direct = module_config.get<bool>("IsChassDataHaveDirection");
    InitTime_ = 0;
    if (IsRunningWithFile_) {
        std::string chassisfileName;
        std::string imufileName;
        chassisfileName = module_config.get<std::string>("Chassisfile");
        imufileName = module_config.get<std::string>("Imufile");
        std::ifstream chassisfile(chassisfileName);
        std::string line;
        unidrive::WheelData odom_data;
        unidrive::ImuData imu_data;
        while (getline(chassisfile, line)) {
            std::stringstream ss(line);
            ss >> odom_data.measure_time;
            if (IsFirstData_) {
                IsFirstData_ = false;
                InitTime_ = odom_data.measure_time;
            }
            if (!is_odom_data_have_direct) {
                double rear_right_v, rear_left_v;
                ss >> rear_right_v >> rear_left_v;
                odom_data.wheel_speed << rear_left_v, rear_right_v;
            } else {
                int valid_rr, valid_rl;
                int direct_code_rr, direct_code_rl;
                double speed_rr, speed_rl;
                ss >> valid_rr >> direct_code_rr >> speed_rr >> valid_rl >> direct_code_rl >> speed_rl;
                odom_data.state_left = unidrive::WheelState(direct_code_rl);
                odom_data.state_right = unidrive::WheelState(direct_code_rr);
                if (odom_data.state_left == unidrive::BACKWARD) speed_rl *= -1;
                if (odom_data.state_right == unidrive::BACKWARD) speed_rr *= -1;
                odom_data.wheel_speed << speed_rl, speed_rr;
                // if (IsPubReceived_) {
                //     Chassis_proto chassis_msg;
                //     auto header = new unidrive::proto::common::Header();
                //     header->set_timestamp_sec(odom_data.measure_time);
                //     chassis_msg.set_allocated_header(header);
                //     auto whell_speed = chassis_msg.mutable_wheel_speed();
                //     whell_speed->set_is_wheel_spd_rr_valid(valid_rr);
                //     whell_speed->set_wheel_direction_rr(
                //         unidrive::proto::canbus::WheelSpeed_WheelSpeedType(direct_code_rr));
                //     whell_speed->set_wheel_spd_rr(speed_rr);
                //     whell_speed->set_is_wheel_spd_rl_valid(valid_rl);
                //     whell_speed->set_wheel_direction_rl(
                //         unidrive::proto::canbus::WheelSpeed_WheelSpeedType(direct_code_rl));
                //     whell_speed->set_wheel_spd_rl(speed_rl);
                //     received_chassis_messages_.push_back(chassis_msg);
                // }
            }
            WheelDatas_.push_back(odom_data);
        }
        std::ifstream imufile(imufileName);
        IsFirstData_ = true;
        while (std::getline(imufile, line)) {
            std::stringstream ss(line);
            double lat, lon, h;
            double yaw, pitch, roll;
            double acc_x, acc_y, acc_z;
            double gyro_x, gyro_y, gyro_z;

            ss >> imu_data.measure_time >> lat >> lon >> h >> yaw >> pitch >> roll >> acc_x >> acc_y >> acc_z >>
                gyro_x >> gyro_y >> gyro_z;
            // publish input data (debug)
            // if (IsPubReceived_) {
            //     // RTK message
            //     LocalizationEstimate rtk_receiver_msg;
            //     rtk_receiver_msg.set_measurement_time(imu_data.measure_time);
            //     auto header = new unidrive::proto::common::Header();
            //     header->set_timestamp_sec(imu_data.measure_time);
            //     rtk_receiver_msg.set_allocated_header(header);
            //     auto pose = rtk_receiver_msg.mutable_pose();
            //     auto ins_data = pose->mutable_asensing_ins();
            //     ins_data->mutable_ins_acc_500()->set_ins_acc_x(acc_x);
            //     ins_data->mutable_ins_acc_500()->set_ins_acc_y(acc_y);
            //     ins_data->mutable_ins_acc_500()->set_ins_acc_z(acc_z);
            //     ins_data->mutable_ins_gyro_501()->set_ins_gyro_x(gyro_x);
            //     ins_data->mutable_ins_gyro_501()->set_ins_gyro_y(gyro_y);
            //     ins_data->mutable_ins_gyro_501()->set_ins_gyro_z(gyro_z);
            //     ins_data->mutable_ins_headingpitchroll_502()->set_ins_headingangle(yaw);
            //     ins_data->mutable_ins_headingpitchroll_502()->set_ins_pitchangle(pitch);
            //     ins_data->mutable_ins_headingpitchroll_502()->set_ins_rollangle(roll);
            //     ins_data->mutable_ins_heightandtime_503()->set_ins_locatheight(h);
            //     ins_data->mutable_ins_latitudelongitude_504()->set_ins_latitude(lat);
            //     ins_data->mutable_ins_latitudelongitude_504()->set_ins_longitude(lon);
            //     received_rtk_messages_.push_back(rtk_receiver_msg);
            // }
            static size_t counter = 0;
            unidrive::GNSSData gnss_data;
            counter++;
            if (counter % 200 == 0 && read_gnss_data) {
                gnss_data.lat = lat;
                gnss_data.lon = lon;
                gnss_data.h = h;
                gnss_data.measure_time = imu_data.measure_time;
                GNSSDatas_.push_back(gnss_data);
            }

            if (IsFirstData_) {
                IsFirstData_ = false;
                if (imu_data.measure_time < InitTime_) InitTime_ = imu_data.measure_time;
            }
            imu_data.acc << acc_y, acc_x, -acc_z;
            imu_data.gyr << gyro_y * deg2Rad, gyro_x * deg2Rad, -gyro_z * deg2Rad;
            ImuDatas_.push_back(imu_data);
        }
        chassisfile.close();
        imufile.close();
    }
}

void unidrive_goins_impl::addGNSSData(const unidrive::GNSSData &gnss_data) {
    MtxBuffer_.lock();
    if (IsFirstData_) {
        IsFirstData_ = false;
        InitTime_ = gnss_data.measure_time;
    }
    GNSSDatas_.push_back(gnss_data);
    MtxBuffer_.unlock();
}

void unidrive_goins_impl::addWheelData(const unidrive::WheelData &wheel_data) {
    MtxBuffer_.lock();
    if (IsFirstData_) {
        IsFirstData_ = false;
        InitTime_ = wheel_data.measure_time;
    }
    WheelDatas_.push_back(wheel_data);
    MtxBuffer_.unlock();
}

void unidrive_goins_impl::addImuData(const unidrive::ImuData &imu_data) {
    MtxBuffer_.lock();
    if (IsFirstData_) {
        IsFirstData_ = false;
        InitTime_ = imu_data.measure_time;
    }
    ImuDatas_.push_back(imu_data);
    MtxBuffer_.unlock();
}

Path unidrive_goins_impl::getPose() {
    auto pose = goins_->getPose();
    return pose;
}

std::deque<Path> unidrive_goins_impl::getTrajc() { return goins_->getTrajc(); }
};  // namespace unidrive