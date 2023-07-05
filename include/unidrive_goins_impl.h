#pragma once
#include <boost/property_tree/ptree.hpp>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include "unidrive_goins.h"
namespace unidrive {
class unidrive_goins_impl {
   public:
    unidrive_goins_impl(bool print_debug_info = false);
    ~unidrive_goins_impl();
    bool Run();
    void Stop();
    void Reset();
    void Init(boost::property_tree::ptree &module_config, bool read_gnss_data = true);
    void addGNSSData(const unidrive::GNSSData &data);
    void addImuData(const unidrive::ImuData &data);
    void addWheelData(const unidrive::WheelData &data);
    Path getPose();
    std::deque<Path> getTrajc();
    inline double getInitTime() { return InitTime_; }
    inline void setOutputTrajPath(const std::string &path) { OutputTrajPath_ = path; }
    inline std::string getOutputTrajPath() { return OutputTrajPath_; }
    inline void setDebugInfo(bool flag) {
        IsPrintDebugInfo_ = flag;
        if (goins_) goins_->setDebugInfo(flag);
    }

   private:
    std::deque<unidrive::WheelData> WheelDatas_;
    std::deque<unidrive::ImuData> ImuDatas_;
    std::deque<unidrive::GNSSData> GNSSDatas_;
    std::shared_ptr<unidrive::unidrive_goins> goins_;
    std::ofstream RTKTraj_;
    unidrive::WheelData LastWheelData_;
    double InitTime_;
    bool IsFinished_ = false;
    bool IsPubReceived_ = false;
    bool IsFirstData_ = true;
    bool IsRunningWithFile_ = false;
    bool IsSaveTraj_ = false;
    bool IsProcessFinished = false;
    bool IsGNSSFusion_ = true;
    bool IsPrintDebugInfo_ = false;
    std::string OutputTrajPath_;
    std::mutex MtxBuffer_;
    double CutedMeanSpeed_;
};
}  // namespace unidrive
