#ifndef SEMIDRIVE_GOINS_UTILS_HPP
#define SEMIDRIVE_GOINS_UTILS_HPP
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/array.hpp>
// #include <unsupported/Eigen/ArpackSupport>

#include <glog/logging.h>
#include <chrono>
#include <fstream>
#include <map>
#include <numeric>
#include <string>

namespace unidrive {

constexpr double G_m_s2 = 9.8015;  // Gravity const in BeiJin/China

constexpr double deg2Rad = M_PI / 180.0;

using V2D = Eigen::Vector2d;
using V3D = Eigen::Vector3d;
using V4D = Eigen::Vector4d;
using V5D = Eigen::Matrix<double, 5, 1>;
using M2D = Eigen::Matrix2d;
using M3D = Eigen::Matrix3d;
using M4D = Eigen::Matrix4d;
using V3F = Eigen::Vector3f;
using V4F = Eigen::Vector4f;
using V5F = Eigen::Matrix<float, 5, 1>;
using M3F = Eigen::Matrix3f;
using M4F = Eigen::Matrix4f;

using VV3D = std::vector<V3D, Eigen::aligned_allocator<V3D>>;
using VV3F = std::vector<V3F, Eigen::aligned_allocator<V3F>>;
using VV4F = std::vector<V4F, Eigen::aligned_allocator<V4F>>;
using VV4D = std::vector<V4D, Eigen::aligned_allocator<V4D>>;
using VV5F = std::vector<V5F, Eigen::aligned_allocator<V5F>>;
using VV5D = std::vector<V5D, Eigen::aligned_allocator<V5D>>;

const M3D Eye3d = M3D::Identity();
const M3F Eye3f = M3F::Identity();
const V3D Zero3d(0, 0, 0);
const V3F Zero3f(0, 0, 0);

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const boost::array<S, 3> &v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const boost::array<S, 9> &v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1> &v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K = SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3) {
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001) {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    } else {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot) {
    T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if (!singular) {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    } else {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& name, double time_usage) {
            func_name_ = name;
            time_usage_in_ms_.emplace_back(time_usage);
        }
        std::string func_name_;
        std::vector<double> time_usage_in_ms_;
    };

    /**
     * call F and save its time usage
     * @tparam F
     * @param func
     * @param func_name
     */
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name) {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        if (records_.find(func_name) != records_.end()) {
            records_[func_name].time_usage_in_ms_.emplace_back(time_used);
        } else {
            records_.insert({func_name, TimerRecord(func_name, time_used)});
        }
    }

    /// print the run time
    static void PrintAll() {
        LOG(INFO) << ">>> ===== Printing run time =====";
        for (const auto& r : records_) {
            LOG(INFO) << "> [ " << r.first << " ] average time usage: "
                      << std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0) /
                             double(r.second.time_usage_in_ms_.size())
                      << " ms , called times: " << r.second.time_usage_in_ms_.size();
        }
        LOG(INFO) << ">>> ===== Printing run time end =====";
    }

    /// dump to a log file
    static void DumpIntoFile(const std::string& file_name) {
        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << file_name;
            return;
        } else {
            LOG(INFO) << "Dump Time Records into file: " << file_name;
        }

        size_t max_length = 0;
        for (const auto& iter : records_) {
            ofs << iter.first << ", ";
            if (iter.second.time_usage_in_ms_.size() > max_length) {
                max_length = iter.second.time_usage_in_ms_.size();
            }
        }
        ofs << std::endl;

        for (size_t i = 0; i < max_length; ++i) {
            for (const auto& iter : records_) {
                if (i < iter.second.time_usage_in_ms_.size()) {
                    ofs << iter.second.time_usage_in_ms_[i] << ",";
                } else {
                    ofs << ",";
                }
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    /// get the average time usage of a function
    static double GetMeanTime(const std::string& func_name) {
        if (records_.find(func_name) == records_.end()) {
            return 0.0;
        }

        auto r = records_[func_name];
        return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) /
               double(r.time_usage_in_ms_.size());
    }

    /// clean the records
    static void Clear() { records_.clear(); }

   private:
    static std::map<std::string, TimerRecord> records_;
};

}  // namespace unidrive

#endif