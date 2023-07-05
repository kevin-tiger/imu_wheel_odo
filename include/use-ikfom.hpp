#ifndef SEMIDRIVE_GOINS_USE_IKFOM_H
#define SEMIDRIVE_GOINS_USE_IKFOM_H

#include "IKFoM_toolkit/esekfom/esekfom.hpp"

namespace unidrive {

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98150, 10000, 1> S2;  //模长 98150/10000 9.8150
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_ikfom, ((vect3, pos))((SO3, rot))((vect3, vel))((vect3, bg))((vect3, ba))((SO3, offset_R_C_I))((vect3, offset_T_C_I))((SO3, offset_R_GNSS_I))((vect3, offset_T_GNSS_I))((S2, grav)));
// // offset gnss imu means the init post w.r.t the reference internal cordinate
MTK_BUILD_MANIFOLD(state_ikfom_gnss, ((vect3, pos))((SO3, offset_R_GNSS_I))((vect3, offset_T_GNSS_I)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, acc))((vect3, gyro)));

MTK_BUILD_MANIFOLD(process_noise_ikfom, ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

inline MTK::get_cov<process_noise_ikfom>::type process_noise_cov() {
    MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
    MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);  // 0.03
    MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na,
                                                    0.0001);  // *dt 0.01 0.01 * dt * dt 0.05
    MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg,
                                                    0.00001);  // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
    MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba,
                                                    0.00001);  // 0.001 0.05 0.0001/out 0.01
    return cov;
}

// double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia
// vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
//名义状态的系统方程
inline Eigen::Matrix<double, 30, 1> get_f(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    vect3 a_inertial = s.rot * (in.acc - s.ba);
    for (int i = 0; i < 3; i++) {
        res(i) = s.vel[i]; //dP
        res(i + 3) = omega[i]; //dR
        res(i + 6) = a_inertial[i] + s.grav[i]; //dv
    }
    return res;
}

// Eigen::Matrix<double, 24, 1> get_f_gnss(state_ikfom_gnss &s, const input_ikfom &in) {
//     Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
//     vect3 omega;
//     in.gyro.boxminus(omega, s.bg);
//     vect3 a_inertial = s.rot * (in.acc - s.ba);
//     for (int i = 0; i < 3; i++) {
//         res(i) = s.vel[i];
//         res(i + 3) = omega[i];
//         res(i + 6) = a_inertial[i] + s.grav[i];
//     }
//     return res;
// }

//误差状态的传递方程
inline Eigen::Matrix<double, 30, 29> df_dx(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 30, 29> cov = Eigen::Matrix<double, 30, 29>::Zero();
    cov.template block<3, 3>(0, s.vel.IDX) = Eigen::Matrix3d::Identity();  // error_p w.r.t error_v
    vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    cov.template block<3, 3>(6, s.rot.IDX) = -s.rot.toRotationMatrix() * MTK::hat(acc_);  // error_v w.r.t error_rot
    cov.template block<3, 3>(6, s.ba.IDX) = -s.rot.toRotationMatrix();                  // error_v w.r.t error_ba
    Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
    Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
    s.S2_Mx(grav_matrix, vec, s.grav.IDX);
    cov.template block<3, 2>(6, s.grav.IDX) = grav_matrix;                  // error_v w.r.t error_g
    cov.template block<3, 3>(3, s.bg.IDX) = -Eigen::Matrix3d::Identity();  // error_rot w.r.t error_bg
    return cov;
}

// Eigen::Matrix<double, 24, 23> df_dx_gnss(state_ikfom_gnss &s, const input_ikfom &in) {
//     Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
//     cov.template block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
//     vect3 acc_;
//     in.acc.boxminus(acc_, s.ba);
//     vect3 omega;
//     in.gyro.boxminus(omega, s.bg);
//     cov.template block<3, 3>(6, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_);
//     cov.template block<3, 3>(6, 12) = -s.rot.toRotationMatrix();
//     Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
//     Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
//     s.S2_Mx(grav_matrix, vec, 15);
//     cov.template block<3, 2>(6, 15) = grav_matrix;
//     cov.template block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();
//     return cov;
// }

inline Eigen::Matrix<double, 30, 12> df_dw(state_ikfom &s, const input_ikfom &in) {
    Eigen::Matrix<double, 30, 12> cov = Eigen::Matrix<double, 30, 12>::Zero();
    cov.template block<3, 3>(s.rot.IDX, 0) = -Eigen::Matrix3d::Identity();  // error_rot w.r.t ng
    cov.template block<3, 3>(s.vel.IDX, 3) = -s.rot.toRotationMatrix();    // error_v w.r.t na
    cov.template block<3, 3>(s.bg.IDX, 6) = Eigen::Matrix3d::Identity(); // error_bg w.r.t nbg
    cov.template block<3, 3>(s.ba.IDX, 9) = Eigen::Matrix3d::Identity(); // error_ba w.r.t nba
    return cov;
}

// Eigen::Matrix<double, 24, 12> df_dw_gnss(state_ikfom_gnss &s, const input_ikfom &in) {
//     Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
//     cov.template block<3, 3>(6, 3) = -s.rot.toRotationMatrix();
//     cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
//     cov.template block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
//     cov.template block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
//     return cov;
// }

inline vect3 SO3ToEuler(const SO3 &orient) {
    Eigen::Matrix<double, 3, 1> _ang;
    Eigen::Vector4d q_data = orient.coeffs().transpose();
    // scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
    double sqw = q_data[3] * q_data[3];
    double sqx = q_data[0] * q_data[0];
    double sqy = q_data[1] * q_data[1];
    double sqz = q_data[2] * q_data[2];
    double unit = sqx + sqy + sqz + sqw;  // if normalized is one, otherwise is correction factor
    double test = q_data[3] * q_data[1] - q_data[2] * q_data[0];

    if (test > 0.49999 * unit) {  // singularity at north pole

        _ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI / 2, 0;
        double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
        vect3 euler_ang(temp, 3);
        return euler_ang;
    }
    if (test < -0.49999 * unit) {  // singularity at south pole
        _ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI / 2, 0;
        double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
        vect3 euler_ang(temp, 3);
        return euler_ang;
    }

    _ang << std::atan2(2 * q_data[0] * q_data[3] + 2 * q_data[1] * q_data[2], -sqx - sqy + sqz + sqw),
        std::asin(2 * test / unit),
        std::atan2(2 * q_data[2] * q_data[3] + 2 * q_data[1] * q_data[0], sqx - sqy - sqz + sqw);
    double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3}; //degree
    // double temp[3] = {_ang[0], _ang[1] , _ang[2]}; //rad
    vect3 euler_ang(temp, 3);
    // euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
    return euler_ang;
}

}  // namespace unidrive

#endif