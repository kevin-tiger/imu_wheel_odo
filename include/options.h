#ifndef SEMIDRIVE_OPTIONS_HPP
#define SEMIDRIVE_OPTIONS_HPP

namespace unidrive::options {

/// fixed params
constexpr double INIT_TIME = 0.25;
constexpr double ODOM_COV = 0.1;
constexpr double GNSS_COV = 5.0;
constexpr int PUBFRAME_PERIOD = 20;


/// configurable params
extern int NUM_MAX_ITERATIONS;      // max iterations of ekf
extern bool FLAG_EXIT;              // flag for exitting

}

#endif