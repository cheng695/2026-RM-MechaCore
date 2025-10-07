#include "odometry.hpp"
#include <vector>

odometry::angle_odometry Accumulate_8191{8191.0f, 8191.0f};
odometry::angle_odometry Accumulate_360 {360.0f , 8192.0f};
odometry::angle_odometry* Accumulate_angle = &Accumulate_8191;
