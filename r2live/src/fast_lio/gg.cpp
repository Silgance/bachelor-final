#pragma once
#include "gg.hpp"

#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>
#include <ros/ros.h>

void Fast_lio::SigHandle()
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void Fast_lio::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}