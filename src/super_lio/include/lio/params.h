/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16, 7: ouster, 8: rs_airy
  extern int   g_scan_line;      // number of scan lines (96 for Airy)
  extern int   g_timestamp_unit; // 0=sec, 1=ms, 2=us, 3=ns
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern double g_kf_quit_eps;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;
  
  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern int  g_pub_step;

  /// for planner
  extern bool g_planner_enable;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;


  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

}

#endif
