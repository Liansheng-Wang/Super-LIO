
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>
#include <limits>


using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  g_save_map_dir = g_root_dir + g_save_map_dir;

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<bool>("lio.multi.enable", false);
  node.get_parameter("lio.multi.enable", g_multi_lidar_enable);

  node.declare_parameter<std::string>("lio.multi.lidar_topic_aux", "/lidar_aux");
  node.get_parameter("lio.multi.lidar_topic_aux", g_lidar_topic_aux);
  node.declare_parameter<std::string>("lio.multi.lidar2.topic", g_lidar_topic_aux);
  if (node.has_parameter("lio.multi.lidar2.topic")) {
    node.get_parameter("lio.multi.lidar2.topic", g_lidar_topic_aux);
  }

  node.declare_parameter<int>("lio.multi.lidar_type_aux", 1);
  node.get_parameter("lio.multi.lidar_type_aux", g_lidar_type_aux);
  node.declare_parameter<int>("lio.multi.lidar2.type", g_lidar_type_aux);
  if (node.has_parameter("lio.multi.lidar2.type")) {
    node.get_parameter("lio.multi.lidar2.type", g_lidar_type_aux);
  }

  node.declare_parameter<std::vector<double>>(
      "lio.multi.extrinsic_lidar2_lidar1", std::vector<double>(12, 0.0));
  node.get_parameter("lio.multi.extrinsic_lidar2_lidar1", g_extrinsic_lidar2_lidar1);
  node.declare_parameter<std::vector<double>>(
      "lio.multi.lidar2.extrinsic_lidar2_lidar1", g_extrinsic_lidar2_lidar1);
  if (node.has_parameter("lio.multi.lidar2.extrinsic_lidar2_lidar1")) {
    node.get_parameter("lio.multi.lidar2.extrinsic_lidar2_lidar1", g_extrinsic_lidar2_lidar1);
  }

  node.declare_parameter<double>("lio.multi.aux_time_sync_max_diff", -1.0);
  node.get_parameter("lio.multi.aux_time_sync_max_diff", g_aux_time_sync_max_diff);

  node.declare_parameter<int>("lio.multi.aux_buffer_max_size", 32);
  node.get_parameter("lio.multi.aux_buffer_max_size", g_aux_buffer_max_size);

  node.declare_parameter<std::string>("lio.multi.fused_topic", "/cloud_lidar2_filtered");
  node.get_parameter("lio.multi.fused_topic", g_multi_fused_topic);

  node.declare_parameter<std::string>("lio.multi.fused_frame_id", "imu");
  node.get_parameter("lio.multi.fused_frame_id", g_multi_fused_frame_id);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<int>("lio.sensor.imu_type", 0);
  node.get_parameter("lio.sensor.imu_type", g_imu_type);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // submaps
  node.declare_parameter<double>("lio.submap.submap_resolution", 0.0);
  node.get_parameter("lio.submap.submap_resolution", g_submap_resolution);

  node.declare_parameter<int>("lio.submap.submap_capacity", 0);
  node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.planner", false);
  node.get_parameter("lio.output.planner", g_planner_enable);

  node.declare_parameter<bool>("lio.output.plan_env_world", false);
  node.get_parameter("lio.output.plan_env_world", g_2_plan_env_world);

  node.declare_parameter<bool>("lio.output.plan_env_body", false);
  node.get_parameter("lio.output.plan_env_body", g_2_plan_env_body);

  node.declare_parameter<bool>("lio.output.ml_map", false);
  node.get_parameter("lio.output.ml_map", g_2_ml_map);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}

inline bool parseAuxExtrinsic(BASIC::M3& R, BASIC::V3& t)
{
  if (g_extrinsic_lidar2_lidar1.size() < 12) return false;
  t = BASIC::V3(
      static_cast<scalar>(g_extrinsic_lidar2_lidar1[0]),
      static_cast<scalar>(g_extrinsic_lidar2_lidar1[1]),
      static_cast<scalar>(g_extrinsic_lidar2_lidar1[2]));
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(g_extrinsic_lidar2_lidar1[3 + i]);
  }
  R = BASIC::M3(r_data.data());
  return true;
}


inline void transformAuxToPrimary(
    const pcl::PointCloud<LI2Sup::PointXTZIT>::Ptr& in_cloud,
    BASIC::CloudPtr& out_cloud)
{
  if (!in_cloud) return;
  BASIC::M3 R = BASIC::M3::Identity();
  BASIC::V3 t = BASIC::V3::Zero();
  if (!parseAuxExtrinsic(R, t)) {
    RCLCPP_WARN_ONCE(rclcpp::get_logger("super_lio"),
                     "lio.multi.extrinsic_lidar2_lidar1 invalid, fallback to identity.");
  }

  out_cloud->clear();
  out_cloud->reserve(in_cloud->size());
  for (const auto& pt : in_cloud->points) {
    BASIC::V3 p_aux(pt.x, pt.y, pt.z);
    BASIC::V3 p_primary = R * p_aux + t;
    out_cloud->emplace_back(p_primary.x(), p_primary.y(), p_primary.z(), pt.intensity);
  }
}


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  msg2uav_.header.frame_id = "world";
  path_.header.frame_id = "world";

  setupIO();
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_sensor_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(500))
                 .best_effort()
                 .durability_volatile();

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20))
                   .best_effort()
                   .durability_volatile();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt);
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt);
  }

  if (g_multi_lidar_enable) {
    if (g_lidar_type_aux == LID_TYPE::LIVOX) {
      sub_lidar_aux_ =
          this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
              g_lidar_topic_aux,
              lidar_qos,
              std::bind(&ROSWrapper::livoxAuxHandler, this, std::placeholders::_1),
              sub_opt);
    } else {
      sub_lidar_aux_std_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              g_lidar_topic_aux,
              lidar_qos,
              std::bind(&ROSWrapper::stdAuxHandler, this, std::placeholders::_1),
              sub_opt);
    }
  }

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", 10);
  pub_cloud_fusion_body_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        g_multi_fused_topic, 10);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc  = V3(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    imu_buffer_.push_back(data);
    last_timestamp_imu_ = data.secs;
    // eskf_->Reset();   // todo:
    return;
  }

  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();

      // ESKF::Predict(..., state_imu, state_robot) currently does not fill
      // state_robot.v/w/a (see todo in ESKF.cpp), so derive robo velocity from imu state.
      const V3 offset_imu = -g_odom_robo.R_ * g_odom_robo.t_;
      const V3 v_robo_world = imu_state.v + imu_state.R * (imu_state.w.cross(offset_imu));
      const V3 w_robo_world = imu_state.R * imu_state.w;

      // Odometry twist is expected in child frame ("robo"), so rotate world->robo.
      const V3 v_robo_local = robo_state.R.transpose() * v_robo_world;
      const V3 w_robo_local = robo_state.R.transpose() * w_robo_world;
      odom_robo.twist.twist.linear.x = v_robo_local(0);
      odom_robo.twist.twist.linear.y = v_robo_local(1);
      odom_robo.twist.twist.linear.z = v_robo_local(2);
      odom_robo.twist.twist.angular.x = w_robo_local(0);
      odom_robo.twist.twist.angular.y = w_robo_local(1);
      odom_robo.twist.twist.angular.z = w_robo_local(2);
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    odom_robo.child_frame_id = "robo";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = stampToSec(msg->header.stamp);
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_buffer_.push_back(lidar_data);
}

static bool parseStandardLidarMsg(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    int lidar_type,
    LidarData& lidar_data)
{
  double offset_time = 0.0;
  switch (lidar_type) {
  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    if (pl_orig.empty()) return false;
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    std::size_t first_valid_idx = 0;
    while (first_valid_idx < pl_orig.size() &&
           !std::isfinite(pl_orig.points[first_valid_idx].timestamp)) {
      ++first_valid_idx;
    }
    if (first_valid_idx >= pl_orig.size()) return false;
    const double time_begin = pl_orig.points[first_valid_idx].timestamp;
    lidar_data.start_time = time_begin;
    double max_offset_time = 0.0;
    for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      if (!std::isfinite(pt.timestamp) || !std::isfinite(pt.intensity)) continue;
      offset_time = pt.timestamp - time_begin;
      if (!std::isfinite(offset_time) || offset_time < 0.0) continue;
      if (offset_time > max_offset_time) max_offset_time = offset_time;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    if (lidar_data.pc->empty()) return false;
    lidar_data.end_time = time_begin + max_offset_time;
    return true;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    return true;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    if (!lidar_data.pc->empty()) {
      lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    } else {
      lidar_data.end_time = lidar_data.start_time;
    }
    return true;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    return true;
  }
  default:
    return false;
  }
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  if (!parseStandardLidarMsg(msg, g_lidar_type, lidar_data)) return;
  
  lidar_buffer_.push_back(lidar_data);
}

void ROSWrapper::livoxAuxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if (!g_multi_lidar_enable || msg->point_num < 10) return;
  LidarData aux_data;
  aux_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  aux_data.pc->reserve(msg->point_num / g_filter_rate + 1);

  for (std::size_t i = 0; i < msg->point_num; i += g_filter_rate) {
    const auto& pt = msg->points[i];
    const auto tag = pt.tag & 0x30;
    if (!(tag == 0x10 || tag == 0x00)) continue;
    const double dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (dis <= g_blind2 || dis >= g_maxrange2) continue;
    aux_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, 0.0);
  }
  if (aux_data.pc->empty()) return;

  BASIC::CloudPtr aux_primary(new BASIC::PointCloudType());
  transformAuxToPrimary(aux_data.pc, aux_primary);
  if (aux_primary->empty()) return;

  AuxLidarFrame f;
  f.cloud_lidar_primary = aux_primary;
  f.stamp_sec = stampToSec(msg->header.stamp);
  lidar_aux_buffer_.push_back(std::move(f));
  while (g_aux_buffer_max_size > 0 &&
         static_cast<int>(lidar_aux_buffer_.size()) > g_aux_buffer_max_size) {
    lidar_aux_buffer_.pop_front();
  }
}


void ROSWrapper::stdAuxHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if (!g_multi_lidar_enable || msg->data.size() < 10) return;
  LidarData aux_data;
  aux_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  if (!parseStandardLidarMsg(msg, g_lidar_type_aux, aux_data)) return;
  if (!aux_data.pc || aux_data.pc->empty()) return;

  BASIC::CloudPtr aux_primary(new BASIC::PointCloudType());
  transformAuxToPrimary(aux_data.pc, aux_primary);
  if (aux_primary->empty()) return;

  AuxLidarFrame f;
  f.cloud_lidar_primary = aux_primary;
  f.stamp_sec = stampToSec(msg->header.stamp);
  lidar_aux_buffer_.push_back(std::move(f));
  while (g_aux_buffer_max_size > 0 &&
         static_cast<int>(lidar_aux_buffer_.size()) > g_aux_buffer_max_size) {
    lidar_aux_buffer_.pop_front();
  }
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if (!std::isfinite(meas.lidar.start_time) || !std::isfinite(meas.lidar.end_time)) {
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (meas.lidar.end_time < meas.lidar.start_time) {
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = "imu";

  const double tx = state.p[0];
  const double ty = state.p[1];
  const double tz = state.p[2];
  const double qx = temp_q.x();
  const double qy = temp_q.y();
  const double qz = temp_q.z();
  const double qw = temp_q.w();
  const bool valid_tf = std::isfinite(tx) && std::isfinite(ty) && std::isfinite(tz) &&
                        std::isfinite(qx) && std::isfinite(qy) &&
                        std::isfinite(qz) && std::isfinite(qw);

  if (valid_tf) {
    tf_msg.transform.translation.x = tx;
    tf_msg.transform.translation.y = ty;
    tf_msg.transform.translation.z = tz;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf_msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skip invalid world->imu TF publish (contains NaN/Inf).");
  }

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_fusion(const BASIC::CloudPtr& pc_body_primary, double time){
  if (!g_multi_lidar_enable || !pub_cloud_fusion_body_ || !pc_body_primary) return;

  BASIC::CloudPtr aux_pick;
  double best_abs = std::numeric_limits<double>::infinity();
  for (const auto& f : lidar_aux_buffer_) {
    const double dt = std::fabs(f.stamp_sec - time);
    if (dt < best_abs) {
      best_abs = dt;
      aux_pick = f.cloud_lidar_primary;
    }
  }

  if (!aux_pick || aux_pick->empty()) return;
  if (g_aux_time_sync_max_diff > 0.0 && best_abs > g_aux_time_sync_max_diff) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Aux lidar frame rejected, |dt|=%.4fs > lio.multi.aux_time_sync_max_diff=%.4fs",
      best_abs, g_aux_time_sync_max_diff);
    return;
  }

  BASIC::CloudPtr merged(new BASIC::PointCloudType());
  merged->reserve(pc_body_primary->size() + aux_pick->size());
  *merged += *pc_body_primary;

  const BASIC::M3& R_li = g_lidar_imu.R_;
  const BASIC::V3& t_li = g_lidar_imu.t_;
  for (const auto& pt : aux_pick->points) {
    BASIC::V3 p_lidar(pt.x, pt.y, pt.z);
    BASIC::V3 p_imu = R_li * p_lidar + t_li;
    merged->emplace_back(p_imu.x(), p_imu.y(), p_imu.z(), pt.intensity);
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*merged, msg);
  msg.header.stamp = toRosTime(time);
  msg.header.frame_id = g_multi_fused_frame_id;
  pub_cloud_fusion_body_->publish(msg);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "world";

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              0.2;

          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          Quat init_rotation(w, x, y, z);

          init_pose = BASIC::SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

          flg_get_init_guess = true;

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: "
                  << init_translation.transpose()
                  << " yaw: "
                  << init_rotation.toRotationMatrix()
                          .eulerAngles(0, 1, 2)
                          .transpose()
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}


} // namespace END.