
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


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

  node.declare_parameter<bool>("lio.loop.enable_keyframe_pub", false);
  node.get_parameter("lio.loop.enable_keyframe_pub", g_enable_keyframe_pub);

  node.declare_parameter<double>("lio.loop.kf_trans_thresh", 1.0);
  node.get_parameter("lio.loop.kf_trans_thresh", g_loop_kf_trans_thresh);

  node.declare_parameter<double>("lio.loop.kf_rot_thresh", 15.0);
  node.get_parameter("lio.loop.kf_rot_thresh", g_loop_kf_rot_thresh);

  node.declare_parameter<std::string>(
    "lio.loop.keyframe_topic", "/super_lio/keyframe");
  node.get_parameter("lio.loop.keyframe_topic", g_loop_keyframe_topic);

  node.declare_parameter<bool>("lio.loop.enable_backend", false);
  node.get_parameter("lio.loop.enable_backend", g_enable_backend);

  node.declare_parameter<double>("lio.loop.backend_service_timeout", 5.0);
  node.get_parameter(
    "lio.loop.backend_service_timeout", g_backend_service_timeout);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  node.declare_parameter<int>("lio.sensor.scan_line", 96);
  node.get_parameter("lio.sensor.scan_line", g_scan_line);

  node.declare_parameter<int>("lio.sensor.timestamp_unit", 0);
  node.get_parameter("lio.sensor.timestamp_unit", g_timestamp_unit);

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

  node.declare_parameter<bool>("lio.output.body_cloud", false);
  node.get_parameter("lio.output.body_cloud", g_pub_body_cloud);

  // false: come up on standby and wait for ~/set_active. Each activation starts
  // a fresh session (empty map, re-estimated gravity and bias).
  node.declare_parameter<bool>("lio.control.start_active", true);
  node.get_parameter("lio.control.start_active", g_start_active);

  node.declare_parameter<std::string>(
    "lio.ros.body_cloud_topic", "/lio/cloud_body");
  node.get_parameter("lio.ros.body_cloud_topic", g_body_cloud_topic);

  node.declare_parameter<std::string>(
    "lio.ros.body_odom_topic", "/lio/odom_body");
  node.get_parameter("lio.ros.body_odom_topic", g_body_odom_topic);

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


void ROSWrapper::createSensorSubscriptions(){
  if (sub_imu_) {
    return;
  }

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
}


void ROSWrapper::destroySensorSubscriptions(){
  sub_imu_.reset();
  sub_lidar_.reset();
  sub_lidar_std_.reset();
}


void ROSWrapper::setActive(bool active){
  if (active == is_active()) {
    return;
  }
  if (active) {
    // Buffers first: whatever is still queued predates this session. process()
    // sees the edge on its next tick and wipes the map and filter too.
    clear();
    createSensorSubscriptions();
    active_.store(true, std::memory_order_release);
    LOG(INFO) << GREEN << " ---> [Control] active" << RESET;
  } else {
    active_.store(false, std::memory_order_release);
    destroySensorSubscriptions();
    clear();
    LOG(INFO) << YELLOW << " ---> [Control] standby" << RESET;
  }
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  active_.store(g_start_active, std::memory_order_release);
  if (g_start_active) {
    createSensorSubscriptions();
  } else {
    LOG(INFO) << YELLOW << " ---> [Control] standby; call "
              << std::string(this->get_name()) << "/set_active to start"
              << RESET;
  }

  set_active_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "~/set_active",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        const bool was = is_active();
        setActive(req->data);
        res->success = true;
        res->message = std::string(req->data ? "active" : "standby") +
                       (was == req->data ? " (unchanged)" : "");
      });

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

  if (g_pub_body_cloud) {
    pub_cloud_body_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          g_body_cloud_topic, rclcpp::QoS(10).reliable());
    pub_body_odom_ =
      this->create_publisher<nav_msgs::msg::Odometry>(
          g_body_odom_topic, rclcpp::QoS(50).reliable());
    LOG(INFO) << GREEN << " ---> [Output] undistorted body cloud: "
              << g_body_cloud_topic << " + " << g_body_odom_topic << RESET;
  }

  if (g_enable_keyframe_pub || g_enable_backend) {
    pub_keyframe_ =
      this->create_publisher<super_lio_loop_msgs::msg::Keyframe>(
        g_loop_keyframe_topic, rclcpp::QoS(20).reliable());
  }
  if (g_enable_backend) {
    corrected_pose_client_ =
      this->create_client<super_lio_loop_msgs::srv::GetCorrectedPoses>(
        "/super_lio_loop_backend/get_corrected_poses");
  }

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
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  if(!msg->header.frame_id.empty()) lidar_frame_id_ = msg->header.frame_id;
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

/**
* 不同雷达处理方法
*/
void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  if(!msg->header.frame_id.empty()) lidar_frame_id_ = msg->header.frame_id;

  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::LIVOX_PC2:
  {
    // livox_ros_driver2 with xfer_format: 0 — the same stream as CustomMsg but
    // as a PointCloud2. `timestamp` is float64; some driver builds emit it as an
    // absolute epoch time, others as an offset from header.stamp, so detect
    // which one it is instead of adding another knob. `tag` is kept for the same
    // return-quality gate livoxHandler applies.
    double timestamp_unit_scale;
    switch (g_timestamp_unit) {
      case 1: timestamp_unit_scale = 1e-3; break;  // milliseconds
      case 2: timestamp_unit_scale = 1e-6; break;  // microseconds
      case 3: timestamp_unit_scale = 1e-9; break;  // nanoseconds
      default: timestamp_unit_scale = 1.0;  break;  // seconds
    }

    pcl::PointCloud<livox_pc2::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    const std::size_t plsize = pl_orig.size();
    if (plsize == 0) return;

    lidar_data.pc->reserve(plsize / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    // Absolute epoch stamps are >> any plausible per-scan offset.
    const double first_ts = pl_orig.points[0].timestamp * timestamp_unit_scale;
    const bool absolute_time = first_ts > 1.0e8;
    const double time_base = absolute_time ? first_ts : 0.0;
    if (absolute_time) lidar_data.start_time = first_ts;

    for (std::size_t i = 0; i < plsize; i += g_filter_rate) {
      auto& pt = pl_orig.points[i];
      const auto tag = pt.tag & 0x30;
      if (tag != 0x10 && tag != 0x00) continue;
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp * timestamp_unit_scale - time_base;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::RS_AIRY:
  {
    // Compute timestamp unit scale: convert input unit to seconds
    double timestamp_unit_scale;
    switch (g_timestamp_unit) {
      case 0: timestamp_unit_scale = 1.0;    break;  // seconds
      case 1: timestamp_unit_scale = 1e-3;   break;  // milliseconds
      case 2: timestamp_unit_scale = 1e-6;   break;  // microseconds
      case 3: timestamp_unit_scale = 1e-9;   break;  // nanoseconds
      default: timestamp_unit_scale = 1.0;   break;
    }

    pcl::PointCloud<robosenseM1_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    std::size_t plsize = pl_orig.size();
    if (plsize == 0) return;

    lidar_data.pc->reserve(plsize / g_filter_rate + 1);

    const double first_timestamp = pl_orig.points[0].timestamp;
    lidar_data.start_time = first_timestamp * timestamp_unit_scale;
    double end_offset = 0.0;

    // Organized point cloud: width = scan lines, height = points per line
    for (std::size_t i = 0; i < plsize; i += g_filter_rate) {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = (pt.timestamp - first_timestamp) * timestamp_unit_scale;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
      end_offset = offset_time;
    }
    lidar_data.end_time = lidar_data.start_time + end_offset;
    break;
  }
  default:
    return;
  }
  
  lidar_buffer_.push_back(lidar_data);
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
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

  tf_msg.transform.translation.x = state.p[0];
  tf_msg.transform.translation.y = state.p[1];
  tf_msg.transform.translation.z = state.p[2];

  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();

  tf_broadcaster_->sendTransform(tf_msg);

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


void ROSWrapper::pub_keyframe(
  std::uint32_t id, const NavState & state, const CloudPtr & body_cloud)
{
  if (!pub_keyframe_ || !body_cloud || body_cloud->empty()) {
    return;
  }

  super_lio_loop_msgs::msg::Keyframe message;
  message.header.stamp = toRosTime(state.timestamp);
  message.header.frame_id = "imu";
  message.id = id;
  message.world_pose.position.x = state.p[0];
  message.world_pose.position.y = state.p[1];
  message.world_pose.position.z = state.p[2];
  const V4 q = state.R.coeffs();
  message.world_pose.orientation.x = q[0];
  message.world_pose.orientation.y = q[1];
  message.world_pose.orientation.z = q[2];
  message.world_pose.orientation.w = q[3];
  pcl::toROSMsg(*body_cloud, message.cloud);
  message.cloud.header = message.header;
  pub_keyframe_->publish(message);
}


bool ROSWrapper::request_corrected_poses(
  std::vector<std::uint32_t> & ids,
  std::vector<geometry_msgs::msg::Pose> & poses,
  double timeout_seconds)
{
  ids.clear();
  poses.clear();
  if (!corrected_pose_client_ || !rclcpp::ok()) {
    return false;
  }

  const auto timeout = std::chrono::duration<double>(
    std::max(0.0, timeout_seconds));
  if (!corrected_pose_client_->wait_for_service(timeout)) {
    return false;
  }
  auto request =
    std::make_shared<super_lio_loop_msgs::srv::GetCorrectedPoses::Request>();
  auto future = corrected_pose_client_->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
    shared_from_this(), future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  const auto response = future.get();
  if (
    !response->success ||
    response->ids.size() != response->corrected_poses.size())
  {
    return false;
  }
  ids = response->ids;
  poses = response->corrected_poses;
  return true;
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


void ROSWrapper::pub_cloud_body_odom(const CloudPtr& body_imu_cloud,
   const NavState& state)
{
  if (!pub_cloud_body_ || !pub_body_odom_ || !body_imu_cloud ||
      body_imu_cloud->empty())
  {
    return;
  }

  // The undistorted scan comes out in the IMU frame at scan-end. Express it in
  // the LiDAR frame instead so a consumer can reach base_link through the
  // robot's existing static TF without duplicating the lidar<-imu extrinsic.
  const M3 R_li = g_lidar_imu.R_;
  const V3 t_li = g_lidar_imu.t_;
  const M3 R_il = R_li.transpose();

  PointCloudType lidar_cloud;
  lidar_cloud.resize(body_imu_cloud->size());
  for (std::size_t i = 0; i < body_imu_cloud->size(); ++i) {
    const auto& src = body_imu_cloud->points[i];
    const V3 p_lidar = R_il * (V3(src.x, src.y, src.z) - t_li);
    auto& dst = lidar_cloud.points[i];
    dst.x = p_lidar[0];
    dst.y = p_lidar[1];
    dst.z = p_lidar[2];
    dst.intensity = src.intensity;
  }
  lidar_cloud.width = lidar_cloud.size();
  lidar_cloud.height = 1;
  lidar_cloud.is_dense = false;

  const auto stamp = toRosTime(state.timestamp);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(lidar_cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = lidar_frame_id_;
  pub_cloud_body_->publish(cloud_msg);

  // world <- lidar on the identical stamp, so the consumer can pair the two by
  // timestamp without any TF interpolation.
  const M3 R_wl = state.R.R_ * R_li;
  const V3 t_wl = state.R.R_ * t_li + state.p;
  const Quat q_wl = Quat(R_wl).normalized();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "world";
  odom.child_frame_id = lidar_frame_id_;
  odom.pose.pose.position.x = t_wl[0];
  odom.pose.pose.position.y = t_wl[1];
  odom.pose.pose.position.z = t_wl[2];
  odom.pose.pose.orientation.x = q_wl.x();
  odom.pose.pose.orientation.y = q_wl.y();
  odom.pose.pose.orientation.z = q_wl.z();
  odom.pose.pose.orientation.w = q_wl.w();
  // World-frame IMU velocity, same convention as /lio/odom (not child-frame as
  // nav_msgs/Odometry nominally specifies).
  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];
  pub_body_odom_->publish(odom);
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
