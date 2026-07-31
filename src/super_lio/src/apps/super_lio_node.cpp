

#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros/ROSWrapper.h"
#include "lio/super_lio_reloc.h"


using namespace LI2Sup;

namespace
{
void signalHandler(int)
{
  g_flag_run = false;
}
}

int main(int argc, char** argv){
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  g_flag_run = true;

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  
  auto lio = std::make_shared<SuperLIO>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  auto timer = data_wrapper->create_wall_timer(
    std::chrono::milliseconds(2),
    [lio]() { lio->process(); },
    data_wrapper->getSensorCallbackGroup()
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(data_wrapper);
  while (rclcpp::ok() && g_flag_run) {
    executor.spin_once(std::chrono::milliseconds(20));
  }
  executor.remove_node(data_wrapper);

  lio->saveMap();
  lio->printTimeRecord();

  rclcpp::shutdown();
  return 0;
}
