#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  rclcpp::spin(glim);
  rclcpp::shutdown();

  // Generate dump directory name as: glim_map_YYYYMMDD_HHMMSS
  const std::string base_dump_dir = "/home/taehyun/go2_ws/src/glim/dump";
  auto make_dump_name = []() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_t = std::chrono::system_clock::to_time_t(now);

    std::tm local_tm {};
#ifdef _WIN32
    localtime_s(&local_tm, &now_t);
#else
    localtime_r(&now_t, &local_tm);
#endif

    std::ostringstream oss;
    oss << "glim_map_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    return oss.str();
  };

  const std::string base_dump_name = make_dump_name();
  std::string dump_name = base_dump_name;
  int suffix = 1;
  while (std::filesystem::exists(base_dump_dir + "/" + dump_name)) {
    dump_name = base_dump_name + "_" + std::to_string(suffix++);
  }

  std::string dump_path = base_dump_dir + "/" + dump_name;
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  glim->wait();
  glim->save(dump_path);

  return 0;
}
