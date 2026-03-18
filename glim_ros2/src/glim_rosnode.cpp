#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

#include <sstream>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  rclcpp::spin(glim);
  rclcpp::shutdown();

  //타임스탬프로 덤프 디렉토리 이름 생성
  std::string base_dump_dir = "/home/taehyun/go2_ws/src/glim/dump";
  auto make_stamp_ns = [](const rclcpp::Time& t) {
    return std::to_string(t.nanoseconds());
  };
  
  std::string dump_path = base_dump_dir + "/" + make_stamp_ns(glim->get_clock()->now());
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  glim->wait();
  glim->save(dump_path);

  return 0;
}