#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mapping_manager.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mot_mapping::MappingRos>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
