#include <string>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto param_node = rclcpp::Node::make_shared("test_load_params");
  param_node->declare_parameter("test_string_param");
  std::string test_string;
  if (param_node->get_parameter<std::string>("test_string_param", test_string)) {
    RCLCPP_INFO(param_node->get_logger(), "String Parameter Value: %s", test_string.c_str());
  } else {
    RCLCPP_ERROR(param_node->get_logger(), "No string param found in node [%s]", param_node->get_fully_qualified_name());
  }
  rclcpp::shutdown();
  param_node = nullptr;
  return 0;
}
