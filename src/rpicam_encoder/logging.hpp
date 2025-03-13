#include "rclcpp/logging.hpp"

#define LOG(level, text)                                                                                               \
	RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rpi_cam"), text)
#define LOG_ERROR(text) RCLCPP_ERROR_STREAM(rclcpp::get_logger("rpi_cam"), text)
