


#ifndef UBLOX_GPS_RAW_DATA_PA_HPP
#define UBLOX_GPS_RAW_DATA_PA_HPP

// STL
#include <fstream>
#include <string>

// ROS includes
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <std_msgs/msg/u_int8_multi_array.hpp>

/**
 * @namespace ublox_node
 * This namespace is for the ROS u-blox node and handles anything regarding
 * ROS parameters, message passing, diagnostics, etc.
 */
namespace ublox_node {

/**
 * @brief Implements functions for raw data stream.
 */
class RawDataStreamPa final : public rclcpp::Node {
 public:

  /**
   * @brief Constructor.
   * Initialises variables and the nodehandle.
   */
  explicit RawDataStreamPa(bool is_ros_subscriber = false);

  /**
   * @brief Get the raw data stream parameters.
   */
  void getRosParams();

  /**
   * @brief Returns the if raw data streaming is enabled.
   */
  bool isEnabled();

  /**
   * @brief Initializes raw data streams
   * If storing to file is enabled, the filename is created and the
   * corresponding filedescriptor will be opened.
   * If publishing ros messages is enabled, an empty msg will be published.
   * (This will implicitly create the publisher)
   */
  void initialize();

  /**
   * @brief Callback function which handles raw data.
   * @param data the buffer of u-blox messages to process
   * @param size the size of the buffer
   */
  void ubloxCallback(const unsigned char* data,
                     std::size_t size);

 private:
  /**
   * @brief Callback function which handles raw data.
   * @param msg ros message
   */
  void msgCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  /**
   * @brief Converts a string into an uint8 multibyte array
   */
  std_msgs::msg::UInt8MultiArray str2uint8(const std::string & str);

  /**
   * @brief Publishes data stream as ros message
   * @param str raw data stream as string
   */
  void publishMsg(const std::string & str);

  /**
   * @brief Stores data to given file
   * @param str raw data stream as string
   */
  void saveToFile(const std::string & str);

  //! Directory name for storing raw data
  std::string file_dir_;
  //! Filename for storing raw data
  std::string file_name_;
  //! Handle for file access
  std::ofstream file_handle_;

  //! Flag for publishing raw data
  bool flag_publish_;

  //! Internal flag
  //! true : subscribing to ros messages and storing those to file
  //! false: publishing ros messages and/or storing to file
  bool is_ros_subscriber_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_data_stream_sub_;
};

}  // namespace ublox_node

#endif
