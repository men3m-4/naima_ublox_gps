
#include <rclcpp/rclcpp.hpp>
// Ublox GPS includes
#include <ublox_gps/raw_data_pa.hpp>

//
// Raw Data Stream (feature from TUC-ProAut)
//

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ublox_node::RawDataStreamPa>(true);
  node->getRosParams();
  node->initialize();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
