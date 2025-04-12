

#ifndef UBLOX_GPS_FTS_PRODUCT_HPP
#define UBLOX_GPS_FTS_PRODUCT_HPP

#include <memory>

#include <ublox_gps/component_interface.hpp>
#include <ublox_gps/gps.hpp>

namespace ublox_node {

/**
 * @brief Implements functions for FTS products. Currently unimplemented.
 * @todo Unimplemented.
 */
class FtsProduct final : public virtual ComponentInterface {
  /**
   * @brief Get the FTS parameters.
   * @todo Currently unimplemented.
   */
  void getRosParams() override {
    // RCLCPP_WARN("Functionality specific to u-blox FTS devices is %s",
    //          "unimplemented. See FtsProduct class in node.hpp & node.cpp.");
  }

  /**
   * @brief Configure FTS settings.
   * @todo Currently unimplemented.
   */
  bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) override {
    (void)gps;
    return false;
  }

  /**
   * @brief Adds diagnostic updaters for FTS status.
   * @todo Currently unimplemented.
   */
  void initializeRosDiagnostics() override {}

  /**
   * @brief Subscribe to FTS messages.
   * @todo Currently unimplemented.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override {
    (void)gps;
  }
};

}  // namespace ublox_node

#endif
