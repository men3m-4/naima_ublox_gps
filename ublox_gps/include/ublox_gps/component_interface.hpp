

#ifndef UBLOX_GPS_COMPONENT_INTERFACE_HPP
#define UBLOX_GPS_COMPONENT_INTERFACE_HPP

#include <memory>

#include <ublox_gps/gps.hpp>

// This file declares the ComponentInterface which acts as a high level
// interface for u-blox firmware, product categories, etc. It contains methods
// to configure the u-blox and subscribe to u-blox messages.
//

namespace ublox_node {

/**
 * @brief This interface is used to add functionality to the main node.
 *
 * @details This interface is generic and can be implemented for other features
 * besides the main node, hardware versions, and firmware versions.
 */
class ComponentInterface {
 public:
  /**
   * @brief Get the ROS parameters.
   * @throws std::runtime_error if a parameter is invalid or required
   * parameters are not set.
   */
  virtual void getRosParams() = 0;

  /**
   * @brief Configure the U-Blox settings.
   * @return true if configured correctly, false otherwise
   */
  virtual bool configureUblox(std::shared_ptr<ublox_gps::Gps> gps) = 0;

  /**
   * @brief Initialize the diagnostics.
   *
   * @details Function may be empty.
   */
  virtual void initializeRosDiagnostics() = 0;

  /**
   * @brief Subscribe to u-blox messages and publish to ROS topics.
   */
  virtual void subscribe(std::shared_ptr<ublox_gps::Gps> gps) = 0;
};

}  // namespace ublox_node

#endif
