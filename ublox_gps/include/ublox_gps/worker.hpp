

#ifndef UBLOX_GPS_WORKER_HPP
#define UBLOX_GPS_WORKER_HPP

#include <chrono>
#include <functional>

namespace ublox_gps {

/**
 * @brief Handles I/O reading and writing.
 */
// clang-tidy insists that we follow rule-of-5 for this abstract base class.
// Generally we'd define the copy and move constructors as delete, but once
// we do that we also have to explicitly define a default constructor (otherwise
// it fails to compile).  That all works, but harms the readability of this
// completely abstract base class (basically an interface), so we just disable
// the checks for this class.
class Worker {  // NOLINT(hicpp-special-member-functions, cppcoreguidelines-special-member-functions)
 public:
  using WorkerCallback = std::function<size_t(unsigned char*, std::size_t)>;
  using WorkerRawCallback = std::function<void(unsigned char*, std::size_t)>;

  virtual ~Worker() = default;

  /**
   * @brief Set the callback function for received messages.
   * @param callback the callback function which process messages in the buffer
   */
  virtual void setCallback(const WorkerCallback& callback) = 0;

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  virtual void setRawDataCallback(const WorkerRawCallback& callback) = 0;

  /**
   * @brief Send the data in the buffer.
   * @param data the bytes to send
   * @param size the size of the buffer
   */
  virtual bool send(const unsigned char* data, const unsigned int size) = 0;

  /**
   * @brief Wait for an incoming message.
   * @param timeout the maximum time to wait.
   */
  virtual void wait(const std::chrono::milliseconds& timeout) = 0;

  /**
   * @brief Whether or not the I/O stream is open.
   */
  virtual bool isOpen() const = 0;
};

}  // namespace ublox_gps

#endif  // UBLOX_GPS_WORKER_HPP
