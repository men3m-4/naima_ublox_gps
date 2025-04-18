

#ifndef UBLOX_GPS_CALLBACK_HPP
#define UBLOX_GPS_CALLBACK_HPP

#include <chrono>
#include <condition_variable>
#include <functional>
#include <ios>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <ublox_msgs/serialization.hpp>

namespace ublox_gps {

/**
 * @brief A callback handler for a u-blox message.
 */
class CallbackHandler {
 public:
  /**
   * @brief Decode the u-blox message.
   */
  virtual void handle(ublox::Reader& reader) = 0;

  /**
   * @brief Wait for on the condition.
   */
  bool wait(const std::chrono::milliseconds& timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout) == std::cv_status::no_timeout;
  }

 protected:
  std::mutex mutex_; //!< Lock for the handler
  std::condition_variable condition_; //!< Condition for the handler lock
};

/**
 * @brief A callback handler for a u-blox message.
 * @typedef T the message type
 */
template <typename T>
class CallbackHandler_ final : public CallbackHandler {
 public:
  using Callback = std::function<void(const T&)>;

  /**
   * @brief Initialize the Callback Handler with a callback function
   * @param func a callback function for the message, defaults to none
   */
  explicit CallbackHandler_(const Callback& func = Callback(), int debug = 1) : func_(func), debug_(debug) {}

  /**
   * @brief Get the last received message.
   */
  virtual const T& get() { return message_; }

  /**
   * @brief Decode the U-Blox message & call the callback function if it exists.
   * @param reader a reader to decode the message buffer
   */
  void handle(ublox::Reader& reader) override {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (!reader.read<T>(message_)) {
        // RCLCPP_DEBUG_COND(debug_ >= 2,
        //                "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)",
        //                static_cast<unsigned int>(reader.classId()),
        //                static_cast<unsigned int>(reader.messageId()),
        //                reader.length());
        condition_.notify_all();
        return;
      }
    } catch (const std::runtime_error& e) {
      // RCLCPP_DEBUG_COND(debug_ >= 2,
      //                "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)",
      //                static_cast<unsigned int>(reader.classId()),
      //                static_cast<unsigned int>(reader.messageId()),
      //                reader.length());
      condition_.notify_all();
      return;
    }

    if (func_) {
      func_(message_);
    }
    condition_.notify_all();
  }

 private:
  Callback func_; //!< the callback function to handle the message
  T message_; //!< The last received message
  int debug_;
};

/**
 * @brief Callback handlers for incoming u-blox messages.
 */
class CallbackHandlers final {
 public:
  explicit CallbackHandlers(int debug) : debug_(debug) {}

  /**
   * @brief Add a callback handler for the given message type.
   * @param callback the callback handler for the message
   * @typedef.a ublox_msgs message with CLASS_ID and MESSAGE_ID constants
   */
  template <typename T>
  void insert(typename CallbackHandler_<T>::Callback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                     std::make_shared<CallbackHandler_<T>>(callback, debug_)));
  }

  /**
   * @brief Add a callback handler for the given message type and ID. This is
   * used for messages in which have the same structure (and therefore msg file)
   * and same class ID but different message IDs. (e.g. INF, ACK)
   * @param callback the callback handler for the message
   * @param message_id the ID of the message
   * @typedef.a ublox_msgs message with a CLASS_ID constant
   */
  template <typename T>
  void insert(
      typename CallbackHandler_<T>::Callback callback,
      unsigned int message_id) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, message_id),
                     std::make_shared<CallbackHandler_<T>>(callback, debug_)));
  }

  /**
   * @brief Calls the callback handler for the message in the reader.
   * @param reader a reader containing a u-blox message
   */
  void handle(ublox::Reader& reader) {
    // Find the callback handlers for the message & decode it
    std::lock_guard<std::mutex> lock(callback_mutex_);
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback) {
      callback->second->handle(reader);
    }
  }

  /**
   * @brief Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename T>
  bool read(T& message, const std::chrono::milliseconds& timeout) {
    bool result = false;
    // Create a callback handler for this message
    callback_mutex_.lock();
    auto handler = std::make_shared<CallbackHandler_<T>>(typename CallbackHandler_<T>::Callback(), debug_);
    Callbacks::iterator callback = callbacks_.insert(
      (std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                      handler)));
    callback_mutex_.unlock();

    // Wait for the message
    if (handler->wait(timeout)) {
      message = handler->get();
      result = true;
    }

    // Remove the callback handler
    callback_mutex_.lock();
    callbacks_.erase(callback);
    callback_mutex_.unlock();
    return result;
  }

  /**
   * @brief Processes u-blox messages in the given buffer & clears the read
   * messages from the buffer.
   * @param data the buffer of u-blox messages to process
   * @param size the size of the buffer
   * @return the number of bytes consumed
   */
  size_t readCallback(unsigned char* data, std::size_t size) {
    ublox::Reader reader(data, size);
    // Read all U-Blox messages in buffer
    while (reader.search() != reader.end() && reader.found()) {
      if (debug_ >= 3) {
        // Print the received bytes
        std::ostringstream oss;
        for (ublox::Reader::iterator it = reader.pos();
             it != reader.pos() + reader.length() + 8; ++it) {
          oss << std::hex << static_cast<unsigned int>(*it) << " ";
        }
        // RCLCPP_DEBUG("U-blox: reading %d bytes\n%s", reader.length() + 8,
        //           oss.str().c_str());
      }

      handle(reader);
    }

    // delete read bytes from ASIO input buffer
    std::copy(reader.pos(), reader.end(), data);

    return reader.pos() - data;
  }

 private:
  using Callbacks = std::multimap<std::pair<uint8_t, uint8_t>,
                                  std::shared_ptr<CallbackHandler>>;

  // Call back handlers for u-blox messages
  Callbacks callbacks_;
  std::mutex callback_mutex_;
  int debug_;
};

}  // namespace ublox_gps

#endif  // UBLOX_GPS_CALLBACK_HPP
