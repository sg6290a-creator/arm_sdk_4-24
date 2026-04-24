/**
 * ============================================================================
 * SocketCANDevice — Linux SocketCAN wrapper (CANable v2.0 / slcan)
 * ============================================================================
 *
 * 사전 준비:
 *   sudo slcand -o -c -s8 /dev/ttyACM0 can0   # 1Mbps
 *   sudo ip link set up can0
 *
 * 또는 native CAN 어댑터:
 *   sudo ip link set can0 type can bitrate 1000000
 *   sudo ip link set up can0
 *
 * ============================================================================
 */

#ifndef ARM_SDK__SOCKETCAN_DEVICE_HPP_
#define ARM_SDK__SOCKETCAN_DEVICE_HPP_

#include <cstdint>
#include <string>

namespace arm_sdk
{

/// Unified CAN frame for TX/RX
struct CANFrame {
    uint32_t id;           ///< CAN ID (29-bit extended or 11-bit standard)
    uint8_t  data_len;     ///< Data length (0-8)
    uint8_t  data[8];      ///< Payload
    bool     extended;     ///< true = 29-bit extended ID (Robstride), false = 11-bit standard (RMD)
};

class SocketCANDevice
{
public:
    SocketCANDevice();
    ~SocketCANDevice();

    // Non-copyable
    SocketCANDevice(const SocketCANDevice&) = delete;
    SocketCANDevice& operator=(const SocketCANDevice&) = delete;

    /// Open SocketCAN interface (e.g. "can0")
    bool open(const std::string& iface);

    /// Close socket
    void close();

    /// Check if socket is open
    bool isOpen() const { return fd_ >= 0; }

    /// Send a single CAN frame
    /// @param extended  true = 29-bit extended ID (Robstride), false = 11-bit standard (RMD)
    bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len, bool extended = false);

    /// Send multiple frames (returns number actually sent)
    int sendFrames(const CANFrame* frames, int count);

    /// Receive a CAN frame with timeout
    /// @param timeout_ms  0 = non-blocking, >0 = wait up to timeout_ms
    bool receiveFrame(uint32_t& id, uint8_t* data, uint8_t& len,
                      int timeout_ms = 100, bool* extended = nullptr);

    /// Receive multiple frames (returns number received)
    /// timeout_ms applies to the first frame only; subsequent reads are non-blocking
    int receiveFrames(CANFrame* frames, int max_count, int timeout_ms = 10);

private:
    int fd_ = -1;
    void setUsbLatency(const std::string& iface);
};

}  // namespace arm_sdk

#endif  // ARM_SDK__SOCKETCAN_DEVICE_HPP_
