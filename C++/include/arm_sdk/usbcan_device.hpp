/**
 * ============================================================================
 * USBCANDevice — Pure C++ wrapper for libusbcan.so (no ROS dependency)
 * ============================================================================
 *
 * USBCAN-UC12 USB-to-CAN 어댑터용 dlopen 기반 래퍼.
 * VCI_OpenDevice, VCI_Transmit, VCI_Receive 등을 캡슐화.
 *
 * ============================================================================
 */

#ifndef ARM_SDK__USBCAN_DEVICE_HPP_
#define ARM_SDK__USBCAN_DEVICE_HPP_

#include <cstdint>
#include <string>

namespace arm_sdk
{

// Device types
constexpr uint32_t USBCAN_I  = 3;
constexpr uint32_t USBCAN_II = 4;

// CAN board info structure
struct CANBoardInfo {
    uint16_t hw_version;
    uint16_t fw_version;
    uint16_t dr_version;
    uint16_t in_version;
    uint16_t irq_num;
    uint8_t  can_num;
    uint8_t  serial_num[20];
    uint8_t  hw_type[40];
    uint8_t  reserved[4];
};

// CAN init config
struct CANInitConfig {
    int      acc_code;
    int      acc_mask;
    int      reserved;
    uint8_t  filter;
    uint8_t  timing0;
    uint8_t  timing1;
    uint8_t  mode;
};

// CAN frame (TX/RX)
struct CANFrame {
    uint32_t id;
    uint32_t timestamp;
    uint8_t  time_flag;
    int8_t   send_type;
    int8_t   remote_flag;
    int8_t   extern_flag;
    int8_t   data_len;
    uint8_t  data[8];
    uint8_t  reserved[3];
};

/// Configuration for USBCANDevice
struct USBCANConfig {
    uint32_t device_type   = USBCAN_II;
    uint32_t device_index  = 0;
    uint32_t can_channel   = 0;
    int      baudrate      = 1000000;  // CAN baudrate in Hz
    std::string lib_path   = "libusbcan.so";
};

class USBCANDevice
{
public:
    USBCANDevice();
    ~USBCANDevice();

    // Non-copyable
    USBCANDevice(const USBCANDevice&) = delete;
    USBCANDevice& operator=(const USBCANDevice&) = delete;

    /// Load libusbcan.so and resolve symbols
    bool loadLibrary(const std::string& lib_path = "libusbcan.so");

    /// Unload library
    void unloadLibrary();

    /// Open USBCAN device
    bool openDevice(uint32_t device_type, uint32_t device_index);

    /// Close USBCAN device
    void closeDevice();

    /// Initialize CAN channel with baudrate timing
    bool initChannel(uint32_t channel, uint16_t baudrate_timing);

    /// Start CAN channel
    bool startChannel(uint32_t channel);

    /// Reset CAN channel
    bool resetChannel(uint32_t channel);

    /// Send a CAN frame (extern_flag=1 for extended 29-bit ID)
    bool sendFrame(uint32_t channel, uint32_t id, const uint8_t* data, uint8_t len, uint8_t extern_flag = 0);

    /// Send multiple CAN frames in one USB call (returns number actually sent)
    int sendFrames(uint32_t channel, const CANFrame* frames, int count);

    /// Receive a CAN frame (returns false if no data within timeout)
    bool receiveFrame(uint32_t channel, uint32_t& id, uint8_t* data, uint8_t& len, int timeout_ms = 100, uint8_t* extern_flag = nullptr);

    /// Receive multiple CAN frames in one USB call (returns number received)
    int receiveFrames(uint32_t channel, CANFrame* frames, int max_count, int timeout_ms = 10);

    /// Get number of pending receive frames
    int getReceiveCount(uint32_t channel);

    /// Check if device is open
    bool isOpen() const { return device_opened_; }

    /// Check if library is loaded
    bool isLoaded() const { return lib_handle_ != nullptr; }

    /// Convert baudrate (Hz) to timing register value
    static uint16_t baudrateToTiming(int baudrate);

private:
    void* lib_handle_ = nullptr;
    bool  device_opened_ = false;
    uint32_t device_type_  = USBCAN_II;
    uint32_t device_index_ = 0;

    // Function pointers (loaded from libusbcan.so)
    using OpenDevice_t    = int (*)(uint32_t, uint32_t, uint32_t);
    using CloseDevice_t   = int (*)(uint32_t, uint32_t);
    using InitCAN_t       = int (*)(uint32_t, uint32_t, uint32_t, void*);
    using StartCAN_t      = int (*)(uint32_t, uint32_t, uint32_t);
    using ResetCAN_t      = int (*)(uint32_t, uint32_t, uint32_t);
    using Transmit_t      = int (*)(uint32_t, uint32_t, uint32_t, void*, uint32_t);
    using Receive_t       = int (*)(uint32_t, uint32_t, uint32_t, void*, uint32_t, int);
    using GetReceiveNum_t = int (*)(uint32_t, uint32_t, uint32_t);
    using ReadBoardInfo_t = int (*)(uint32_t, uint32_t, void*);

    OpenDevice_t    fn_open_    = nullptr;
    CloseDevice_t   fn_close_   = nullptr;
    InitCAN_t       fn_init_    = nullptr;
    StartCAN_t      fn_start_   = nullptr;
    ResetCAN_t      fn_reset_   = nullptr;
    Transmit_t      fn_transmit_= nullptr;
    Receive_t       fn_receive_ = nullptr;
    GetReceiveNum_t fn_recv_num_= nullptr;
    ReadBoardInfo_t fn_board_   = nullptr;
};

}  // namespace arm_sdk

#endif  // ARM_SDK__USBCAN_DEVICE_HPP_
