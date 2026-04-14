/**
 * ============================================================================
 * USBCANDevice — Implementation
 * ============================================================================
 */

#include "arm_sdk/usbcan_device.hpp"

#include <cstdio>
#include <cstring>
#include <dlfcn.h>

namespace arm_sdk
{

// Internal struct matching libusbcan.so's ZCAN_CAN_INIT_CONFIG layout
struct ZcanInitConfig {
    int      AccCode;
    int      AccMask;
    int      Reserved;
    uint8_t  Filter;
    uint8_t  Timing0;
    uint8_t  Timing1;
    uint8_t  Mode;
};

// Internal struct matching libusbcan.so's ZCAN_CAN_OBJ layout
struct ZcanObj {
    uint32_t ID;
    uint32_t TimeStamp;
    uint8_t  TimeFlag;
    int8_t   SendType;
    int8_t   RemoteFlag;
    int8_t   ExternFlag;
    int8_t   DataLen;
    uint8_t  Data[8];
    uint8_t  Reserved[3];
};

USBCANDevice::USBCANDevice() = default;

USBCANDevice::~USBCANDevice()
{
    closeDevice();
    unloadLibrary();
}

uint16_t USBCANDevice::baudrateToTiming(int baudrate)
{
    switch (baudrate)
    {
        case 1000000: return 0x1400;
        case 500000:  return 0x1c00;
        case 250000:  return 0x1c01;
        case 125000:  return 0x1c03;
        default:      return 0x1400;
    }
}

bool USBCANDevice::loadLibrary(const std::string& lib_path)
{
    unloadLibrary();

    lib_handle_ = dlopen(lib_path.c_str(), RTLD_NOW);
    if (!lib_handle_)
    {
        fprintf(stderr, "[USBCANDevice] Failed to load %s: %s\n",
                lib_path.c_str(), dlerror());
        fprintf(stderr, "[USBCANDevice] Install: sudo cp libusbcan.so /lib/ && sudo ldconfig\n");
        return false;
    }

    fn_open_     = (OpenDevice_t)   dlsym(lib_handle_, "VCI_OpenDevice");
    fn_close_    = (CloseDevice_t)  dlsym(lib_handle_, "VCI_CloseDevice");
    fn_init_     = (InitCAN_t)      dlsym(lib_handle_, "VCI_InitCAN");
    fn_start_    = (StartCAN_t)     dlsym(lib_handle_, "VCI_StartCAN");
    fn_reset_    = (ResetCAN_t)     dlsym(lib_handle_, "VCI_ResetCAN");
    fn_transmit_ = (Transmit_t)    dlsym(lib_handle_, "VCI_Transmit");
    fn_receive_  = (Receive_t)     dlsym(lib_handle_, "VCI_Receive");
    fn_recv_num_ = (GetReceiveNum_t)dlsym(lib_handle_, "VCI_GetReceiveNum");
    fn_board_    = (ReadBoardInfo_t)dlsym(lib_handle_, "VCI_ReadBoardInfo");

    if (!fn_open_ || !fn_close_ || !fn_init_ ||
        !fn_start_ || !fn_transmit_ || !fn_receive_)
    {
        fprintf(stderr, "[USBCANDevice] Failed to resolve required symbols from %s\n",
                lib_path.c_str());
        dlclose(lib_handle_);
        lib_handle_ = nullptr;
        return false;
    }

    printf("[USBCANDevice] Library loaded: %s\n", lib_path.c_str());
    return true;
}

void USBCANDevice::unloadLibrary()
{
    if (lib_handle_)
    {
        dlclose(lib_handle_);
        lib_handle_ = nullptr;
    }
    fn_open_ = nullptr;
    fn_close_ = nullptr;
    fn_init_ = nullptr;
    fn_start_ = nullptr;
    fn_reset_ = nullptr;
    fn_transmit_ = nullptr;
    fn_receive_ = nullptr;
    fn_recv_num_ = nullptr;
    fn_board_ = nullptr;
}

bool USBCANDevice::openDevice(uint32_t device_type, uint32_t device_index)
{
    if (!fn_open_) return false;

    int ret = fn_open_(device_type, device_index, 0);
    if (ret != 1)
    {
        fprintf(stderr, "[USBCANDevice] Failed to open device (type=%u, index=%u)\n",
                device_type, device_index);
        fprintf(stderr, "[USBCANDevice] Check: lsusb | grep 0471, udev rules, permissions\n");
        return false;
    }

    device_type_  = device_type;
    device_index_ = device_index;
    device_opened_ = true;
    printf("[USBCANDevice] Device opened (type=%u, index=%u)\n", device_type, device_index);
    return true;
}

void USBCANDevice::closeDevice()
{
    if (device_opened_ && fn_close_)
    {
        if (fn_reset_) fn_reset_(device_type_, device_index_, 0);
        fn_close_(device_type_, device_index_);
        device_opened_ = false;
        printf("[USBCANDevice] Device closed\n");
    }
}

bool USBCANDevice::initChannel(uint32_t channel, uint16_t baudrate_timing)
{
    if (!fn_init_) return false;

    ZcanInitConfig config;
    config.AccCode  = 0;
    config.AccMask  = 0xFFFFFFFF;
    config.Reserved = 0;
    config.Filter   = 1;
    config.Timing0  = baudrate_timing & 0xFF;
    config.Timing1  = (baudrate_timing >> 8) & 0xFF;
    config.Mode     = 0;  // Normal mode

    int ret = fn_init_(device_type_, device_index_, channel, &config);
    if (ret != 1)
    {
        fprintf(stderr, "[USBCANDevice] Failed to init CAN channel %u\n", channel);
        return false;
    }

    printf("[USBCANDevice] CAN channel %u initialized (timing=0x%04X)\n",
           channel, baudrate_timing);
    return true;
}

bool USBCANDevice::startChannel(uint32_t channel)
{
    if (!fn_start_) return false;

    int ret = fn_start_(device_type_, device_index_, channel);
    if (ret != 1)
    {
        fprintf(stderr, "[USBCANDevice] Failed to start CAN channel %u\n", channel);
        return false;
    }

    printf("[USBCANDevice] CAN channel %u started\n", channel);
    return true;
}

bool USBCANDevice::resetChannel(uint32_t channel)
{
    if (!fn_reset_) return false;
    fn_reset_(device_type_, device_index_, channel);
    return true;
}

bool USBCANDevice::sendFrame(uint32_t channel, uint32_t id,
                              const uint8_t* data, uint8_t len, uint8_t extern_flag)
{
    if (!fn_transmit_) return false;

    ZcanObj msg;
    memset(&msg, 0, sizeof(msg));
    msg.ID = id;
    msg.SendType = 0;
    msg.RemoteFlag = 0;
    msg.ExternFlag = extern_flag;
    msg.DataLen = len;
    memcpy(msg.Data, data, len);

    int ret = fn_transmit_(device_type_, device_index_, channel, &msg, 1);
    return ret == 1;
}

int USBCANDevice::sendFrames(uint32_t channel, const CANFrame* frames, int count)
{
    if (!fn_transmit_ || count <= 0) return 0;

    ZcanObj objs[32];
    int n = (count > 32) ? 32 : count;
    for (int i = 0; i < n; ++i) {
        memset(&objs[i], 0, sizeof(ZcanObj));
        objs[i].ID = frames[i].id;
        objs[i].SendType = 0;
        objs[i].RemoteFlag = 0;
        objs[i].ExternFlag = frames[i].extern_flag;
        objs[i].DataLen = frames[i].data_len;
        memcpy(objs[i].Data, frames[i].data, frames[i].data_len);
    }
    return fn_transmit_(device_type_, device_index_, channel, objs, n);
}

bool USBCANDevice::receiveFrame(uint32_t channel, uint32_t& id,
                                 uint8_t* data, uint8_t& len, int timeout_ms, uint8_t* extern_flag)
{
    if (!fn_receive_) return false;

    ZcanObj msg;
    int ret = fn_receive_(device_type_, device_index_, channel, &msg, 1, timeout_ms);
    if (ret <= 0) return false;

    id  = msg.ID;
    len = msg.DataLen;
    memcpy(data, msg.Data, len);
    if (extern_flag) *extern_flag = msg.ExternFlag;
    return true;
}

int USBCANDevice::receiveFrames(uint32_t channel, CANFrame* frames, int max_count, int timeout_ms)
{
    if (!fn_receive_ || max_count <= 0) return 0;

    ZcanObj objs[64];
    int n = (max_count > 64) ? 64 : max_count;
    int ret = fn_receive_(device_type_, device_index_, channel, objs, n, timeout_ms);
    if (ret <= 0) return 0;

    for (int i = 0; i < ret; ++i) {
        frames[i].id = objs[i].ID;
        frames[i].timestamp = objs[i].TimeStamp;
        frames[i].time_flag = objs[i].TimeFlag;
        frames[i].send_type = objs[i].SendType;
        frames[i].remote_flag = objs[i].RemoteFlag;
        frames[i].extern_flag = objs[i].ExternFlag;
        frames[i].data_len = objs[i].DataLen;
        memcpy(frames[i].data, objs[i].Data, 8);
    }
    return ret;
}

int USBCANDevice::getReceiveCount(uint32_t channel)
{
    if (!fn_recv_num_) return 0;
    return fn_recv_num_(device_type_, device_index_, channel);
}

}  // namespace arm_sdk
