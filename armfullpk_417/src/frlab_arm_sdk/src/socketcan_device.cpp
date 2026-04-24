/**
 * ============================================================================
 * SocketCANDevice — Implementation
 * ============================================================================
 */

#include "arm_sdk/socketcan_device.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <dirent.h>

namespace arm_sdk
{

SocketCANDevice::SocketCANDevice() = default;

SocketCANDevice::~SocketCANDevice()
{
    close();
}

bool SocketCANDevice::open(const std::string& iface)
{
    // Create raw CAN socket
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        perror("[SocketCANDevice] socket()");
        fd_ = -1;
        return false;
    }

    // Resolve interface index
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);

    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "[SocketCANDevice] interface '%s' not found: %s\n",
                iface.c_str(), strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Bind to interface — accept both standard and extended frames
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[SocketCANDevice] bind()");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    setUsbLatency(iface);
    return true;
}

// slcan 인터페이스(can0 등) → 대응 ttyACMx 찾아 USB latency_timer를 1ms로 설정
void SocketCANDevice::setUsbLatency(const std::string& iface)
{
    // /sys/class/net/<iface>/device/tty/ 아래에 ttyACMx 심볼릭 링크가 있음
    std::string tty_dir = "/sys/class/net/" + iface + "/device/tty";
    DIR* dir = opendir(tty_dir.c_str());
    if (!dir) return;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name(entry->d_name);
        if (name == "." || name == "..") continue;

        std::string path = "/sys/bus/usb-serial/devices/" + name + "/latency_timer";
        FILE* f = fopen(path.c_str(), "w");
        if (f) {
            fprintf(f, "1\n");
            fclose(f);
            fprintf(stderr, "[SocketCANDevice] USB latency_timer set to 1ms for %s\n", name.c_str());
        }
        break;
    }
    closedir(dir);
}

void SocketCANDevice::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SocketCANDevice::sendFrame(uint32_t id, const uint8_t* data, uint8_t len, bool extended)
{
    if (fd_ < 0) return false;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id  = extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = (len > 8) ? 8 : len;
    std::memcpy(frame.data, data, frame.can_dlc);

    ssize_t n = write(fd_, &frame, sizeof(frame));
    return n == static_cast<ssize_t>(sizeof(frame));
}

int SocketCANDevice::sendFrames(const CANFrame* frames, int count)
{
    int sent = 0;
    for (int i = 0; i < count; ++i) {
        if (sendFrame(frames[i].id, frames[i].data, frames[i].data_len, frames[i].extended))
            sent++;
    }
    return sent;
}

bool SocketCANDevice::receiveFrame(uint32_t& id, uint8_t* data, uint8_t& len,
                                   int timeout_ms, bool* extended)
{
    if (fd_ < 0) return false;

    if (timeout_ms > 0) {
        // Wait with select()
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(fd_, &rset);

        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(fd_ + 1, &rset, nullptr, nullptr, &tv);
        if (ret <= 0) return false;  // timeout or error
    } else {
        // Non-blocking check
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(fd_, &rset);

        struct timeval tv = {0, 0};
        int ret = select(fd_ + 1, &rset, nullptr, nullptr, &tv);
        if (ret <= 0) return false;
    }

    struct can_frame frame;
    ssize_t n = read(fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) return false;

    // Strip flags to get raw ID
    bool is_ext = (frame.can_id & CAN_EFF_FLAG) != 0;
    id  = is_ext ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
    len = frame.can_dlc;
    std::memcpy(data, frame.data, frame.can_dlc);

    if (extended) *extended = is_ext;
    return true;
}

int SocketCANDevice::receiveFrames(CANFrame* frames, int max_count, int timeout_ms)
{
    if (fd_ < 0 || max_count <= 0) return 0;

    int count = 0;

    // First frame: use caller's timeout
    {
        uint32_t id;
        uint8_t  data[8], len;
        bool     ext = false;
        if (!receiveFrame(id, data, len, timeout_ms, &ext))
            return 0;

        frames[count].id       = id;
        frames[count].data_len = len;
        frames[count].extended = ext;
        std::memcpy(frames[count].data, data, len);
        count++;
    }

    // Subsequent frames: non-blocking
    while (count < max_count) {
        uint32_t id;
        uint8_t  data[8], len;
        bool     ext = false;
        if (!receiveFrame(id, data, len, 0, &ext))
            break;

        frames[count].id       = id;
        frames[count].data_len = len;
        frames[count].extended = ext;
        std::memcpy(frames[count].data, data, len);
        count++;
    }

    return count;
}

}  // namespace arm_sdk
