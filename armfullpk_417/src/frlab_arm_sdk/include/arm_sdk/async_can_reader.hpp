/**
 * ============================================================================
 * AsyncCANReader — lightweight background SocketCAN RX loop
 * ============================================================================
 *
 * Purpose:
 *   Keep draining the CAN RX queue on a background thread and hand frames to a
 *   caller-provided callback. This lets the realtime control loop read cached
 *   joint state instead of waiting synchronously for every motor response.
 *
 * Notes:
 *   - The reader uses a shared I/O mutex with the driver. Synchronous
 *     request/response operations can hold that mutex, and the background
 *     thread will back off briefly instead of stealing those responses.
 *   - Frames are received in non-blocking mode and the thread idles for a
 *     short sleep when the RX queue is empty.
 *
 * ============================================================================
 */

#ifndef ARM_SDK__ASYNC_CAN_READER_HPP_
#define ARM_SDK__ASYNC_CAN_READER_HPP_

#include "arm_sdk/socketcan_device.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace arm_sdk
{

class AsyncCANReader
{
public:
    using FrameHandler = std::function<void(uint32_t id, const uint8_t* data,
                                            uint8_t len, bool extended)>;

    AsyncCANReader() = default;
    ~AsyncCANReader() { stop(); }

    AsyncCANReader(const AsyncCANReader&) = delete;
    AsyncCANReader& operator=(const AsyncCANReader&) = delete;

    void setFrameHandler(FrameHandler handler)
    {
        handler_ = std::move(handler);
    }

    bool start(SocketCANDevice* can_device, std::mutex* io_mutex, int idle_sleep_us = 250)
    {
        stop();
        if (!can_device || !io_mutex || !handler_) {
            return false;
        }

        can_device_ = can_device;
        io_mutex_ = io_mutex;
        idle_sleep_us_ = (idle_sleep_us < 50) ? 50 : idle_sleep_us;
        running_.store(true);
        worker_ = std::thread(&AsyncCANReader::run, this);
        return true;
    }

    void stop()
    {
        running_.store(false);
        if (worker_.joinable()) {
            worker_.join();
        }
        can_device_ = nullptr;
        io_mutex_ = nullptr;
    }

    bool isRunning() const
    {
        return running_.load();
    }

private:
    void run()
    {
        while (running_.load()) {
            SocketCANDevice* can_device = can_device_;
            std::mutex* io_mutex = io_mutex_;
            if (!can_device || !io_mutex || !handler_) {
                std::this_thread::sleep_for(std::chrono::microseconds(idle_sleep_us_));
                continue;
            }

            uint32_t id = 0;
            uint8_t data[8] = {0};
            uint8_t len = 0;
            bool extended = false;
            bool got_frame = false;

            {
                std::unique_lock<std::mutex> io_lock(*io_mutex, std::try_to_lock);
                if (io_lock.owns_lock() && can_device->isOpen()) {
                    got_frame = can_device->receiveFrame(id, data, len, 0, &extended);
                }
            }

            if (got_frame) {
                handler_(id, data, len, extended);
                continue;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(idle_sleep_us_));
        }
    }

    std::atomic<bool> running_{false};
    SocketCANDevice* can_device_ = nullptr;
    std::mutex* io_mutex_ = nullptr;
    int idle_sleep_us_ = 250;
    FrameHandler handler_;
    std::thread worker_;
};

}  // namespace arm_sdk

#endif  // ARM_SDK__ASYNC_CAN_READER_HPP_
