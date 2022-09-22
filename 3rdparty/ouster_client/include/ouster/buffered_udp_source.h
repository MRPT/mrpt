/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Wrapper around sensor::client to provide buffering
 *
 * *Not* a public API. Currently part of the Python bindings implementation.
 *
 * Maintains a single-producer / single-consumer circular buffer that can be
 * populated by a thread without holding the GIL to deal the relatively small
 * default OS buffer size and high sensor UDP data rate. Must be thread-safe to
 * allow reading data without holding the GIL while other references to the
 * client exist.
 */

#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "ouster/client.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {
namespace impl {

class BufferedUDPSource {
    // client handle
    std::mutex cli_mtx_;
    std::shared_ptr<ouster::sensor::client> cli_;
    uint32_t lidar_port_;
    uint32_t imu_port_;

    // protect read/write_ind_ and stop_
    std::mutex cv_mtx_;
    std::condition_variable cv_;
    size_t read_ind_{0}, write_ind_{0};

    // flag for other threads to signal producer to shut down
    bool stop_{false};

    // internal packet buffer
    size_t capacity_{0};
    using entry = std::pair<client_state, std::unique_ptr<uint8_t[]>>;
    std::vector<entry> bufs_;

    explicit BufferedUDPSource(size_t buf_size);

   public:
    /* Extra bit flag compatible with client_state to signal buffer overflow. */
    static constexpr int CLIENT_OVERFLOW = 0x10;

    /**
     * Listen for sensor data on the specified ports; do not configure the
     * sensor.
     *
     * @param[in] hostname hostname or IP of the sensor.
     * @param[in] lidar_port port on which the sensor will send lidar data.
     * @param[in] imu_port port on which the sensor will send imu data.
     * @param[in] buf_size size of internal buffer, in no. packets.
     */
    BufferedUDPSource(const std::string& hostname, int lidar_port, int imu_port,
                      size_t buf_size);

    /**
     * Connect to and configure the sensor and start listening for data.
     *
     * Will be removed.
     */
    [[deprecated]] BufferedUDPSource(const std::string& hostname,
                                     const std::string& udp_dest_host,
                                     lidar_mode mode, timestamp_mode ts_mode,
                                     int lidar_port, int imu_port,
                                     int timeout_sec, size_t buf_size);

    /**
     * Fetch metadata from the sensor.
     *
     * @param[in] timeout_sec maximum time to wait until sensor is initialized.
     * @param[in] legacy_format whether to use legacy format for metadata.
     * @return a json string of the sensor metadata.
     */
    std::string get_metadata(int timeout_sec = 60, bool legacy_format = true);

    /**
     * Signal the producer to exit.
     *
     * Subsequent calls to consume() will return
     * CLIENT_EXIT instead of blocking. Multiple calls to shutdown() are not an
     * error.
     */
    void shutdown();

    /**
     * Drop up to the specified number of packets from internal buffers.
     *
     * Drop all internally buffered data when n_packets = 0. Should only be
     * called by the consumer thread.
     *
     * @param n_packets number of packets to drop.
     */
    void flush(size_t n_packets);

    /**
     * Get current buffer size.
     *
     * @return number of packets currently buffered.
     */
    size_t size();

    /**
     * Get the maximum buffer size.
     *
     * @return maximum number of packets that can be buffered.
     */
    size_t capacity();

    /**
     * Read next available packet in the buffer.
     *
     * Blocks if the queue is empty for up to `timeout_sec` (zero means wait
     * forever). Should only be called by the consumer thread. If reading from
     * the network was blocked because the buffer was full, the the
     * CLIENT_OVERFLOW flag will be set on the next returned status.
     *
     * @param[in] buf the buffer to read into.
     * @param[in] buf_sz maximum number of bytes to read into the buffer.
     * @param[in] timeout_sec maximum time to wait for data.
     * @return client status, see sensor::poll_client().
     */
    client_state consume(uint8_t* buf, size_t buf_sz, float timeout_sec);

    /**
     * Write data from the network into the circular buffer.
     *
     * Returns when shutdown() is signaled by the reader. Should be called from
     * a separate thread from the consumer.
     *
     * @param[in] pf the packet format associated with the UDP stream.
     */
    void produce(const ouster::sensor::packet_format& pf);

    /**
     * Return the port used to listen for lidar UDP data.
     *
     * @return the lidar UDP port or 0 if shut down.
     */
    int get_lidar_port();

    /**
     * Return the port used to listen for imu UDP data.
     *
     * @return the lidar UDP port or 0 if shut down.
     */
    int get_imu_port();
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
