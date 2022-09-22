/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/buffered_udp_source.h"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "ouster/client.h"
#include "ouster/types.h"

/*
 * Locks are not held during the actual buffer reads and writes: thread safety
 * w.r.t the producer relies on the invariants that only the consumer modifies
 * the read index and only the producer modifies the write index, always while
 * holding cv_mtx_ to make sure that cv notifications are not lost between
 * checking the empty/full condition and entering the waiting state.
 */
namespace ouster {
namespace sensor {
namespace impl {

using fsec = std::chrono::duration<float>;

// 64 big enough for any UDP packet
constexpr size_t packet_size = 65536;

/*
 * Initialize the internal circular buffer.
 *
 * NOTE: capacity_ is the capacity of the internal buffer vector, not the
 * max number of buffered packets (which is one less).
 */
BufferedUDPSource::BufferedUDPSource(size_t buf_size)
    : capacity_{buf_size + 1} {
    std::generate_n(std::back_inserter(bufs_), capacity_, [&] {
        return std::make_pair(
            client_state::CLIENT_ERROR,
            std::make_unique<uint8_t[]>(packet_size));
    });
}

BufferedUDPSource::BufferedUDPSource(const std::string& hostname,
                                     int lidar_port, int imu_port,
                                     size_t buf_size)
    : BufferedUDPSource(buf_size) {
    cli_ = init_client(hostname, lidar_port, imu_port);
    if (!cli_) throw std::runtime_error("Failed to initialize client");
    lidar_port_ = sensor::get_lidar_port(*cli_);
    imu_port_ = sensor::get_imu_port(*cli_);
}

BufferedUDPSource::BufferedUDPSource(const std::string& hostname,
                                     const std::string& udp_dest_host,
                                     lidar_mode mode, timestamp_mode ts_mode,
                                     int lidar_port, int imu_port,
                                     int timeout_sec, size_t buf_size)
    : BufferedUDPSource(buf_size) {
    cli_ = init_client(hostname, udp_dest_host, mode, ts_mode, lidar_port,
                       imu_port, timeout_sec);
    if (!cli_) throw std::runtime_error("Failed to initialize client");
    lidar_port_ = sensor::get_lidar_port(*cli_);
    imu_port_ = sensor::get_imu_port(*cli_);
}

std::string BufferedUDPSource::get_metadata(int timeout_sec,
                                            bool legacy_format) {
    std::unique_lock<std::mutex> lock(cli_mtx_, std::try_to_lock);
    if (!lock.owns_lock())
        throw std::invalid_argument(
            "Another thread is already using the client");
    if (!cli_) throw std::invalid_argument("Client has already been shut down");
    return sensor::get_metadata(*cli_, timeout_sec, legacy_format);
}

/*
 * Invariant: nothing can access cli_ when stop_ is true. Producer will
 * release _cli_mtx_ only when it exits the loop.
 */
void BufferedUDPSource::shutdown() {
    {
        std::unique_lock<std::mutex> lock{cv_mtx_};
        if (stop_) return;
        stop_ = true;
    }
    cv_.notify_all();

    // close UDP sockets when any producer has exited
    std::lock_guard<std::mutex> cli_lock{cli_mtx_};
    cli_.reset();
}

/*
 * Advance the read index to drop data. Can only be called by the consumer to
 * maintain the invariant that only the reader modifies the read index.
 */
void BufferedUDPSource::flush(size_t n_packets) {
    {
        std::unique_lock<std::mutex> lock{cv_mtx_};
        auto sz = (capacity_ + write_ind_ - read_ind_) % capacity_;
        auto n = (n_packets == 0) ? sz : std::min(sz, n_packets);
        read_ind_ = (capacity_ + read_ind_ + n) % capacity_;
    }
    cv_.notify_one();
}

size_t BufferedUDPSource::size() {
    std::unique_lock<std::mutex> lock{cv_mtx_};
    return (capacity_ + write_ind_ - read_ind_) % capacity_;
}

size_t BufferedUDPSource::capacity() { return (capacity_ - 1); }

client_state BufferedUDPSource::consume(uint8_t* buf, size_t buf_sz,
                                        float timeout_sec) {
    // wait for producer to wake us up if the queue is empty
    {
        std::unique_lock<std::mutex> lock{cv_mtx_};
        bool timeout = !cv_.wait_for(lock, fsec{timeout_sec}, [this] {
            return stop_ || write_ind_ != read_ind_;
        });
        if (timeout)
            return client_state::TIMEOUT;
        else if (stop_)
            return client_state::EXIT;
    }

    // read data into buffer
    auto sz = std::min<size_t>(buf_sz, packet_size);
    auto& e = bufs_[read_ind_];
    std::memcpy(buf, e.second.get(), sz);

    // advance read ind and unblock producer, if necessary
    {
        std::unique_lock<std::mutex> lock{cv_mtx_};
        read_ind_ = (read_ind_ + 1) % capacity_;
    }
    cv_.notify_one();
    return e.first;
}

/*
 * Hold the client mutex to protect client state and prevent multiple
 * producers from running concurrently.
 */
void BufferedUDPSource::produce(const packet_format& pf) {
    std::lock_guard<std::mutex> cli_lock{cli_mtx_};

    auto exit_mask =
        client_state(client_state::CLIENT_ERROR | client_state::EXIT);
    auto st = client_state(0);

    while (!(st & exit_mask)) {
        // Wait for consumer to wake us up if the queue is full
        bool overflow = false;
        {
            std::unique_lock<std::mutex> lock{cv_mtx_};
            while (!stop_ && (write_ind_ + 1) % capacity_ == read_ind_) {
                overflow = true;
                cv_.wait(lock);
            }
            if (stop_) return;
        }

        // Write data and status to circular buffer. EXIT and ERROR status
        // are just passed through with stale data.
        st = poll_client(*cli_);
        if (st == client_state::TIMEOUT) continue;

        auto& e = bufs_[write_ind_];
        if (st & LIDAR_DATA) {
            if (!read_lidar_packet(*cli_, e.second.get(), pf)) continue;
        } else if (st & IMU_DATA) {
            if (!read_imu_packet(*cli_, e.second.get(), pf)) continue;
        }
        if (overflow) st = client_state(st | CLIENT_OVERFLOW);
        e.first = st;

        // Advance write ind and wake up consumer, if blocked
        {
            std::unique_lock<std::mutex> lock{cv_mtx_};
            write_ind_ = (write_ind_ + 1) % capacity_;
        }
        cv_.notify_one();
    }
}

int BufferedUDPSource::get_lidar_port() {
    std::lock_guard<std::mutex> lock{cv_mtx_};
    return stop_ ? 0 : lidar_port_;
}

int BufferedUDPSource::get_imu_port() {
    std::lock_guard<std::mutex> lock{cv_mtx_};
    return stop_ ? 0 : imu_port_;
}

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
