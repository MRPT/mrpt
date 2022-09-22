/**
 * Copyright(c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/client.h"

#include <json/json.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "netcompat.h"
#include "ouster/types.h"
#include "sensor_http.h"

using namespace std::chrono_literals;
namespace chrono = std::chrono;
using ouster::sensor::util::SensorHttp;

namespace ouster {
namespace sensor {

struct client {
    SOCKET lidar_fd;
    SOCKET imu_fd;
    std::string hostname;
    Json::Value meta;
    ~client() {
        impl::socket_close(lidar_fd);
        impl::socket_close(imu_fd);
    }
};

// defined in types.cpp
Json::Value to_json(const sensor_config& config);

namespace {

// default udp receive buffer size on windows is very low -- use 256K
const int RCVBUF_SIZE = 256 * 1024;

int32_t get_sock_port(SOCKET sock_fd) {
    struct sockaddr_storage ss;
    socklen_t addrlen = sizeof ss;

    if (!impl::socket_valid(
            getsockname(sock_fd, (struct sockaddr*)&ss, &addrlen))) {
        std::cerr << "udp getsockname(): " << impl::socket_get_error()
                  << std::endl;
        return SOCKET_ERROR;
    }

    if (ss.ss_family == AF_INET)
        return ntohs(((struct sockaddr_in*)&ss)->sin_port);
    else if (ss.ss_family == AF_INET6)
        return ntohs(((struct sockaddr_in6*)&ss)->sin6_port);
    else
        return SOCKET_ERROR;
}

SOCKET udp_data_socket(int port) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    auto port_s = std::to_string(port);

    int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
    if (ret != 0) {
        std::cerr << "udp getaddrinfo(): " << gai_strerror(ret) << std::endl;
        return SOCKET_ERROR;
    }
    if (info_start == NULL) {
        std::cerr << "udp getaddrinfo(): empty result" << std::endl;
        return SOCKET_ERROR;
    }

    // try to bind a dual-stack ipv6 socket, but fall back to ipv4 only if that
    // fails (when ipv6 is disabled via kernel parameters). Use two passes to
    // deal with glibc addrinfo ordering:
    // https://sourceware.org/bugzilla/show_bug.cgi?id=9981
    for (auto preferred_af : {AF_INET6, AF_INET}) {
        for (ai = info_start; ai != NULL; ai = ai->ai_next) {
            if (ai->ai_family != preferred_af) continue;

            // choose first addrinfo where bind() succeeds
            SOCKET sock_fd =
                socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (!impl::socket_valid(sock_fd)) {
                std::cerr << "udp socket(): " << impl::socket_get_error()
                          << std::endl;
                continue;
            }

            int off = 0;
            if (ai->ai_family == AF_INET6 &&
                setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&off,
                           sizeof(off))) {
                std::cerr << "udp setsockopt(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            if (impl::socket_set_reuse(sock_fd)) {
                std::cerr << "udp socket_set_reuse(): "
                          << impl::socket_get_error() << std::endl;
            }

            if (::bind(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen)) {
                std::cerr << "udp bind(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            // bind() succeeded; set some options and return
            if (impl::socket_set_non_blocking(sock_fd)) {
                std::cerr << "udp fcntl(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, (char*)&RCVBUF_SIZE,
                           sizeof(RCVBUF_SIZE))) {
                std::cerr << "udp setsockopt(): " << impl::socket_get_error()
                          << std::endl;
                impl::socket_close(sock_fd);
                continue;
            }

            freeaddrinfo(info_start);
            return sock_fd;
        }
    }

    // could not bind() a UDP server socket
    freeaddrinfo(info_start);
    return SOCKET_ERROR;
}

bool collect_metadata(client& cli, SensorHttp& sensor_http,
                      chrono::seconds timeout) {
    auto timeout_time = chrono::steady_clock::now() + timeout;
    std::string status;

    do {
        if (chrono::steady_clock::now() >= timeout_time) return false;
        std::this_thread::sleep_for(1s);
        status = sensor_http.sensor_info()["status"].asString();
    } while (status == "INITIALIZING");

    // not all metadata available when sensor isn't RUNNING
    if (status != "RUNNING") {
        throw std::runtime_error(
            "Cannot obtain full metadata with sensor status: " + status +
            ". Please ensure that sensor is not in a STANDBY, UNCONFIGURED, "
            "WARMUP, or ERROR state");
    }

    cli.meta = sensor_http.metadata();

    // merge extra info into metadata
    cli.meta["client_version"] = client_version();

    return true;
}

}  // namespace

bool get_config(const std::string& hostname, sensor_config& config,
                bool active) {
    auto sensor_http = SensorHttp::create(hostname);
    auto res = sensor_http->get_config_params(active);
    config = parse_config(res);
    return true;
}

bool set_config(const std::string& hostname, const sensor_config& config,
                uint8_t config_flags) {
    auto sensor_http = SensorHttp::create(hostname);

    // reset staged config to avoid spurious errors
    auto active_params = sensor_http->get_config_params(true);

    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root;
    auto parse_success = reader->parse(
        active_params.c_str(), active_params.c_str() + active_params.size(),
        &root, nullptr);

    if (!parse_success)
        throw std::runtime_error("Error while parsing current sensor config.");

    // set all desired config parameters
    Json::Value config_json = to_json(config);
    for (const auto& key : config_json.getMemberNames()) {
        root[key] = config_json[key];
    }

    active_params = Json::FastWriter().write(root);
    sensor_http->set_config_param(".", active_params);

    // set automatic udp dest, if flag specified
    if (config_flags & CONFIG_UDP_DEST_AUTO) {
        if (config.udp_dest)
            throw std::invalid_argument(
                "UDP_DEST_AUTO flag set but provided config has udp_dest");
        sensor_http->set_udp_dest_auto();
    }

    // reinitialize to make all staged parameters effective
    sensor_http->reinitialize();
    // save if indicated
    if (config_flags & CONFIG_PERSIST) {
        sensor_http->save_config_params();
    }

    return true;
}

std::string get_metadata(client& cli, int timeout_sec, bool legacy_format) {
    if (!cli.meta) {
        auto sensor_http = SensorHttp::create(cli.hostname);
        if (!collect_metadata(cli, *sensor_http, chrono::seconds{timeout_sec}))
            return "";
    }

    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";
    auto metadata_string = Json::writeString(builder, cli.meta);
    return legacy_format ? convert_to_legacy(metadata_string) : metadata_string;
}

std::shared_ptr<client> init_client(const std::string& hostname, int lidar_port,
                                    int imu_port) {
    auto cli = std::make_shared<client>();
    cli->hostname = hostname;
    cli->lidar_fd = udp_data_socket(lidar_port);
    cli->imu_fd = udp_data_socket(imu_port);

    if (!impl::socket_valid(cli->lidar_fd) || !impl::socket_valid(cli->imu_fd))
        return std::shared_ptr<client>();

    return cli;
}

std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode, timestamp_mode ts_mode,
                                    int lidar_port, int imu_port,
                                    int timeout_sec) {
    auto cli = init_client(hostname, lidar_port, imu_port);
    if (!cli) return std::shared_ptr<client>();

    // update requested ports to actual bound ports
    lidar_port = get_sock_port(cli->lidar_fd);
    imu_port = get_sock_port(cli->imu_fd);
    if (!impl::socket_valid(lidar_port) || !impl::socket_valid(imu_port))
        return std::shared_ptr<client>();

    bool success = true;

    try {
        // fail fast if we can't reach the sensor via HTTP
        auto sensor_http = SensorHttp::create(hostname);

        // if dest address is not specified, have the sensor to set it
        // automatically
        if (udp_dest_host.empty()) {
            sensor_http->set_udp_dest_auto();
        } else {
            sensor_http->set_config_param("udp_dest", udp_dest_host);
        }

        sensor_http->set_config_param("udp_port_lidar",
                                      std::to_string(lidar_port));
        sensor_http->set_config_param("udp_port_imu", std::to_string(imu_port));

        // if specified (not UNSPEC), set the lidar and timestamp modes
        if (mode) {
            sensor_http->set_config_param("lidar_mode", std::to_string(mode));
        }

        if (ts_mode) {
            sensor_http->set_config_param("timestamp_mode",
                                          std::to_string(ts_mode));
        }

        // wake up from STANDBY, if necessary
        sensor_http->set_config_param("operating_mode", "NORMAL");
        sensor_http->reinitialize();
        // will block until no longer INITIALIZING
        success &=
            collect_metadata(*cli, *sensor_http, chrono::seconds{timeout_sec});
        // check for sensor error states
        auto status = cli->meta["sensor_info"]["status"].asString();
        success &= (status != "ERROR" && status != "UNCONFIGURED");
    } catch (const std::runtime_error& e) {
        // log error message
        std::cerr << "init_client error: " << e.what() << std::endl;
        return std::shared_ptr<client>();
    }

    return success ? cli : std::shared_ptr<client>();
}

client_state poll_client(const client& c, const int timeout_sec) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(c.lidar_fd, &rfds);
    FD_SET(c.imu_fd, &rfds);

    timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;

    SOCKET max_fd = std::max(c.lidar_fd, c.imu_fd);

    SOCKET retval = select((int)max_fd + 1, &rfds, NULL, NULL, &tv);

    client_state res = client_state(0);

    if (!impl::socket_valid(retval) && impl::socket_exit()) {
        res = EXIT;
    } else if (!impl::socket_valid(retval)) {
        std::cerr << "select: " << impl::socket_get_error() << std::endl;
        res = client_state(res | CLIENT_ERROR);
    } else if (retval) {
        if (FD_ISSET(c.lidar_fd, &rfds)) res = client_state(res | LIDAR_DATA);
        if (FD_ISSET(c.imu_fd, &rfds)) res = client_state(res | IMU_DATA);
    }

    return res;
}

static bool recv_fixed(SOCKET fd, void* buf, int64_t len) {
    int64_t bytes_read = recv(fd, (char*)buf, len + 1, 0);

    if (bytes_read == len) {
        return true;
    } else if (bytes_read == -1) {
        std::cerr << "recvfrom: " << impl::socket_get_error() << std::endl;
    } else {
        std::cerr << "Unexpected udp packet length of: " << bytes_read
                  << " bytes. Expected: " << len << " bytes." << std::endl;
    }
    return false;
}

bool read_lidar_packet(const client& cli, uint8_t* buf,
                       const packet_format& pf) {
    return recv_fixed(cli.lidar_fd, buf, pf.lidar_packet_size);
}

bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf) {
    return recv_fixed(cli.imu_fd, buf, pf.imu_packet_size);
}

int get_lidar_port(client& cli) { return get_sock_port(cli.lidar_fd); }

int get_imu_port(client& cli) { return get_sock_port(cli.imu_fd); }

/**
 * Return the socket file descriptor used to listen for lidar UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the socket file descriptor.
 */
extern SOCKET get_lidar_socket_fd(client& cli) { return cli.lidar_fd; }

/**
 * Return the socket file descriptor used to listen for imu UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the socket file descriptor.
 */
extern SOCKET get_imu_socket_fd(client& cli) { return cli.imu_fd; }

}  // namespace sensor
}  // namespace ouster
