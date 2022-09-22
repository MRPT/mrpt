/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief sample sensor client
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ouster/types.h"
#include "ouster/version.h"

namespace ouster {
namespace sensor {

struct client;

/** Returned by poll_client. */
enum client_state {
    TIMEOUT = 0,       ///< Client has timed out
    CLIENT_ERROR = 1,  ///< Client has reported an error
    LIDAR_DATA = 2,    ///< New lidar data available
    IMU_DATA = 4,      ///< New IMU data available
    EXIT = 8           ///< Client has exited
};

/** Minimum supported version. */
const util::version min_version = {1, 12, 0};

/** \defgroup ouster_client_init Ouster Client Client Initialization
 * @{
 */

/**
 * Listen for sensor data on the specified ports; do not configure the sensor.
 *
 * @param[in] hostname The hostname to connect to.
 * @param[in] lidar_port port on which the sensor will send lidar data.
 * @param[in] imu_port port on which the sensor will send imu data.
 *
 * @return pointer owning the resources associated with the connection.
 */
std::shared_ptr<client> init_client(const std::string& hostname = "",
                                    int lidar_port = 7502, int imu_port = 7503);

/**
 * Connect to and configure the sensor and start listening for data.
 *
 * @param[in] hostname hostname or ip of the sensor.
 * @param[in] udp_dest_host hostname or ip where the sensor should send data
 * or "" for automatic detection of destination.
 * @param[in] mode The lidar mode to use.
 * @param[in] ts_mode The timestamp mode to use.
 * @param[in] lidar_port port on which the sensor will send lidar data.
 * @param[in] imu_port port on which the sensor will send imu data.
 * @param[in] timeout_sec how long to wait for the sensor to initialize.
 *
 * @return pointer owning the resources associated with the connection.
 */
std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    lidar_mode mode = MODE_UNSPEC,
                                    timestamp_mode ts_mode = TIME_FROM_UNSPEC,
                                    int lidar_port = 0, int imu_port = 0,
                                    int timeout_sec = 60);
/** @}*/

/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 *
 * NOTE: will return immediately if LIDAR_DATA or IMU_DATA are set and not
 * cleared by read_lidar_data() and read_imu_data() before the next call.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[in] timeout_sec seconds to block while waiting for data.
 *
 * @return client_state s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read, and (s & IMU_DATA) is
 * true if imu data is ready to read.
 */
client_state poll_client(const client& cli, int timeout_sec = 1);

/**
 * Read lidar data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if a lidar packet was successfully read.
 */
bool read_lidar_packet(const client& cli, uint8_t* buf,
                       const packet_format& pf);

/**
 * Read imu data from the sensor. Will not block.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[out] buf buffer to which to write imu data. Must be at least
 * imu_packet_bytes + 1 bytes.
 * @param[in] pf The packet format.
 *
 * @return true if an imu packet was successfully read.
 */
bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf);

/**
 * Get metadata text blob from the sensor.
 *
 * Will attempt to fetch from the network if not already populated.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 * @param[in] timeout_sec how long to wait for the sensor to initialize.
 * @param[in] legacy_format whether to use legacy format of metadata output.
 *
 * @return a text blob of metadata parseable into a sensor_info struct.
 */
std::string get_metadata(client& cli, int timeout_sec = 60,
                         bool legacy_format = true);

/**
 * Get sensor config from the sensor.
 *
 * Populates passed in config with the results of get_config.
 *
 * @param[in] hostname sensor hostname.
 * @param[out] config sensor config to populate.
 * @param[in] active whether to pull active or passive configs.
 *
 * @return true if sensor config successfully populated.
 */
bool get_config(const std::string& hostname, sensor_config& config,
                bool active = true);

/**
 * Flags for set_config()
 */
enum config_flags : uint8_t {
    CONFIG_UDP_DEST_AUTO = (1 << 0),  ///< Set udp_dest automatically
    CONFIG_PERSIST = (1 << 1)         ///< Make configuration persistent
};

/**
 * Set sensor config on sensor.
 *
 * @throw runtime_error on failure to communcate with the sensor.
 * @throw invalid_argument when config parameters fail validation.
 *
 * @param[in] hostname sensor hostname.
 * @param[in] config sensor config.
 * @param[in] config_flags flags to pass in.
 *
 * @return true if config params successfuly set on sensor.
 */
bool set_config(const std::string& hostname, const sensor_config& config,
                uint8_t config_flags = 0);

/**
 * Return the port used to listen for lidar UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the port number.
 */
int get_lidar_port(client& cli);

/**
 * Return the port used to listen for imu UDP data.
 *
 * @param[in] cli client returned by init_client associated with the connection.
 *
 * @return the port number.
 */
int get_imu_port(client& cli);

}  // namespace sensor
}  // namespace ouster
