/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_tcp.h
 * @brief A high level TCP interface for Ouster sensors.
 *
 */

#pragma once

#include "netcompat.h"

#include "sensor_http.h"

namespace ouster {
namespace sensor {
namespace impl {

/**
 * A TCP implementation of the SensorHTTP interface
 */
class SensorTcpImp : public util::SensorHttp {
    // timeout for reading from a TCP socket during config
    const int RCVTIMEOUT_SEC = 10;
    // maximum size to to handle during recv
    const size_t MAX_RESULT_LENGTH = 16 * 1024;

   public:
    /**
     * Constructs an tcp interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    SensorTcpImp(const std::string& hostname);

    /**
     * Deconstruct the sensor tcp interface.
     */
    ~SensorTcpImp() override;
    /**
     * Queries the sensor metadata.
     *
     * @return returns a Json object of the sensor metadata.
     */
    Json::Value metadata() const override;

    /**
     * Queries the sensor_info.
     *
     * @return returns a Json object representing the sensor_info.
     */
    Json::Value sensor_info() const override;

    /**
     * Queries active/staged configuration on the sensor
     *
     * @param[in] active if true retrieve active, otherwise get staged configs.
     *
     * @return a string represnting the active or staged config
     */
    std::string get_config_params(bool active) const override;

    /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     */
    void set_config_param(const std::string& key,
                          const std::string& value) const override;

    /**
     * Enables automatic assignment of udp destination ports.
     */
    void set_udp_dest_auto() const override;

    /**
     * Retrieves beam intrinsics of the sensor.
     */
    Json::Value beam_intrinsics() const override;

    /**
     * Retrieves imu intrinsics of the sensor.
     */
    Json::Value imu_intrinsics() const override;

    /**
     * Retrieves lidar intrinsics of the sensor.
     */
    Json::Value lidar_intrinsics() const override;

    /**
     * Retrieves lidar data format.
     */
    Json::Value lidar_data_format() const override;

    /**
     * Gets the calibaration status of the sensor.
     */
    Json::Value calibration_status() const override;

    /**
     * Restarts the sensor applying all staged configurations.
     */
    void reinitialize() const override;

    /**
     * Persist active configuration parameters to the sensor.
     */
    void save_config_params() const override;

   private:
    SOCKET cfg_socket(const char* addr);

    std::string tcp_cmd(const std::vector<std::string>& cmd_tokens) const;

    void tcp_cmd_with_validation(const std::vector<std::string>& cmd_tokens,
                                 const std::string& validation) const;

    Json::Value tcp_cmd_json(const std::vector<std::string>& cmd_tokens,
                             bool exception_on_parse_errors = true) const;

   private:
    SOCKET socket_handle;
    std::unique_ptr<char[]> read_buf;
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster