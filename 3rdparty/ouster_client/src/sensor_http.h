/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_http.h
 * @brief A high level HTTP interface for Ouster sensors.
 *
 */

#pragma once

#include <json/json.h>
#include <ouster/version.h>

#include <memory>

namespace ouster {
namespace sensor {
namespace util {

/**
 * An interface to communicate with Ouster sensors via http requests
 */
class SensorHttp {
   protected:
    /**
     * Constructs an http interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    SensorHttp() = default;

   public:
    /**
     * Deconstruct the sensor http interface.
     */
    virtual ~SensorHttp() = default;

    /**
     * Queries the sensor metadata.
     *
     * @return returns a Json object of the sensor metadata.
     */
    virtual Json::Value metadata() const = 0;

    /**
     * Queries the sensor_info.
     *
     * @return returns a Json object representing the sensor_info.
     */
    virtual Json::Value sensor_info() const = 0;

    /**
     * Queries active/staged configuration on the sensor
     *
     * @param[in] active if true retrieve active, otherwise get staged configs.
     *
     * @return a string represnting the active or staged config
     */
    virtual std::string get_config_params(bool active) const = 0;

    /**
     * Set the value of a specfic configuration on the sensor, the changed
     * configuration is not active until the sensor is restarted.
     *
     * @param[in] key name of the config to change.
     * @param[in] value the new value to set for the selected configuration.
     */
    virtual void set_config_param(const std::string& key,
                                  const std::string& value) const = 0;

    /**
     * Enables automatic assignment of udp destination ports.
     */
    virtual void set_udp_dest_auto() const = 0;

    /**
     * Retrieves beam intrinsics of the sensor.
     */
    virtual Json::Value beam_intrinsics() const = 0;

    /**
     * Retrieves imu intrinsics of the sensor.
     */
    virtual Json::Value imu_intrinsics() const = 0;

    /**
     * Retrieves lidar intrinsics of the sensor.
     */
    virtual Json::Value lidar_intrinsics() const = 0;

    /**
     * Retrieves lidar data format.
     */
    virtual Json::Value lidar_data_format() const = 0;

    /**
     * Gets the calibaration status of the sensor.
     */
    virtual Json::Value calibration_status() const = 0;

    /**
     * Restarts the sensor applying all staged configurations.
     */
    virtual void reinitialize() const = 0;

    /**
     * Persist active configuration parameters to the sensor.
     */
    virtual void save_config_params() const = 0;

    /**
     * Retrieves sensor firmware version information as a string.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    static std::string firmware_version_string(const std::string& hostname);

    /**
     * Retrieves sensor firmware version information.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    static ouster::util::version firmware_version(const std::string& hostname);

    /**
     * Creates an instance of the SensorHttp interface.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    static std::unique_ptr<SensorHttp> create(const std::string& hostname);
};

}  // namespace util
}  // namespace sensor
}  // namespace ouster