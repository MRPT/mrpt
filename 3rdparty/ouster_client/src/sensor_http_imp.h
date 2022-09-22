/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_http_imp.h
 * @brief An implementation of the HTTP interface for Ouster sensors.
 *
 */

#pragma once

#include "http_client.h"
#include "sensor_http.h"

namespace ouster {
namespace sensor {
namespace impl {

/**
 * An implementation of the sensor http interface
 */
class SensorHttpImp : public util::SensorHttp {
   public:
    /**
     * Constructs an http interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    SensorHttpImp(const std::string& hostname);

    /**
     * Deconstruct the sensor http interface.
     */
    ~SensorHttpImp() override;

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

   protected:
    std::string get(const std::string& url) const;

    Json::Value get_json(const std::string& url) const;

    void execute(const std::string& url, const std::string& validation) const;

   protected:
    std::unique_ptr<ouster::util::HttpClient> http_client;
};

// TODO: remove when firmware 2.2 has been fully phased out
class SensorHttpImp_2_2 : public SensorHttpImp {
   public:
    SensorHttpImp_2_2(const std::string& hostname);

    void set_udp_dest_auto() const override;
};

/**
 * An implementation of the sensor http interface
 */
class SensorHttpImp_2_1 : public SensorHttpImp_2_2 {
   public:
    /**
     * Constructs an http interface to communicate with the sensor.
     *
     * @param[in] hostname hostname of the sensor to communicate with.
     */
    SensorHttpImp_2_1(const std::string& hostname);

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
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
