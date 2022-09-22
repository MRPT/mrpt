#include "sensor_http_imp.h"

#include "curl_client.h"

using std::string;
using namespace ouster::sensor::impl;

SensorHttpImp::SensorHttpImp(const string& hostname)
    : http_client(std::make_unique<CurlClient>("http://" + hostname)) {}

SensorHttpImp::~SensorHttpImp() = default;

Json::Value SensorHttpImp::metadata() const {
    return get_json("api/v1/sensor/metadata");
}

Json::Value SensorHttpImp::sensor_info() const {
    return get_json("api/v1/sensor/metadata/sensor_info");
}

string SensorHttpImp::get_config_params(bool active) const {
    auto config_type = active ? "active" : "staged";
    return get(string("api/v1/sensor/cmd/get_config_param?args=") +
               config_type);
}

void SensorHttpImp::set_config_param(const string& key,
                                     const string& value) const {
    auto encoded_value = http_client->encode(value);  // encode config params
    auto url =
        "api/v1/sensor/cmd/set_config_param?args=" + key + "+" + encoded_value;
    execute(url, "\"set_config_param\"");
}

void SensorHttpImp::set_udp_dest_auto() const {
    execute("api/v1/sensor/cmd/set_udp_dest_auto", "{}");
}

Json::Value SensorHttpImp::beam_intrinsics() const {
    return get_json("api/v1/sensor/metadata/beam_intrinsics");
}

Json::Value SensorHttpImp::imu_intrinsics() const {
    return get_json("api/v1/sensor/metadata/imu_intrinsics");
}

Json::Value SensorHttpImp::lidar_intrinsics() const {
    return get_json("api/v1/sensor/metadata/lidar_intrinsics");
}

Json::Value SensorHttpImp::lidar_data_format() const {
    return get_json("api/v1/sensor/metadata/lidar_data_format");
}

Json::Value SensorHttpImp::calibration_status() const {
    return get_json("api/v1/sensor/metadata/calibration_status");
}

// reinitialize to activate new settings
void SensorHttpImp::reinitialize() const {
    execute("api/v1/sensor/cmd/reinitialize", "{}");
}

void SensorHttpImp::save_config_params() const {
    execute("api/v1/sensor/cmd/save_config_params", "{}");
}

string SensorHttpImp::get(const string& url) const {
    return http_client->get(url);
}

Json::Value SensorHttpImp::get_json(const string& url) const {
    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value root;
    auto result = get(url);
    if (!reader->parse(result.c_str(), result.c_str() + result.size(), &root,
                       nullptr))
        throw std::runtime_error("SensorHttpImp::get_json failed! url: " + url);
    return root;
}

void SensorHttpImp::execute(const string& url, const string& validation) const {
    auto result = get(url);
    if (result != validation)
        throw std::runtime_error("SensorHttpImp::execute failed! url: " + url +
                                 " returned [" + result + "], expected [" +
                                 validation + "]");
}

SensorHttpImp_2_2::SensorHttpImp_2_2(const string& hostname)
    : SensorHttpImp(hostname) {}

void SensorHttpImp_2_2::set_udp_dest_auto() const {
    return execute("api/v1/sensor/cmd/set_udp_dest_auto",
                   "\"set_config_param\"");
}

SensorHttpImp_2_1::SensorHttpImp_2_1(const string& hostname)
    : SensorHttpImp_2_2(hostname) {}

Json::Value SensorHttpImp_2_1::metadata() const {
    Json::Value root;
    root["sensor_info"] = sensor_info();
    root["beam_intrinsics"] = beam_intrinsics();
    root["imu_intrinsics"] = imu_intrinsics();
    root["lidar_intrinsics"] = lidar_intrinsics();
    root["lidar_data_format"] = lidar_data_format();
    root["calibration_status"] = calibration_status();

    Json::CharReaderBuilder builder;
    auto reader = std::unique_ptr<Json::CharReader>{builder.newCharReader()};
    Json::Value node;
    auto res = get_config_params(true);
    auto parse_success =
        reader->parse(res.c_str(), res.c_str() + res.size(), &node, nullptr);
    root["config_params"] = parse_success ? node : res;
    return root;
}

Json::Value SensorHttpImp_2_1::sensor_info() const {
    return get_json("api/v1/sensor/cmd/get_sensor_info");
}

Json::Value SensorHttpImp_2_1::beam_intrinsics() const {
    return get_json("api/v1/sensor/cmd/get_beam_intrinsics");
}

Json::Value SensorHttpImp_2_1::imu_intrinsics() const {
    return get_json("api/v1/sensor/cmd/get_imu_intrinsics");
}

Json::Value SensorHttpImp_2_1::lidar_intrinsics() const {
    return get_json("api/v1/sensor/cmd/get_lidar_intrinsics");
}

Json::Value SensorHttpImp_2_1::lidar_data_format() const {
    return get_json("api/v1/sensor/cmd/get_lidar_data_format");
}

Json::Value SensorHttpImp_2_1::calibration_status() const {
    return get_json("api/v1/sensor/cmd/get_calibration_status");
}
