/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/types.h"

#include <json/json.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster/impl/build.h"
#include "ouster/version.h"

namespace ouster {

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

namespace sensor {

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

extern const Table<lidar_mode, const char*, 7> lidar_mode_strings{
    {{MODE_UNSPEC, "UNKNOWN"},
     {MODE_512x10, "512x10"},
     {MODE_512x20, "512x20"},
     {MODE_1024x10, "1024x10"},
     {MODE_1024x20, "1024x20"},
     {MODE_2048x10, "2048x10"},
     {MODE_4096x5, "4096x5"}}};

extern const Table<timestamp_mode, const char*, 4> timestamp_mode_strings{
    {{TIME_FROM_UNSPEC, "UNKNOWN"},
     {TIME_FROM_INTERNAL_OSC, "TIME_FROM_INTERNAL_OSC"},
     {TIME_FROM_SYNC_PULSE_IN, "TIME_FROM_SYNC_PULSE_IN"},
     {TIME_FROM_PTP_1588, "TIME_FROM_PTP_1588"}}};

extern const Table<OperatingMode, const char*, 2> operating_mode_strings{
    {{OPERATING_NORMAL, "NORMAL"}, {OPERATING_STANDBY, "STANDBY"}}};

extern const Table<MultipurposeIOMode, const char*, 6>
    multipurpose_io_mode_strings{
        {{MULTIPURPOSE_OFF, "OFF"},
         {MULTIPURPOSE_INPUT_NMEA_UART, "INPUT_NMEA_UART"},
         {MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC, "OUTPUT_FROM_INTERNAL_OSC"},
         {MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN, "OUTPUT_FROM_SYNC_PULSE_IN"},
         {MULTIPURPOSE_OUTPUT_FROM_PTP_1588, "OUTPUT_FROM_PTP_1588"},
         {MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE,
          "OUTPUT_FROM_ENCODER_ANGLE"}}};

extern const Table<Polarity, const char*, 2> polarity_strings{
    {{POLARITY_ACTIVE_LOW, "ACTIVE_LOW"},
     {POLARITY_ACTIVE_HIGH, "ACTIVE_HIGH"}}};

extern const Table<NMEABaudRate, const char*, 2> nmea_baud_rate_strings{
    {{BAUD_9600, "BAUD_9600"}, {BAUD_115200, "BAUD_115200"}}};

Table<sensor::ChanField, const char*, 23> chanfield_strings{{
    {ChanField::RANGE, "RANGE"},
    {ChanField::RANGE2, "RANGE2"},
    {ChanField::SIGNAL, "SIGNAL"},
    {ChanField::SIGNAL2, "SIGNAL2"},
    {ChanField::REFLECTIVITY, "REFLECTIVITY"},
    {ChanField::REFLECTIVITY2, "REFLECTIVITY2"},
    {ChanField::NEAR_IR, "NEAR_IR"},
    {ChanField::FLAGS, "FLAGS"},
    {ChanField::FLAGS2, "FLAGS2"},
    {ChanField::CUSTOM0, "CUSTOM0"},
    {ChanField::CUSTOM1, "CUSTOM1"},
    {ChanField::CUSTOM2, "CUSTOM2"},
    {ChanField::CUSTOM3, "CUSTOM3"},
    {ChanField::CUSTOM4, "CUSTOM4"},
    {ChanField::CUSTOM5, "CUSTOM5"},
    {ChanField::CUSTOM6, "CUSTOM6"},
    {ChanField::CUSTOM7, "CUSTOM7"},
    {ChanField::CUSTOM8, "CUSTOM8"},
    {ChanField::CUSTOM9, "CUSTOM9"},
    {ChanField::RAW32_WORD1, "RAW32_WORD1"},
    {ChanField::RAW32_WORD2, "RAW32_WORD2"},
    {ChanField::RAW32_WORD3, "RAW32_WORD3"},
    {ChanField::RAW32_WORD4, "RAW32_WORD4"},
}};

Table<UDPProfileLidar, const char*, 4> udp_profile_lidar_strings{{
    {PROFILE_LIDAR_LEGACY, "LEGACY"},
    {PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL, "RNG19_RFL8_SIG16_NIR16_DUAL"},
    {PROFILE_RNG19_RFL8_SIG16_NIR16, "RNG19_RFL8_SIG16_NIR16"},
    {PROFILE_RNG15_RFL8_NIR8, "RNG15_RFL8_NIR8"},
}};

Table<UDPProfileIMU, const char*, 1> udp_profile_imu_strings{{
    {PROFILE_IMU_LEGACY, "LEGACY"},
}};

}  // namespace impl

/* Equality operators */

bool operator==(const data_format& lhs, const data_format& rhs) {
    return (lhs.pixels_per_column == rhs.pixels_per_column &&
            lhs.columns_per_packet == rhs.columns_per_packet &&
            lhs.columns_per_frame == rhs.columns_per_frame &&
            lhs.pixel_shift_by_row == rhs.pixel_shift_by_row &&
            lhs.column_window == rhs.column_window &&
            lhs.udp_profile_lidar == rhs.udp_profile_lidar &&
            lhs.udp_profile_imu == rhs.udp_profile_imu);
}

bool operator!=(const data_format& lhs, const data_format& rhs) {
    return !(lhs == rhs);
}

bool operator==(const sensor_info& lhs, const sensor_info& rhs) {
    return (lhs.name == rhs.name && lhs.sn == rhs.sn &&
            lhs.fw_rev == rhs.fw_rev && lhs.mode == rhs.mode &&
            lhs.prod_line == rhs.prod_line && lhs.format == rhs.format &&
            lhs.beam_azimuth_angles == rhs.beam_azimuth_angles &&
            lhs.beam_altitude_angles == rhs.beam_altitude_angles &&
            lhs.lidar_origin_to_beam_origin_mm ==
                rhs.lidar_origin_to_beam_origin_mm &&
            lhs.imu_to_sensor_transform == rhs.imu_to_sensor_transform &&
            lhs.lidar_to_sensor_transform == rhs.lidar_to_sensor_transform &&
            lhs.extrinsic == rhs.extrinsic && lhs.init_id == rhs.init_id &&
            lhs.udp_port_lidar == rhs.udp_port_lidar &&
            lhs.udp_port_imu == rhs.udp_port_imu);
}

bool operator!=(const sensor_info& lhs, const sensor_info& rhs) {
    return !(lhs == rhs);
}

bool operator==(const sensor_config& lhs, const sensor_config& rhs) {
    return (lhs.udp_dest == rhs.udp_dest &&
            lhs.udp_port_lidar == rhs.udp_port_lidar &&
            lhs.udp_port_imu == rhs.udp_port_imu &&
            lhs.ts_mode == rhs.ts_mode && lhs.ld_mode == rhs.ld_mode &&
            lhs.operating_mode == rhs.operating_mode &&
            lhs.multipurpose_io_mode == rhs.multipurpose_io_mode &&
            lhs.azimuth_window == rhs.azimuth_window &&
            lhs.signal_multiplier == rhs.signal_multiplier &&
            lhs.nmea_in_polarity == rhs.nmea_in_polarity &&
            lhs.nmea_ignore_valid_char == rhs.nmea_ignore_valid_char &&
            lhs.nmea_baud_rate == rhs.nmea_baud_rate &&
            lhs.nmea_leap_seconds == rhs.nmea_leap_seconds &&
            lhs.sync_pulse_in_polarity == rhs.sync_pulse_in_polarity &&
            lhs.sync_pulse_out_polarity == rhs.sync_pulse_out_polarity &&
            lhs.sync_pulse_out_angle == rhs.sync_pulse_out_angle &&
            lhs.sync_pulse_out_pulse_width == rhs.sync_pulse_out_pulse_width &&
            lhs.sync_pulse_out_frequency == rhs.sync_pulse_out_frequency &&
            lhs.phase_lock_enable == rhs.phase_lock_enable &&
            lhs.phase_lock_offset == rhs.phase_lock_offset &&
            lhs.columns_per_packet == rhs.columns_per_packet &&
            lhs.udp_profile_lidar == rhs.udp_profile_lidar &&
            lhs.udp_profile_imu == rhs.udp_profile_imu);
}

bool operator!=(const sensor_config& lhs, const sensor_config& rhs) {
    return !(lhs == rhs);
}

/* Default values */

static ColumnWindow default_column_window(uint32_t columns_per_frame) {
    return {0, columns_per_frame - 1};
}

data_format default_data_format(lidar_mode mode) {
    auto repeat = [](int n, const std::vector<int>& v) {
        std::vector<int> res{};
        for (int i = 0; i < n; i++) res.insert(res.end(), v.begin(), v.end());
        return res;
    };

    uint32_t pixels_per_column = 64;
    uint32_t columns_per_packet = 16;
    uint32_t columns_per_frame = n_cols_of_lidar_mode(mode);
    ColumnWindow column_window = default_column_window(columns_per_frame);

    std::vector<int> offset;
    switch (columns_per_frame) {
        case 512:
            offset = repeat(16, {9, 6, 3, 0});
            break;
        case 1024:
            offset = repeat(16, {18, 12, 6, 0});
            break;
        case 2048:
            offset = repeat(16, {36, 24, 12, 0});
            break;
        default:
            throw std::invalid_argument{"default_data_format"};
    }

    return {pixels_per_column,
            columns_per_packet,
            columns_per_frame,
            offset,
            column_window,
            UDPProfileLidar::PROFILE_LIDAR_LEGACY,
            UDPProfileIMU::PROFILE_IMU_LEGACY};
}

static double default_lidar_origin_to_beam_origin(std::string prod_line) {
    double lidar_origin_to_beam_origin_mm = 12.163;  // default for gen 1
    if (prod_line.find("OS-0-") == 0)
        lidar_origin_to_beam_origin_mm = 27.67;
    else if (prod_line.find("OS-1-") == 0)
        lidar_origin_to_beam_origin_mm = 15.806;
    else if (prod_line.find("OS-2-") == 0)
        lidar_origin_to_beam_origin_mm = 13.762;
    return lidar_origin_to_beam_origin_mm;
}

sensor_info default_sensor_info(lidar_mode mode) {
    return sensor::sensor_info{"UNKNOWN",
                               "000000000000",
                               "UNKNOWN",
                               mode,
                               "OS-1-64",
                               default_data_format(mode),
                               gen1_azimuth_angles,
                               gen1_altitude_angles,
                               default_lidar_origin_to_beam_origin("OS-1-64"),
                               default_imu_to_sensor_transform,
                               default_lidar_to_sensor_transform,
                               mat4d::Identity(),
                               0,
                               0,
                               0};
}

extern const std::vector<double> gen1_altitude_angles = {
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

extern const std::vector<double> gen1_azimuth_angles = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

extern const mat4d default_imu_to_sensor_transform =
    (mat4d() << 1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1)
        .finished();

extern const mat4d default_lidar_to_sensor_transform =
    (mat4d() << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1)
        .finished();

/* Misc operations */

uint32_t n_cols_of_lidar_mode(lidar_mode mode) {
    switch (mode) {
        case MODE_512x10:
        case MODE_512x20:
            return 512;
        case MODE_1024x10:
        case MODE_1024x20:
            return 1024;
        case MODE_2048x10:
            return 2048;
        case MODE_4096x5:
            return 4096;
        default:
            throw std::invalid_argument{"n_cols_of_lidar_mode"};
    }
}

int frequency_of_lidar_mode(lidar_mode mode) {
    switch (mode) {
        case MODE_4096x5:
            return 5;
        case MODE_512x10:
        case MODE_1024x10:
        case MODE_2048x10:
            return 10;
        case MODE_512x20:
        case MODE_1024x20:
            return 20;
        default:
            throw std::invalid_argument{"frequency_of_lidar_mode"};
    }
}

std::string client_version() {
    return std::string("ouster_client ").append(ouster::SDK_VERSION);
}

/* String conversion */

template <typename K, typename V, size_t N>
static optional<V> lookup(const impl::Table<K, V, N> table, const K& k) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end, [&](const std::pair<K, V>& p) {
        return p.first == k;
    });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
static optional<K> rlookup(const impl::Table<K, const char*, N> table,
                           const char* v) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end,
                            [&](const std::pair<K, const char*>& p) {
                                return std::strcmp(p.second, v) == 0;
                            });

    return res == end ? nullopt : make_optional<K>(res->first);
}

std::string to_string(lidar_mode mode) {
    auto res = lookup(impl::lidar_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

lidar_mode lidar_mode_of_string(const std::string& s) {
    auto res = rlookup(impl::lidar_mode_strings, s.c_str());
    return res ? res.value() : lidar_mode::MODE_UNSPEC;
}

std::string to_string(timestamp_mode mode) {
    auto res = lookup(impl::timestamp_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

timestamp_mode timestamp_mode_of_string(const std::string& s) {
    auto res = rlookup(impl::timestamp_mode_strings, s.c_str());
    return res ? res.value() : timestamp_mode::TIME_FROM_UNSPEC;
}

std::string to_string(OperatingMode mode) {
    auto res = lookup(impl::operating_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<OperatingMode> operating_mode_of_string(const std::string& s) {
    return rlookup(impl::operating_mode_strings, s.c_str());
}

std::string to_string(MultipurposeIOMode mode) {
    auto res = lookup(impl::multipurpose_io_mode_strings, mode);
    return res ? res.value() : "UNKNOWN";
}

optional<MultipurposeIOMode> multipurpose_io_mode_of_string(
    const std::string& s) {
    return rlookup(impl::multipurpose_io_mode_strings, s.c_str());
}

std::string to_string(Polarity polarity) {
    auto res = lookup(impl::polarity_strings, polarity);
    return res ? res.value() : "UNKNOWN";
}

optional<Polarity> polarity_of_string(const std::string& s) {
    return rlookup(impl::polarity_strings, s.c_str());
}

std::string to_string(NMEABaudRate rate) {
    auto res = lookup(impl::nmea_baud_rate_strings, rate);
    return res ? res.value() : "UNKNOWN";
}

optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& s) {
    return rlookup(impl::nmea_baud_rate_strings, s.c_str());
}

std::string to_string(AzimuthWindow azimuth_window) {
    std::stringstream ss;
    ss << "[" << azimuth_window.first << ", " << azimuth_window.second << "]";
    return ss.str();
}

std::string to_string(ChanField field) {
    auto res = lookup(impl::chanfield_strings, field);
    return res ? res.value() : "UNKNOWN";
}

std::string to_string(UDPProfileLidar profile) {
    auto res = lookup(impl::udp_profile_lidar_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<UDPProfileLidar> udp_profile_lidar_of_string(const std::string& s) {
    return rlookup(impl::udp_profile_lidar_strings, s.c_str());
}

std::string to_string(UDPProfileIMU profile) {
    auto res = lookup(impl::udp_profile_imu_strings, profile);
    return res ? res.value() : "UNKNOWN";
}

optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& s) {
    return rlookup(impl::udp_profile_imu_strings, s.c_str());
}

static sensor_config parse_config(const Json::Value& root) {
    sensor_config config{};

    if (!root["udp_dest"].empty()) {
        config.udp_dest = root["udp_dest"].asString();
    } else if (!root["udp_ip"].empty()) {
        // deprecated params from FW 1.13. Set FW 2.0+ configs appropriately
        config.udp_dest = root["udp_ip"].asString();
        std::cerr << "Please note that udp_ip has been deprecated in favor of "
                     "udp_dest. Will set udp_dest appropriately..."
                  << std::endl;
    }

    if (!root["udp_port_lidar"].empty())
        config.udp_port_lidar = root["udp_port_lidar"].asInt();
    if (!root["udp_port_imu"].empty())
        config.udp_port_imu = root["udp_port_imu"].asInt();
    if (!root["timestamp_mode"].empty())
        config.ts_mode =
            timestamp_mode_of_string(root["timestamp_mode"].asString());
    if (!root["lidar_mode"].empty())
        config.ld_mode = lidar_mode_of_string(root["lidar_mode"].asString());

    if (!root["azimuth_window"].empty())
        config.azimuth_window =
            std::make_pair(root["azimuth_window"][0].asInt(),
                           root["azimuth_window"][1].asInt());

    if (!root["signal_multiplier"].empty())
        config.signal_multiplier = root["signal_multiplier"].asInt();

    if (!root["operating_mode"].empty()) {
        auto operating_mode =
            operating_mode_of_string(root["operating_mode"].asString());
        if (operating_mode) {
            config.operating_mode = operating_mode;
        } else {
            throw std::invalid_argument("Unexpected Operating Mode");
        }
    } else if (!root["auto_start_flag"].empty()) {
        std::cerr
            << "Please note that auto_start_flag has been deprecated in favor "
               "of operating_mode. Will set operating_mode appropriately..."
            << std::endl;
        config.operating_mode = root["auto_start_flag"].asBool()
                                    ? sensor::OPERATING_NORMAL
                                    : sensor::OPERATING_STANDBY;
    }

    if (!root["multipurpose_io_mode"].empty()) {
        auto multipurpose_io_mode = multipurpose_io_mode_of_string(
            root["multipurpose_io_mode"].asString());
        if (multipurpose_io_mode) {
            config.multipurpose_io_mode = multipurpose_io_mode;
        } else {
            throw std::invalid_argument("Unexpected Multipurpose IO Mode");
        }
    }
    if (!root["sync_pulse_out_angle"].empty())
        config.sync_pulse_out_angle = root["sync_pulse_out_angle"].asInt();
    if (!root["sync_pulse_out_pulse_width"].empty())
        config.sync_pulse_out_pulse_width =
            root["sync_pulse_out_pulse_width"].asInt();

    if (!root["nmea_in_polarity"].empty()) {
        auto nmea_in_polarity =
            polarity_of_string(root["nmea_in_polarity"].asString());
        if (nmea_in_polarity) {
            config.nmea_in_polarity = nmea_in_polarity;
        } else {
            throw std::invalid_argument("Unexpected NMEA Input Polarity");
        }
    }
    if (!root["nmea_baud_rate"].empty()) {
        auto nmea_baud_rate =
            nmea_baud_rate_of_string(root["nmea_baud_rate"].asString());
        if (nmea_baud_rate) {
            config.nmea_baud_rate = nmea_baud_rate;
        } else {
            throw std::invalid_argument("Unexpected NMEA Baud Rate");
        }
    }
    if (!root["nmea_ignore_valid_char"].empty())
        config.nmea_ignore_valid_char = root["nmea_ignore_valid_char"].asBool();
    if (!root["nmea_leap_seconds"].empty())
        config.nmea_leap_seconds = root["nmea_leap_seconds"].asInt();

    if (!root["sync_pulse_in_polarity"].empty()) {
        auto sync_pulse_in_polarity =
            polarity_of_string(root["sync_pulse_in_polarity"].asString());
        if (sync_pulse_in_polarity) {
            config.sync_pulse_in_polarity = sync_pulse_in_polarity;
        } else {
            throw std::invalid_argument("Unexpected Sync Pulse Input Polarity");
        }
    }
    if (!root["sync_pulse_out_polarity"].empty()) {
        auto sync_pulse_out_polarity =
            polarity_of_string(root["sync_pulse_out_polarity"].asString());
        if (sync_pulse_out_polarity) {
            config.sync_pulse_out_polarity = sync_pulse_out_polarity;
        } else {
            throw std::invalid_argument(
                "Unexpected Sync Pulse Output Polarity");
        }
    }
    if (!root["sync_pulse_out_frequency"].empty())
        config.sync_pulse_out_frequency =
            root["sync_pulse_out_frequency"].asInt();

    if (!root["phase_lock_enable"].empty())
        config.phase_lock_enable =
            root["phase_lock_enable"].asString() == "true" ? true : false;

    if (!root["phase_lock_offset"].empty())
        config.phase_lock_offset = root["phase_lock_offset"].asInt();

    if (!root["columns_per_packet"].empty())
        config.columns_per_packet = root["columns_per_packet"].asInt();

    // udp_profiles
    if (!root["udp_profile_lidar"].empty())
        config.udp_profile_lidar =
            udp_profile_lidar_of_string(root["udp_profile_lidar"].asString());

    if (!root["udp_profile_imu"].empty())
        config.udp_profile_imu =
            udp_profile_imu_of_string(root["udp_profile_imu"].asString());

    return config;
}

sensor_config parse_config(const std::string& config) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{config};

    if (config.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors)) {
            throw std::invalid_argument{errors.c_str()};
        }
    }

    return parse_config(root);
}

static bool valid_response(const Json::Value& root,
                           const std::string& tcp_request) {
    return (root.isMember(tcp_request) && root[tcp_request].isObject());
}

// TODO make robust to new formats that are incorrect instead of returning false
// and sending to legacy
static bool is_new_format(const std::string& metadata) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::invalid_argument{errors.c_str()};
    }

    const std::vector<std::string> valid_response_required = {
        "sensor_info", "beam_intrinsics", "imu_intrinsics", "lidar_intrinsics",
        "config_params"};

    for (const auto& key : valid_response_required) {
        if (!valid_response(root, key)) {
            return false;
        }
    }

    const std::vector<std::string> member_required = {"lidar_data_format",
                                                      "calibration_status"};

    for (const auto& key : member_required) {
        if (!root.isMember(key)) {
            return false;
        }
    }

    return true;
}

static data_format parse_data_format(const Json::Value& root) {
    data_format format;

    format.pixels_per_column = root["pixels_per_column"].asInt();
    format.columns_per_packet = root["columns_per_packet"].asInt();
    format.columns_per_frame = root["columns_per_frame"].asInt();

    if (root["pixel_shift_by_row"].size() != format.pixels_per_column) {
        throw std::invalid_argument{"Unexpected number of pixel_shift_by_row"};
    }

    for (const auto& v : root["pixel_shift_by_row"])
        format.pixel_shift_by_row.push_back(v.asInt());

    if (root.isMember("column_window")) {
        if (root["column_window"].size() != 2) {
            throw std::invalid_argument{
                "Unexpected size of column_window tuple"};
        }
        format.column_window.first = root["column_window"][0].asInt();
        format.column_window.second = root["column_window"][1].asInt();
    } else {
        std::cerr << "WARNING: No column window found." << std::endl;
        format.column_window = default_column_window(format.columns_per_frame);
    }

    if (root.isMember("udp_profile_lidar")) {
        // initializing directly triggers -Wmaybe-uninitialized GCC 8.3.1
        optional<UDPProfileLidar> profile{nullopt};
        profile =
            udp_profile_lidar_of_string(root["udp_profile_lidar"].asString());
        if (profile) {
            format.udp_profile_lidar = profile.value();
        } else {
            throw std::invalid_argument{"Unexpected udp lidar profile"};
        }
    } else {
        std::cerr << "WARNING: No lidar profile found." << std::endl;
        format.udp_profile_lidar = PROFILE_LIDAR_LEGACY;
    }

    if (root.isMember("udp_profile_imu")) {
        optional<UDPProfileIMU> profile{nullopt};
        profile = udp_profile_imu_of_string(root["udp_profile_imu"].asString());
        if (profile) {
            format.udp_profile_imu = profile.value();
        } else {
            throw std::invalid_argument{"Unexpected udp imu profile"};
        }
    } else {
        format.udp_profile_imu = PROFILE_IMU_LEGACY;
    }

    return format;
}

static sensor_info parse_legacy(const std::string& meta) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{meta};

    if (meta.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::invalid_argument{errors.c_str()};
    }
    sensor_info info{};

    // info.name is deprecated, will be empty string if not present
    info.name = root["hostname"].asString();

    info.sn = root["prod_sn"].asString();
    info.fw_rev = root["build_rev"].asString();
    info.mode = lidar_mode_of_string(root["lidar_mode"].asString());
    info.prod_line = root["prod_line"].asString();

    // "data_format" introduced in fw 2.0. Fall back to 1.13
    if (root.isMember("data_format")) {
        info.format = parse_data_format(root["data_format"]);
    } else {
        std::cerr << "WARNING: No data_format found." << std::endl;
        info.format = default_data_format(info.mode);
    }

    // "lidar_origin_to_beam_origin_mm" introduced in fw 2.0. Fall back
    // to 1.13
    if (root.isMember("lidar_origin_to_beam_origin_mm")) {
        info.lidar_origin_to_beam_origin_mm =
            root["lidar_origin_to_beam_origin_mm"].asDouble();
    } else {
        std::cerr << "WARNING: No lidar_origin_to_beam_origin_mm found."
                  << std::endl;
        info.lidar_origin_to_beam_origin_mm =
            default_lidar_origin_to_beam_origin(info.prod_line);
    }

    if (root["beam_altitude_angles"].size() != info.format.pixels_per_column) {
        throw std::invalid_argument{
            "Unexpected number of beam_altitude_angles"};
    }

    if (root["beam_azimuth_angles"].size() != info.format.pixels_per_column) {
        throw std::invalid_argument{"Unexpected number of beam_azimuth_angles"};
    }

    for (const auto& v : root["beam_altitude_angles"])
        info.beam_altitude_angles.push_back(v.asDouble());

    for (const auto& v : root["beam_azimuth_angles"])
        info.beam_azimuth_angles.push_back(v.asDouble());

    // "imu_to_sensor_transform" may be absent in sensor config
    // produced by Ouster Studio, so we backfill it with default value
    if (root["imu_to_sensor_transform"].size() == 16) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.imu_to_sensor_transform(i, j) =
                    root["imu_to_sensor_transform"][ind].asDouble();
            }
        }
    } else {
        std::cerr << "WARNING: No valid imu_to_sensor_transform found."
                  << std::endl;
        info.imu_to_sensor_transform = default_imu_to_sensor_transform;
    }

    // "lidar_to_sensor_transform" may be absent in sensor config
    // produced by Ouster Studio, so we backfill it with default value
    if (root["lidar_to_sensor_transform"].size() == 16) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                const Json::Value::ArrayIndex ind = i * 4 + j;
                info.lidar_to_sensor_transform(i, j) =
                    root["lidar_to_sensor_transform"][ind].asDouble();
            }
        }
    } else {
        std::cerr << "WARNING: No valid lidar_to_sensor_transform found."
                  << std::endl;
        info.lidar_to_sensor_transform = default_lidar_to_sensor_transform;
    }

    info.extrinsic = mat4d::Identity();

    // default to 0 if keys are not present
    info.init_id = root["initialization_id"].asInt();
    info.udp_port_lidar = root["udp_port_lidar"].asInt();
    info.udp_port_imu = root["udp_port_imu"].asInt();

    return info;
}

static void update_json_obj(Json::Value& dst, const Json::Value& src) {
    const std::vector<std::string>& members = src.getMemberNames();
    for (const auto& key : members) {
        dst[key] = src[key];
    }
}

/* version field required by ouster studio */
enum configuration_version { FW_2_0 = 3, FW_2_2 = 4 };

std::string convert_to_legacy(const std::string& metadata) {
    if (!is_new_format(metadata))
        throw std::invalid_argument(
            "Could not convert invalid non-legacy metadata format");

    Json::Value root{};
    Json::CharReaderBuilder read_builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(read_builder, ss, &root, &errors)) {
            throw std::invalid_argument{errors.c_str()};
        }
    }
    Json::Value result{};

    if (root.isMember("config_params")) {
        result["lidar_mode"] = root["config_params"]["lidar_mode"];
        result["udp_port_lidar"] = root["config_params"]["udp_port_lidar"];
        result["udp_port_imu"] = root["config_params"]["udp_port_imu"];
    }
    if (root.isMember("client_version")) {
        result["client_version"] = root["client_version"];
    }
    result["json_calibration_version"] = FW_2_2;

    result["hostname"] = "";

    update_json_obj(result, root["sensor_info"]);
    update_json_obj(result, root["beam_intrinsics"]);
    update_json_obj(result, root["imu_intrinsics"]);
    update_json_obj(result, root["lidar_intrinsics"]);

    if (root.isMember("lidar_data_format") &&
        root["lidar_data_format"].isObject()) {
        result["data_format"] = Json::Value{};
        update_json_obj(result["data_format"], root["lidar_data_format"]);
    }

    Json::StreamWriterBuilder write_builder;
    write_builder["enableYAMLCompatibility"] = true;
    write_builder["precision"] = 6;
    write_builder["indentation"] = "    ";
    return Json::writeString(write_builder, result);
}

sensor_info parse_metadata(const std::string& metadata) {
    Json::Value root{};
    Json::CharReaderBuilder builder{};
    std::string errors{};
    std::stringstream ss{metadata};

    if (metadata.size()) {
        if (!Json::parseFromStream(builder, ss, &root, &errors))
            throw std::invalid_argument{errors.c_str()};
    }

    sensor_info info{};
    if (is_new_format(metadata)) {
        info = parse_legacy(convert_to_legacy(metadata));
    } else {
        info = parse_legacy(metadata);
    }
    return info;
}

sensor_info metadata_from_json(const std::string& json_file) {
    std::stringstream buf{};
    std::ifstream ifs{};
    ifs.open(json_file);
    buf << ifs.rdbuf();
    ifs.close();

    if (!ifs) {
        std::stringstream ss;
        ss << "Failed to read metadata file: " << json_file;
        throw std::runtime_error(ss.str());
    }

    return parse_metadata(buf.str());
}

std::string to_string(const sensor_info& info) {
    Json::Value root{};

    root["client_version"] = client_version();
    root["hostname"] = "";
    root["prod_sn"] = info.sn;
    root["build_rev"] = info.fw_rev;
    root["lidar_mode"] = to_string(info.mode);
    root["prod_line"] = info.prod_line;

    root["data_format"]["pixels_per_column"] = info.format.pixels_per_column;
    root["data_format"]["columns_per_packet"] = info.format.columns_per_packet;
    root["data_format"]["columns_per_frame"] = info.format.columns_per_frame;
    for (auto i : info.format.pixel_shift_by_row)
        root["data_format"]["pixel_shift_by_row"].append(i);

    root["data_format"]["column_window"].append(
        info.format.column_window.first);
    root["data_format"]["column_window"].append(
        info.format.column_window.second);

    root["data_format"]["udp_profile_lidar"] =
        to_string(info.format.udp_profile_lidar);
    root["data_format"]["udp_profile_imu"] =
        to_string(info.format.udp_profile_imu);

    root["lidar_origin_to_beam_origin_mm"] =
        info.lidar_origin_to_beam_origin_mm;

    for (auto i : info.beam_azimuth_angles)
        root["beam_azimuth_angles"].append(i);
    for (auto i : info.beam_altitude_angles)
        root["beam_altitude_angles"].append(i);

    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            root["imu_to_sensor_transform"].append(
                info.imu_to_sensor_transform(i, j));
        }
    }
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            root["lidar_to_sensor_transform"].append(
                info.lidar_to_sensor_transform(i, j));
        }
    }

    root["initialization_id"] = info.init_id;
    root["udp_port_lidar"] = info.udp_port_lidar;
    root["udp_port_imu"] = info.udp_port_imu;

    root["json_calibration_version"] = FW_2_2;

    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";
    return Json::writeString(builder, root);
}

Json::Value to_json(const sensor_config& config) {
    Json::Value root{Json::objectValue};

    if (config.udp_dest) {
        root["udp_dest"] = config.udp_dest.value();
    }

    if (config.udp_port_lidar) {
        root["udp_port_lidar"] = config.udp_port_lidar.value();
    }

    if (config.udp_port_imu) {
        root["udp_port_imu"] = config.udp_port_imu.value();
    }

    if (config.ts_mode) {
        root["timestamp_mode"] = to_string(config.ts_mode.value());
    }

    if (config.ld_mode) {
        root["lidar_mode"] = to_string(config.ld_mode.value());
    }

    if (config.operating_mode) {
        auto mode = config.operating_mode.value();
        root["operating_mode"] = to_string(mode);
    }

    if (config.multipurpose_io_mode) {
        root["multipurpose_io_mode"] =
            to_string(config.multipurpose_io_mode.value());
    }

    if (config.azimuth_window) {
        Json::Value azimuth_window;
        azimuth_window.append(config.azimuth_window.value().first);
        azimuth_window.append(config.azimuth_window.value().second);
        root["azimuth_window"] = azimuth_window;
    }

    if (config.signal_multiplier) {
        root["signal_multiplier"] = config.signal_multiplier.value();
    }

    if (config.sync_pulse_out_angle) {
        root["sync_pulse_out_angle"] = config.sync_pulse_out_angle.value();
    }

    if (config.sync_pulse_out_pulse_width) {
        root["sync_pulse_out_pulse_width"] =
            config.sync_pulse_out_pulse_width.value();
    }

    if (config.nmea_in_polarity) {
        root["nmea_in_polarity"] = to_string(config.nmea_in_polarity.value());
    }

    if (config.nmea_baud_rate) {
        root["nmea_baud_rate"] = to_string(config.nmea_baud_rate.value());
    }

    if (config.nmea_ignore_valid_char) {
        root["nmea_ignore_valid_char"] =
            config.nmea_ignore_valid_char.value() ? 1 : 0;
    }

    if (config.nmea_leap_seconds) {
        root["nmea_leap_seconds"] = config.nmea_leap_seconds.value();
    }

    if (config.sync_pulse_in_polarity) {
        root["sync_pulse_in_polarity"] =
            to_string(config.sync_pulse_in_polarity.value());
    }

    if (config.sync_pulse_out_polarity) {
        root["sync_pulse_out_polarity"] =
            to_string(config.sync_pulse_out_polarity.value());
    }

    if (config.sync_pulse_out_frequency) {
        root["sync_pulse_out_frequency"] =
            config.sync_pulse_out_frequency.value();
    }

    if (config.phase_lock_enable) {
        root["phase_lock_enable"] = config.phase_lock_enable.value();
    }

    if (config.phase_lock_offset) {
        root["phase_lock_offset"] = config.phase_lock_offset.value();
    }

    if (config.columns_per_packet) {
        root["columns_per_packet"] = config.columns_per_packet.value();
    }

    if (config.udp_profile_lidar) {
        root["udp_profile_lidar"] = to_string(config.udp_profile_lidar.value());
    }

    if (config.udp_profile_imu) {
        root["udp_profile_imu"] = to_string(config.udp_profile_imu.value());
    }

    return root;
}

std::string to_string(const sensor_config& config) {
    Json::Value root = to_json(config);

    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 6;
    builder["indentation"] = "    ";
    return Json::writeString(builder, root);
}

}  // namespace sensor

namespace util {

std::string to_string(const version& v) {
    if (v == invalid_version) {
        return "UNKNOWN";
    }

    std::stringstream ss{};
    ss << "v" << v.major << "." << v.minor << "." << v.patch;
    return ss.str();
}

version version_of_string(const std::string& s) {
    std::istringstream is{s};
    char c1, c2, c3;
    version v;

    is >> c1 >> v.major >> c2 >> v.minor >> c3 >> v.patch;

    if (is && c1 == 'v' && c2 == '.' && c3 == '.')
        return v;
    else
        return invalid_version;
}

}  // namespace util
}  // namespace ouster
