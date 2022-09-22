#include "sensor_http.h"

#include <regex>

#include "curl_client.h"
#include "sensor_http_imp.h"
#include "sensor_tcp_imp.h"

using std::stoul;
using std::string;

using namespace ouster::util;
using namespace ouster::sensor;
using namespace ouster::sensor::util;
using namespace ouster::sensor::impl;

string SensorHttp::firmware_version_string(const string& hostname) {
    auto http_client = std::make_unique<CurlClient>("http://" + hostname);
    return http_client->get("api/v1/system/firmware");
}

version SensorHttp::firmware_version(const string& hostname) {
    auto result = firmware_version_string(hostname);
    auto rgx = std::regex(R"(v(\d+).(\d+)\.(\d+))");
    std::smatch matches;
    std::regex_search(result, matches, rgx);

    if (matches.size() < 4) return invalid_version;

    try {
        return version{static_cast<uint16_t>(stoul(matches[1])),
                       static_cast<uint16_t>(stoul(matches[2])),
                       static_cast<uint16_t>(stoul(matches[3]))};
    } catch (const std::exception&) {
        return invalid_version;
    }
}

std::unique_ptr<SensorHttp> SensorHttp::create(const string& hostname) {
    auto fw = firmware_version(hostname);

    if (fw == invalid_version || fw.major < 2) {
        throw std::runtime_error(
            "SensorHttp:: create firmware version information unavailable or "
            "not fully supported version. Please upgrade your sensor to FW "
            "2.0 or later.");
    }

    if (fw.major == 2) {
        switch (fw.minor) {
            case 0:
                // FW 2.0 doesn't work properly with http
                return std::make_unique<SensorTcpImp>(hostname);
            case 1:
                return std::make_unique<SensorHttpImp_2_1>(hostname);
            case 2:
                return std::make_unique<SensorHttpImp_2_2>(hostname);
        }
    }

    return std::make_unique<SensorHttpImp>(hostname);
}
