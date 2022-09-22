/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Simple version struct
 */

#include <cstdint>
#include <string>

#pragma once

namespace ouster {
namespace util {

struct version {
    uint16_t major;  ///< Major version number
    uint16_t minor;  ///< Minor version number
    uint16_t patch;  ///< Patch(or revision) version number
};

const version invalid_version = {0, 0, 0};

/** \defgroup ouster_client_version_operators Ouster Client version.h Operators
 * @{
 */
/**
 * Equality operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the versions are the same.
 */
inline bool operator==(const version& u, const version& v) {
    return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

/**
 * Less than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is less than the second version.
 */
inline bool operator<(const version& u, const version& v) {
    return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
           (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

/**
 * Less than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is less than or equal to the second version.
 */
inline bool operator<=(const version& u, const version& v) {
    return u < v || u == v;
}

/**
 * In-equality operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the versions are not the same.
 */
inline bool operator!=(const version& u, const version& v) { return !(u == v); }

/**
 * Greater than or equal to operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than or equal to the second version.
 */
inline bool operator>=(const version& u, const version& v) { return !(u < v); }

/**
 * Greater than operation for version structs.
 *
 * @param[in] u The first version to compare.
 * @param[in] v The second version to compare.
 *
 * @return If the first version is greater than the second version.
 */
inline bool operator>(const version& u, const version& v) { return !(u <= v); }
/** @}*/

/**
 * Get string representation of a version.
 *
 * @param[in] v version.
 *
 * @return string representation of the version.
 */
std::string to_string(const version& v);

/**
 * Get version from string.
 *
 * @param[in] s string.
 *
 * @return version corresponding to the string, or invalid_version on error.
 */
version version_of_string(const std::string& s);

}  // namespace util
}  // namespace ouster
