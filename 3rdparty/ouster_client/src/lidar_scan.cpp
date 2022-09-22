/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/lidar_scan.h"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>
#include <vector>

#include "ouster/impl/lidar_scan_impl.h"
#include "ouster/types.h"

namespace ouster {

using sensor::ChanField;
using sensor::ChanFieldType;
using sensor::UDPProfileLidar;

constexpr int LidarScan::N_FIELDS;

LidarScan::LidarScan() = default;
LidarScan::LidarScan(const LidarScan&) = default;
LidarScan::LidarScan(LidarScan&&) = default;
LidarScan& LidarScan::operator=(const LidarScan&) = default;
LidarScan& LidarScan::operator=(LidarScan&&) = default;
LidarScan::~LidarScan() = default;

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

static const Table<ChanField, ChanFieldType, 4> legacy_field_slots{
    {{ChanField::RANGE, ChanFieldType::UINT32},
     {ChanField::SIGNAL, ChanFieldType::UINT32},
     {ChanField::NEAR_IR, ChanFieldType::UINT32},
     {ChanField::REFLECTIVITY, ChanFieldType::UINT32}}};

static const Table<ChanField, ChanFieldType, 7> dual_field_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::RANGE2, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::SIGNAL2, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT8},
    {ChanField::REFLECTIVITY2, ChanFieldType::UINT8},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<ChanField, ChanFieldType, 4> single_field_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::SIGNAL, ChanFieldType::UINT16},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT16},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

static const Table<ChanField, ChanFieldType, 3> lb_field_slots{{
    {ChanField::RANGE, ChanFieldType::UINT32},
    {ChanField::REFLECTIVITY, ChanFieldType::UINT16},
    {ChanField::NEAR_IR, ChanFieldType::UINT16},
}};

struct DefaultFieldsEntry {
    const std::pair<ChanField, ChanFieldType>* fields;
    size_t n_fields;
};

Table<UDPProfileLidar, DefaultFieldsEntry, 32> default_scan_fields{
    {{UDPProfileLidar::PROFILE_LIDAR_LEGACY,
      {legacy_field_slots.data(), legacy_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
      {dual_field_slots.data(), dual_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16,
      {single_field_slots.data(), single_field_slots.size()}},
     {UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8,
      {lb_field_slots.data(), lb_field_slots.size()}}}};

static std::vector<std::pair<ChanField, ChanFieldType>> lookup_scan_fields(
    UDPProfileLidar profile) {
    auto end = impl::default_scan_fields.end();
    auto it =
        std::find_if(impl::default_scan_fields.begin(), end,
                     [profile](const auto& kv) { return kv.first == profile; });

    if (it == end || it->first == 0)
        throw std::invalid_argument("Unknown lidar udp profile");

    auto entry = it->second;
    return {entry.fields, entry.fields + entry.n_fields};
}

}  // namespace impl

// specify sensor:: namespace for doxygen matching
LidarScan::LidarScan(
    size_t w, size_t h,
    std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>
        field_types)
    : timestamp_{Header<uint64_t>::Zero(w)},
      measurement_id_{Header<uint16_t>::Zero(w)},
      status_{Header<uint32_t>::Zero(w)},
      field_types_{std::move(field_types)},
      w{static_cast<std::ptrdiff_t>(w)},
      h{static_cast<std::ptrdiff_t>(h)},
      headers{w, BlockHeader{ts_t{0}, 0, 0}} {
    // TODO: error on duplicate fields
    for (const auto& ft : field_types_) {
        if (fields_.count(ft.first) > 0)
            throw std::invalid_argument("Duplicated fields found");
        fields_[ft.first] = impl::FieldSlot{ft.second, w, h};
    }
}

LidarScan::LidarScan(size_t w, size_t h, sensor::UDPProfileLidar profile)
    : LidarScan{w, h, impl::lookup_scan_fields(profile)} {}

LidarScan::LidarScan(size_t w, size_t h)
    : LidarScan{w, h, UDPProfileLidar::PROFILE_LIDAR_LEGACY} {}

std::vector<LidarScan::ts_t> LidarScan::timestamps() const {
    std::vector<LidarScan::ts_t> res;
    res.reserve(headers.size());
    for (const auto& h : headers) res.push_back(h.timestamp);
    return res;
}

LidarScan::BlockHeader& LidarScan::header(size_t m_id) {
    return headers.at(m_id);
}

const LidarScan::BlockHeader& LidarScan::header(size_t m_id) const {
    return headers.at(m_id);
}

template <typename T,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
Eigen::Ref<img_t<T>> LidarScan::field(ChanField f) {
    return fields_.at(f).get<T>();
}

template <typename T,
          typename std::enable_if<std::is_unsigned<T>::value, T>::type>
Eigen::Ref<const img_t<T>> LidarScan::field(ChanField f) const {
    return fields_.at(f).get<T>();
}

// explicitly instantiate for each supported field type
template Eigen::Ref<img_t<uint8_t>> LidarScan::field(ChanField f);
template Eigen::Ref<img_t<uint16_t>> LidarScan::field(ChanField f);
template Eigen::Ref<img_t<uint32_t>> LidarScan::field(ChanField f);
template Eigen::Ref<img_t<uint64_t>> LidarScan::field(ChanField f);
template Eigen::Ref<const img_t<uint8_t>> LidarScan::field(ChanField f) const;
template Eigen::Ref<const img_t<uint16_t>> LidarScan::field(ChanField f) const;
template Eigen::Ref<const img_t<uint32_t>> LidarScan::field(ChanField f) const;
template Eigen::Ref<const img_t<uint64_t>> LidarScan::field(ChanField f) const;

ChanFieldType LidarScan::field_type(ChanField f) const {
    return fields_.count(f) ? fields_.at(f).tag : ChanFieldType::VOID;
}

LidarScan::FieldIter LidarScan::begin() const { return field_types_.cbegin(); }

LidarScan::FieldIter LidarScan::end() const { return field_types_.cend(); }

Eigen::Ref<LidarScan::Header<uint64_t>> LidarScan::timestamp() {
    return timestamp_;
}
Eigen::Ref<const LidarScan::Header<uint64_t>> LidarScan::timestamp() const {
    return timestamp_;
}

Eigen::Ref<LidarScan::Header<uint16_t>> LidarScan::measurement_id() {
    return measurement_id_;
}
Eigen::Ref<const LidarScan::Header<uint16_t>> LidarScan::measurement_id()
    const {
    return measurement_id_;
}

Eigen::Ref<LidarScan::Header<uint32_t>> LidarScan::status() { return status_; }
Eigen::Ref<const LidarScan::Header<uint32_t>> LidarScan::status() const {
    return status_;
}

bool LidarScan::complete(sensor::ColumnWindow window) const {
    const auto& status = this->status();
    auto start = window.first;
    auto end = window.second;

    if (start <= end) {
        return status.segment(start, end - start + 1)
            .unaryExpr([](uint32_t s) { return s & 0x01; })
            .isConstant(0x01);
    } else {
        return status.segment(0, end)
                   .unaryExpr([](uint32_t s) { return s & 0x01; })
                   .isConstant(0x01) &&
               status.segment(start, this->w - start)
                   .unaryExpr([](uint32_t s) { return s & 0x01; })
                   .isConstant(0x01);
    }
}

bool operator==(const LidarScan::BlockHeader& a,
                const LidarScan::BlockHeader& b) {
    return a.timestamp == b.timestamp && a.encoder == b.encoder &&
           a.status == b.status;
}

bool operator==(const LidarScan& a, const LidarScan& b) {
    return a.frame_id == b.frame_id && a.w == b.w && a.h == b.h &&
           a.fields_ == b.fields_ && a.field_types_ == b.field_types_ &&
           (a.timestamp() == b.timestamp()).all() &&
           (a.measurement_id() == b.measurement_id()).all() &&
           (a.status() == b.status()).all();
}

XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg) {
    if (w <= 0 || h <= 0)
        throw std::invalid_argument("lut dimensions must be greater than zero");
    if (azimuth_angles_deg.size() != h || altitude_angles_deg.size() != h)
        throw std::invalid_argument("unexpected scan dimensions");

    Eigen::ArrayXd encoder(w * h);   // theta_e
    Eigen::ArrayXd azimuth(w * h);   // theta_a
    Eigen::ArrayXd altitude(w * h);  // phi

    const double azimuth_radians = M_PI * 2.0 / w;

    // populate angles for each pixel
    for (size_t v = 0; v < w; v++) {
        for (size_t u = 0; u < h; u++) {
            size_t i = u * w + v;
            encoder(i) = 2.0 * M_PI - (v * azimuth_radians);
            azimuth(i) = -azimuth_angles_deg[u] * M_PI / 180.0;
            altitude(i) = altitude_angles_deg[u] * M_PI / 180.0;
        }
    }

    XYZLut lut;

    // unit vectors for each pixel
    lut.direction = LidarScan::Points{w * h, 3};
    lut.direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
    lut.direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
    lut.direction.col(2) = altitude.sin();

    // offsets due to beam origin
    lut.offset = LidarScan::Points{w * h, 3};
    lut.offset.col(0) = encoder.cos() - lut.direction.col(0);
    lut.offset.col(1) = encoder.sin() - lut.direction.col(1);
    lut.offset.col(2) = -lut.direction.col(2);
    lut.offset *= lidar_origin_to_beam_origin_mm;

    // apply the supplied transform
    auto rot = transform.topLeftCorner(3, 3).transpose();
    auto trans = transform.topRightCorner(3, 1).transpose();
    lut.direction.matrix() *= rot;
    lut.offset.matrix() *= rot;
    lut.offset.matrix() += trans.replicate(w * h, 1);

    // apply scaling factor
    lut.direction *= range_unit;
    lut.offset *= range_unit;

    return lut;
}

LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut) {
    return cartesian(scan.field(ChanField::RANGE), lut);
}

LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                            const XYZLut& lut) {
    if (range.cols() * range.rows() != lut.direction.rows())
        throw std::invalid_argument("unexpected image dimensions");

    auto reshaped = Eigen::Map<const Eigen::Array<uint32_t, -1, 1>>(
        range.data(), range.cols() * range.rows());
    auto nooffset = lut.direction.colwise() * reshaped.cast<double>();
    return (nooffset.array() == 0.0).select(nooffset, nooffset + lut.offset);
}

ScanBatcher::ScanBatcher(size_t w, const sensor::packet_format& pf)
    : w(w),
      h(pf.pixels_per_column),
      next_m_id(0),
      cache(pf.lidar_packet_size),
      pf(pf) {}

ScanBatcher::ScanBatcher(const sensor::sensor_info& info)
    : ScanBatcher(info.format.columns_per_frame, sensor::get_format(info)) {}

namespace {

/*
 * Generic operation to set all columns in the range [start, end) to zero
 */
struct zero_field_cols {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField, std::ptrdiff_t start,
                    std::ptrdiff_t end) {
        field.block(0, start, field.rows(), end - start).setZero();
    }
};

/*
 * Zero out all measurement block headers in range [start, end)
 */
void zero_header_cols(LidarScan& ls, std::ptrdiff_t start, std::ptrdiff_t end) {
    ls.timestamp().segment(start, end - start).setZero();
    ls.measurement_id().segment(start, end - start).setZero();
    ls.status().segment(start, end - start).setZero();

    // zero deprecated header blocks
    for (auto m_id = start; m_id < end; m_id++) ls.header(m_id) = {};
}

/*
 * Generic operation to read a channel field from a packet measurement block
 * into a scan
 */
struct parse_field_col {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field, ChanField f, uint16_t m_id,
                    const sensor::packet_format& pf, const uint8_t* col_buf) {
        if (f >= ChanField::CUSTOM0 && f <= ChanField::CUSTOM9) return;
        pf.col_field(col_buf, f, field.col(m_id).data(), field.cols());
    }
};

}  // namespace

bool ScanBatcher::operator()(const uint8_t* packet_buf, LidarScan& ls) {
    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");

    // process cached packet
    if (cached_packet) {
        cached_packet = false;
        ls.frame_id = -1;
        this->operator()(cache.data(), ls);
    }

    const uint16_t f_id = pf.frame_id(packet_buf);

    if (ls.frame_id == -1) {
        // expecting to start batching a new scan
        next_m_id = 0;
        ls.frame_id = f_id;
    } else if (ls.frame_id == f_id + 1) {
        // drop reordered packets from the previous frame
        return false;
    } else if (ls.frame_id != f_id) {
        // got a packet from a new frame
        impl::foreach_field(ls, zero_field_cols(), next_m_id, w);
        zero_header_cols(ls, next_m_id, w);
        std::memcpy(cache.data(), packet_buf, cache.size());
        cached_packet = true;
        return true;
    }

    // parse measurement blocks
    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
        const uint16_t m_id = pf.col_measurement_id(col_buf);
        const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
        const uint32_t encoder = pf.col_encoder(col_buf);
        const uint32_t status = pf.col_status(col_buf);
        const bool valid = (status & 0x01);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (!valid || m_id >= w) continue;

        // zero out missing columns if we jumped forward
        if (m_id >= next_m_id) {
            impl::foreach_field(ls, zero_field_cols(), next_m_id, m_id);
            zero_header_cols(ls, next_m_id, m_id);
            next_m_id = m_id + 1;
        }

        // old header API; will be removed in a future release
        ls.header(m_id) = {ts, encoder, status};

        // write new header values
        ls.timestamp()[m_id] = ts.count();
        ls.measurement_id()[m_id] = m_id;
        ls.status()[m_id] = status;

        impl::foreach_field(ls, parse_field_col(), m_id, pf, col_buf);
    }
    return false;
}

}  // namespace ouster
