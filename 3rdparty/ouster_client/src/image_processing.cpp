/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/image_processing.h"

#include <Eigen/src/Core/util/Meta.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <iostream>
#include <vector>

namespace ouster {
namespace viz {

namespace {

/*
 * damping makes the autoexposure smooth and avoids flickering however, it
 * becomes slower to update.
 * 1.0 --> slowest, smoothest
 * 0.0 --> fastest, prone to flickering
 */
const double ae_damping = 0.90;

/*
 * for performance reasons, we may not want to update every frame but rather
 * every few frames
 */
const int ae_default_update_every = 3;

/* for performance reasons, only consider a subset of points */
const size_t ae_stride = 4;

/* if there are too few points, do nothing */
const size_t ae_min_nonzero_points = 100;

/* default percentile for scaling in autoexposure */
const double ae_default_percentile = 0.1;

}  // namespace

AutoExposure::AutoExposure()
    : lo_percentile(ae_default_percentile),
      hi_percentile(ae_default_percentile),
      ae_update_every(ae_default_update_every) {}

AutoExposure::AutoExposure(int update_every)
    : lo_percentile(ae_default_percentile),
      hi_percentile(ae_default_percentile),
      ae_update_every(update_every) {}

AutoExposure::AutoExposure(double lo_percentile, double hi_percentile,
                           int update_every)
    : lo_percentile(lo_percentile),
      hi_percentile(hi_percentile),
      ae_update_every(update_every) {}

template <typename T>
void AutoExposure::update(Eigen::Ref<img_t<T>> image, bool update_state) {
    Eigen::Map<Eigen::Array<T, -1, 1>> key_eigen(image.data(), image.size());

    // int a;
    if (counter == 0 && update_state) {
        const size_t n = key_eigen.rows();
        std::vector<size_t> indices;
        indices.reserve(n);
        for (size_t i = 0; i < n; i += ae_stride) {
            // ignore 0 values, which are often due to dropped packets etc
            if (key_eigen[i] > 0) {
                indices.push_back(i);
            }
        }
        if (indices.size() < ae_min_nonzero_points) {
            // too few nonzero values, nothing to do
            return;
        }
        auto cmp = [&](const size_t a, const size_t b) {
            return key_eigen(a) < key_eigen(b);
        };

        const size_t lo_kth_extreme =
            static_cast<size_t>(indices.size() * lo_percentile);
        std::nth_element(indices.begin(), indices.begin() + lo_kth_extreme,
                         indices.end(), cmp);
        lo = key_eigen[*(indices.begin() + lo_kth_extreme)];

        const size_t hi_kth_extreme =
            static_cast<size_t>(indices.size() * hi_percentile);
        std::nth_element(indices.begin() + lo_kth_extreme,
                         indices.end() - hi_kth_extreme - 1, indices.end(),
                         cmp);
        hi = key_eigen[*(indices.end() - hi_kth_extreme - 1)];

        if (!initialized) {
            initialized = true;
            lo_state = lo;
            hi_state = hi;
        }
    }
    if (!initialized) {
        return;
    }

    // we use the simplest form of exponential smoothing
    if (update_state) {
        lo_state = ae_damping * lo_state + (1.0 - ae_damping) * lo;
        hi_state = ae_damping * hi_state + (1.0 - ae_damping) * hi;
    }

    // Apply affine transformation mapping lo_state to lo_percentile and
    // hi_state to 1 - hi_percentile. If it would map 0 to positive number,
    // instead map using only hi_state
    double lo_hi_scale =
        (1.0 - (lo_percentile + hi_percentile)) / (hi_state - lo_state);

    if (std::isinf(lo_hi_scale) || std::isnan(lo_hi_scale)) {
        // map everything relative to hi_state being 0.5 due to small spread or
        // nan
        key_eigen *= 0.5 / hi_state;
    } else if (lo_hi_scale * (0.0 - lo_state) + lo_percentile <= 0.00) {
        // apply affine transformation
        key_eigen -= lo_state;
        key_eigen *= lo_hi_scale;
        key_eigen += lo_percentile;
    } else {
        // lo_hi_state transformation would map 0 to positive number
        // instead, map using only hi_state
        key_eigen *= (1.0 - hi_percentile) / (hi_state);
    }

    // clamp
    key_eigen = key_eigen.max(0.0).min(1.0);

    if (update_state) {
        counter = (counter + 1) % ae_update_every;
    }
}

// use overloads vs templates so implicit conversion to Eigen::Ref still works
void AutoExposure::operator()(Eigen::Ref<img_t<float>> image,
                              bool update_state) {
    update(image, update_state);
}

void AutoExposure::operator()(Eigen::Ref<img_t<double>> image,
                              bool update_state) {
    update(image, update_state);
}

namespace {

/*
 * damping makes the correction smooth and avoids flickering.
 * however, it becomes slower to update.
 * 1.0 --> slowest, smoothest
 * 0.0 --> fastest, prone to flickering
 */
const double buc_damping = 0.92;

/*
 * for performance reasons, we may not want to update every frame
 * but rather every 8 or so frames.
 */
const int buc_update_every = 8;

}  // namespace

/*
 * computes the dark count, i.e. an additive offset in the brightness of the
 * image, to smoothe the difference between rows
 */
template <typename T>
static Eigen::Array<T, -1, 1> compute_dark_count(
    const Eigen::Ref<img_t<T>>& image) {
    const size_t image_h = image.rows();
    const size_t image_w = image.cols();

    Eigen::Array<T, -1, 1> tmp{image_w};
    Eigen::Array<T, -1, 1> new_dark_count{image_h};

    // probably computed lazily when used below?
    auto row_diffs = image.bottomRows(image_h - 1) - image.topRows(image_h - 1);

    // to handle azimuth-masked data, only consider columns with nonzero values
    Eigen::Array<bool, -1, 1> col_mask =
        image.template cast<bool>().colwise().any();
    const size_t n_cols = col_mask.count();

    img_t<T> row_diffs_nonzero{image_h - 1, n_cols};
    for (size_t i = 0, j = 0; i < image_w && j < n_cols; i++) {
        if (col_mask[i]) {
            row_diffs_nonzero.col(j) = row_diffs.col(i);
            j++;
        }
    }

    // compute the median of differences between rows
    for (size_t i = 1; i < image_h; i++) {
        tmp = row_diffs_nonzero.row(i - 1);
        std::nth_element(tmp.data(), tmp.data() + n_cols / 2,
                         tmp.data() + n_cols);
        new_dark_count[i] = new_dark_count[i - 1] + tmp[n_cols / 2];
    }

    // remove gradients in the entire height of image by doing linear fit
    Eigen::Matrix<T, -1, 2> A(image_h, 2);
    for (size_t i = 0; i < image_h; i++) {
        A(i, 0) = 1;
        A(i, 1) = static_cast<T>(i);
    }
    Eigen::Matrix<T, 2, 1> x = A.fullPivLu().solve(new_dark_count.matrix());
    new_dark_count -= (A * x).array();

    // subtract minimum value
    new_dark_count -= new_dark_count.minCoeff();
    return new_dark_count;
}

template <typename T>
void BeamUniformityCorrector::update(Eigen::Ref<img_t<T>> image,
                                     bool update_state) {
    const auto image_h = image.rows();

    // compute dark counts, if necessary
    if (dark_count.size() != image_h) {
        dark_count = compute_dark_count(image).template cast<double>();
    } else if (update_state && counter == 0) {
        // if previous state exists, update using exponential smoothing:
        Eigen::ArrayXd new_dark_count =
            compute_dark_count(image).template cast<double>();
        dark_count *= buc_damping;
        dark_count += new_dark_count * (1.0 - buc_damping);
    }
    counter = (counter + 1) % buc_update_every;

    // apply the dark count correction
    image.colwise() -= dark_count.cast<T>();

    // clamp any negative values
    image = image.cwiseMax(0.0);
}

void BeamUniformityCorrector::operator()(Eigen::Ref<img_t<float>> image,
                                         bool update_state) {
    update(image, update_state);
}

void BeamUniformityCorrector::operator()(Eigen::Ref<img_t<double>> image,
                                         bool update_state) {
    update(image, update_state);
}

}  // namespace viz
}  // namespace ouster
