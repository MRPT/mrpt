/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Utilities for post-processing image data
 */

#pragma once

#include <Eigen/Core>

#include "ouster/types.h"

namespace ouster {
namespace viz {

/** Adjusts brightness to between 0 and 1. */
class AutoExposure {
    const double lo_percentile, hi_percentile;  // percentiles used for scaling
    const int ae_update_every;

    double lo_state = -1.0;
    double hi_state = -1.0;
    double lo = -1.0;
    double hi = -1.0;

    bool initialized = false;
    int counter = 0;

    template <typename T>
    void update(Eigen::Ref<img_t<T>> image, bool update_state);

   public:
    /** Default constructor using default percentile and update values. */
    AutoExposure();

    /**
     * Constructor specifying update modulo, and using default percentiles.
     *
     * @param[in] update_every update every this number of frames.
     */
    AutoExposure(int update_every);

    /**
     * Constructor specifying low and high percentiles, and update modulo.
     *
     * @param[in] lo_percentile low percentile to use for adjustment.
     * @param[in] hi_percentile high percentile to use for adjustment.
     * @param[in] update_every update every this number of frames.
     */
    AutoExposure(double lo_percentile, double hi_percentile, int update_every);

    /**
     * Scales the image so that contrast is stretched between 0 and 1.
     *
     * The top percentile is 1 - hi_percentile and the bottom percentile is
     * lo_percentile. Similar to linear 'contrast-stretch', i.e. normalization.
     *
     * @param[in] image Reference to the image, modified in place.
     * @param[in] update_state Update lo/hi percentiles if true.
     */
    void operator()(Eigen::Ref<img_t<float>> image, bool update_state = true);

    /**
     * Scales the image so that contrast is stretched between 0 and 1.
     *
     * The top percentile is 1 - hi_percentile and the bottom percentile is
     * lo_percentile. Similar to linear 'contrast-stretch', i.e. normalization.
     *
     * @param[in] image Reference to the image, modified in place.
     * @param[in] update_state Update lo/hi percentiles if true.
     */
    void operator()(Eigen::Ref<img_t<double>> image, bool update_state = true);
};

/**
 * Corrects beam uniformity by minimizing median difference between rows,
 * thereby correcting subtle horizontal line artifacts in images, especially the
 * ambient image.
 */
class BeamUniformityCorrector {
   private:
    int counter = 0;
    Eigen::ArrayXd dark_count;

    template <typename T>
    void update(Eigen::Ref<img_t<T>> image, bool update_state);

   public:
    /**
     * Applies dark count correction to an image, modifying it in-place to have
     * reduced horizontal line artifacts.
     *
     * @param[in] image Reference to the image, modified in-place.
     * @param[in] update_state Update dark counts if true.
     */
    void operator()(Eigen::Ref<img_t<float>> image, bool update_state = true);

    /**
     * Applies dark count correction to an image, modifying it in-place to have
     * reduced horizontal line artifacts.
     *
     * @param[in] image Reference to the image, modified in-place.
     * @param[in] update_state Update dark counts if true.
     */
    void operator()(Eigen::Ref<img_t<double>> image, bool update_state = true);
};
}  // namespace viz
}  // namespace ouster
