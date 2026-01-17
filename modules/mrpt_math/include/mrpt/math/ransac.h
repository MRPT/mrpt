/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/COutputLogger.h>

#include <cstddef>
#include <functional>
#include <vector>

namespace mrpt::math
{
/** @addtogroup ransac_grp RANSAC and other model fitting algorithms
 * \ingroup mrpt_math_grp
 * @{ */

/** Define overloaded functions for user types as required.
 * This default implementation assumes datasets in matrices, with each sample
 * being a column, the dimensionality being the number of rows. */
template <typename T>
size_t ransacDatasetSize(const CMatrixDynamic<T>& dataset)
{
  return dataset.cols();
}

/** A generic RANSAC implementation. By default, the input "dataset" and output
 * "model" are matrices, but this can be changed via template arguments to be
 * any user-defined type. Define ransacDatasetSize() for your custom data types.
 *
 * See \a RANSAC_Template::execute for more info on usage, and examples under
 * `[MRPT]/samples/math_ransac_*`.
 *
 * \sa mrpt::math::ModelSearch, another RANSAC implementation where
 * models can be anything else, not only matrices, and capable of genetic
 * algorithms.
 *
 * \note New in MRPT 2.0.2: The second and third template arguments.
 */
template <
    typename NUMTYPE = double,
    typename DATASET = CMatrixDynamic<NUMTYPE>,
    typename MODEL = CMatrixDynamic<NUMTYPE>>
class RANSAC_Template : public mrpt::system::COutputLogger
{
 public:
  RANSAC_Template() : mrpt::system::COutputLogger("RANSAC_Template") {}

  /** The type of the function passed to mrpt::math::ransac - See the
   * documentation for that method for more info. */
  using TRansacFitFunctor = std::function<void(
      const DATASET& allData,
      const std::vector<size_t>& useIndices,
      std::vector<MODEL>& fitModels)>;

  /** The type of the function passed to mrpt::math::ransac  - See the
   * documentation for that method for more info. */
  using TRansacDistanceFunctor = std::function<void(
      const DATASET& allData,
      const std::vector<MODEL>& testModels,
      const NUMTYPE distanceThreshold,
      unsigned int& out_bestModelIndex,
      std::vector<size_t>& out_inlierIndices)>;

  /** The type of the function passed to mrpt::math::ransac  - See the
   * documentation for that method for more info. */
  using TRansacDegenerateFunctor =
      std::function<bool(const DATASET& allData, const std::vector<size_t>& useIndices)>;

  /** An implementation of the RANSAC algorithm for robust fitting of models
   * to data.
   *
   *  \param data A DxN matrix with all the observed data. D is the
   * dimensionality of data points and N the number of points.
   *  \param
   *
   *  This implementation is highly inspired on Peter Kovesi's MATLAB scripts
   * (http://www.csse.uwa.edu.au/~pk).
   * \return false if no good solution can be found, true on success.
   * \note [MRPT 1.5.0] `verbose` parameter has been removed, supersedded by
   * COutputLogger settings.
   */
  bool execute(
      const DATASET& data,
      const TRansacFitFunctor& fit_func,
      const TRansacDistanceFunctor& dist_func,
      const TRansacDegenerateFunctor& degen_func,
      const double distanceThreshold,
      const unsigned int minimumSizeSamplesToFit,
      std::vector<size_t>& out_best_inliers,
      MODEL& out_best_model,
      const double prob_good_sample = 0.999,
      const size_t maxIter = 2000) const;

};  // end class

/** The default instance of RANSAC, for double type */
using RANSAC = RANSAC_Template<double>;

// TEMPLATE IMPLEMENTATION ======================================

template <typename NUMTYPE, typename DATASET, typename MODEL>
bool RANSAC_Template<NUMTYPE, DATASET, MODEL>::execute(
    const DATASET& data,
    const TRansacFitFunctor& fit_func,
    const TRansacDistanceFunctor& dist_func,
    const TRansacDegenerateFunctor& is_degenerated,
    const double distanceThreshold,
    const unsigned int minimumSizeSamplesToFit,
    std::vector<size_t>& out_best_inliers,
    MODEL& out_best_model,
    const double p,
    const size_t maxIter) const
{
  // Highly inspired on http://www.csse.uwa.edu.au/~pk/
  MRPT_START

  ASSERT_GE_(minimumSizeSamplesToFit, 1U);

  const size_t pointCount = ransacDatasetSize(data);

  ASSERT_GT_(pointCount, 1);

  // Maximum number of attempts to select a non-degenerate data set.
  const size_t maxDataTrials = 100;

  // Sentinel value allowing detection of solution failure.
  out_best_model = MODEL();
  out_best_inliers.clear();

  size_t trialcount = 0;
  size_t bestscore = std::string::npos;  // npos will mean "none"
  size_t N = 1;                          // Dummy initialisation for number of trials.

  std::vector<size_t> ind(minimumSizeSamplesToFit);

  while (N > trialcount)
  {
    // Select at random s datapoints to form a trial model, M.
    // In selecting these points we have to check that they are not in
    // a degenerate configuration.
    bool degenerate = true;
    size_t count = 1;
    std::vector<MODEL> MODELS;

    while (degenerate)
    {
      // Generate s random indicies in the range 1..pointCount
      ind.resize(minimumSizeSamplesToFit);

      // The +0.99... is due to the floor rounding afterwards when
      // converting from random double samples to size_t
      mrpt::random::getRandomGenerator().drawUniformVector(
          ind, 0.0, static_cast<double>(pointCount - 1) + 0.999999);

      // Test that these points are not a degenerate configuration.
      degenerate = is_degenerated(data, ind);

      if (!degenerate)
      {
        // Fit model to this random selection of data points.
        // Note that M may represent a set of models that fit the data
        fit_func(data, ind, MODELS);

        // Depending on your problem it might be that the only way you
        // can determine whether a data set is degenerate or not is to
        // try to fit a model and see if it succeeds.  If it fails we
        // reset degenerate to true.
        degenerate = MODELS.empty();
      }

      // Safeguard against being stuck in this loop forever
      if (++count > maxDataTrials)
      {
        MRPT_LOG_WARN("Unable to select a nondegenerate data set");
        break;
      }
    }

    // Once we are out here we should have some kind of model...
    // Evaluate distances between points and model returning the indices
    // of elements in x that are inliers.  Additionally, if M is a cell
    // array of possible models 'distance' will return the model that has
    // the most inliers.
    unsigned int bestModelIdx = std::numeric_limits<unsigned int>::max();
    std::vector<size_t> inliers;
    if (!degenerate)
    {
      dist_func(data, MODELS, static_cast<NUMTYPE>(distanceThreshold), bestModelIdx, inliers);
      ASSERT_LT_(bestModelIdx, MODELS.size());
    }

    // Find the number of inliers to this model.
    const size_t nInliers = inliers.size();
    // Always update on the first iteration, regardless of the result (even
    // for nInliers=0)
    bool update_estimated_iters = (trialcount == 0);

    if (nInliers > bestscore || (bestscore == std::string::npos && nInliers != 0))
    {
      bestscore = nInliers;  // Record data for this model

      out_best_model = std::move(MODELS[bestModelIdx]);
      out_best_inliers = std::move(inliers);
      update_estimated_iters = true;
    }

    if (update_estimated_iters)
    {
      // Update estimate of N, the number of trials to ensure we pick,
      // with probability p, a data set with no outliers.
      double fracInliers = nInliers / static_cast<double>(pointCount);
      double pNoOutliers = 1 - pow(fracInliers, static_cast<double>(minimumSizeSamplesToFit));

      pNoOutliers = std::max(
          std::numeric_limits<double>::epsilon(),
          pNoOutliers);  // Avoid division by -Inf
      pNoOutliers = std::min(
          1.0 - std::numeric_limits<double>::epsilon(),
          pNoOutliers);  // Avoid division by 0.
      // Number of
      N = static_cast<size_t>(log(1 - p) / log(pNoOutliers));
      MRPT_LOG_DEBUG_FMT(
          "Iter #%u Estimated number of iters: %u  pNoOutliers = %f  "
          "#inliers: %u",
          static_cast<unsigned>(trialcount), static_cast<unsigned>(N), pNoOutliers,
          static_cast<unsigned>(nInliers));
    }

    ++trialcount;

    MRPT_LOG_DEBUG_FMT(
        "trial %u out of %u", static_cast<unsigned int>(trialcount),
        static_cast<unsigned int>(ceil(static_cast<double>(N))));

    // Safeguard against being stuck in this loop forever
    if (trialcount > maxIter)
    {
      MRPT_LOG_WARN_FMT("Warning: maximum number of trials (%u) reached\n", (unsigned)maxIter);
      break;
    }
  }

  if (!out_best_inliers.empty())
  {  // We got a solution
    MRPT_LOG_INFO_FMT("Finished in %u iterations.", (unsigned)trialcount);
    return true;
  }

  MRPT_LOG_WARN("Finished without any proper solution");
  return false;

  MRPT_END
}

/** @} */

}  // namespace mrpt::math
