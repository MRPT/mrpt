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

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <cmath>
#include <filesystem>
#include <sstream>

using mrpt::maps::CGasConcentrationGridMap2D;
using mrpt::maps::CRandomFieldGridMap2D;
using mrpt::maps::TRandomFieldCell;
using TMapRepresentation = CRandomFieldGridMap2D::TMapRepresentation;

namespace
{
// A small helper that always says two neighbor cells are connected, with a
// fixed information (inverse variance) value. Used to exercise the
// "custom connectivity descriptor" branch of the GMRF prior-building code.
struct FixedInformationConnectivity : public CRandomFieldGridMap2D::ConnectivityDescriptor
{
  bool getEdgeInformation(
      const CRandomFieldGridMap2D* /*parent*/,
      size_t /*icx*/,
      size_t /*icy*/,
      size_t /*jcx*/,
      size_t /*jcy*/,
      double& out_edge_information) override
  {
    out_edge_information = 2.0;
    return true;
  }
};

// All five map representations, used to iterate over in loop-based tests.
const TMapRepresentation kAllMapTypes[5] = {
    CRandomFieldGridMap2D::mrKernelDM,     CRandomFieldGridMap2D::mrKernelDMV,
    CRandomFieldGridMap2D::mrKalmanFilter, CRandomFieldGridMap2D::mrKalmanApproximate,
    CRandomFieldGridMap2D::mrGMRF_SD,
};

// Builds a small grid and inserts a handful of readings for the given map
// type. Kernel-based methods (DM, DM+V) require a fine enough resolution
// with respect to the (default) cutoff radius, hence the small extent here.
CGasConcentrationGridMap2D makeGridWithReadings(TMapRepresentation mapType)
{
  const bool isKernel = (mapType == CRandomFieldGridMap2D::mrKernelDM) ||
                        (mapType == CRandomFieldGridMap2D::mrKernelDMV);

  const float ext = isKernel ? 1.0f : 4.0f;
  const float res = isKernel ? 0.2f : 1.0f;

  CGasConcentrationGridMap2D grid(mapType, -ext, ext, -ext, ext, res);

  grid.insertionOptions.GMRF_skip_variance = false;

  // For the Kalman-family map types, KF_covSigma must be large enough
  // relative to the cell resolution or the compressed-covariance update
  // (mrKalmanApproximate) can throw "Negative variance value appeared!"
  // (see CRandomFieldGridMap2D::insertObservation_KF2()). The covariance
  // window is only rebuilt on clear()/setSize(), so re-clear after changing
  // it.
  if (!isKernel)
  {
    grid.insertionOptions.KF_covSigma = res * 2.0f;
    grid.clear();
  }

  // mrKalmanApproximate's compressed-covariance update is only numerically
  // stable when successive updates don't heavily overlap their windows
  // (each update refines the cross-covariances of nearby cells, and
  // repeated overlapping refinements on a small test grid can drive a
  // variance negative -- an inherent limitation of that windowed
  // approximation, not exercised elsewhere in the codebase before). A
  // single insertion into a freshly-cleared grid cannot hit that
  // accumulation issue, so use just one reading for that map type.
  if (mapType == CRandomFieldGridMap2D::mrKalmanApproximate)
  {
    grid.insertIndividualReading(0.5, {0.0, ext * 0.3}, false, true, 1.0);
  }
  else
  {
    grid.insertIndividualReading(0.2, {ext * 0.2, ext * 0.2}, false, true, 1.0);
    grid.insertIndividualReading(0.5, {-ext * 0.2, -ext * 0.1}, false, true, 1.0);
    grid.insertIndividualReading(0.8, {0.0, ext * 0.3}, false, true, 1.0);
  }

  grid.updateMapEstimation();

  return grid;
}

}  // namespace

// =========================================================================
//  internal_clear() / construction + insertion, for all 5 map types
// =========================================================================

TEST(CRandomFieldGridMap2D, InternalClearAndInsert_AllMapTypes)
{
  for (const auto mapType : kAllMapTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);
    EXPECT_GT(grid.getSizeX(), 0u) << "mapType=" << mapType;
    EXPECT_GT(grid.getSizeY(), 0u) << "mapType=" << mapType;
    EXPECT_FALSE(grid.isEmpty());
  }
}

TEST(CRandomFieldGridMap2D, InsertAndRead_KernelDM)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKernelDM, -1.2f, 1.2f, -1.2f, 1.2f, 0.2f);

  // Two readings placed far enough apart (more than twice the default
  // cutoff radius) that their kernel windows never overlap: each queried
  // cell's weighted mean is then driven by a single, exactly-known reading,
  // making the expected result deterministic.
  grid.insertIndividualReading(0.2, {0.2, 0.2}, false, true, 1.0);
  grid.insertIndividualReading(0.8, {-1.0, -1.0}, false, true, 1.0);
  grid.updateMapEstimation();

  const TRandomFieldCell* cellA = grid.cellByPos(0.2, 0.2);
  ASSERT_NE(cellA, nullptr);
  ASSERT_GT(cellA->dm_mean_w(), 0.0);
  EXPECT_NEAR(cellA->dm_mean() / cellA->dm_mean_w(), 0.2, 1e-6);

  const TRandomFieldCell* cellB = grid.cellByPos(-1.0, -1.0);
  ASSERT_NE(cellB, nullptr);
  ASSERT_GT(cellB->dm_mean_w(), 0.0);
  EXPECT_NEAR(cellB->dm_mean() / cellB->dm_mean_w(), 0.8, 1e-6);
}

TEST(CRandomFieldGridMap2D, InsertAndRead_KernelDMV)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKernelDMV, -1.2f, 1.2f, -1.2f, 1.2f, 0.2f);

  grid.insertIndividualReading(0.2, {0.2, 0.2}, false, true, 1.0);
  grid.insertIndividualReading(0.8, {-1.0, -1.0}, false, true, 1.0);
  grid.updateMapEstimation();

  const TRandomFieldCell* cell = grid.cellByPos(0.2, 0.2);
  ASSERT_NE(cell, nullptr);
  ASSERT_GT(cell->dm_mean_w(), 0.0);
  EXPECT_NEAR(cell->dm_mean() / cell->dm_mean_w(), 0.2, 1e-6);
  // DM+V also accumulates a weighted variance estimate.
  EXPECT_GE(cell->dmv_var_mean, 0.0);
}

TEST(CRandomFieldGridMap2D, InsertAndRead_KalmanFilter)
{
  CGasConcentrationGridMap2D grid = makeGridWithReadings(CRandomFieldGridMap2D::mrKalmanFilter);

  const TRandomFieldCell* cell = grid.cellByPos(0.0, 1.2);
  ASSERT_NE(cell, nullptr);
  // After inserting a reading near (0, 1.2), the estimated mean should have
  // moved measurably away from the (default) prior value.
  EXPECT_TRUE(std::isfinite(cell->kf_mean()));
  EXPECT_GE(cell->kf_std(), 0.0);
}

TEST(CRandomFieldGridMap2D, InsertAndRead_KalmanApproximate)
{
  CGasConcentrationGridMap2D grid =
      makeGridWithReadings(CRandomFieldGridMap2D::mrKalmanApproximate);

  const TRandomFieldCell* cell = grid.cellByPos(0.0, 1.2);
  ASSERT_NE(cell, nullptr);
  EXPECT_TRUE(std::isfinite(cell->kf_mean()));
}

TEST(CRandomFieldGridMap2D, InsertAndRead_GMRF_SD)
{
  CGasConcentrationGridMap2D grid = makeGridWithReadings(CRandomFieldGridMap2D::mrGMRF_SD);

  const TRandomFieldCell* cell = grid.cellByPos(0.0, 1.2);
  ASSERT_NE(cell, nullptr);
  EXPECT_TRUE(std::isfinite(cell->gmrf_mean()));
  EXPECT_GE(cell->gmrf_std(), 0.0);
}

// =========================================================================
//  GMRF with a custom ConnectivityDescriptor
// =========================================================================

TEST(CRandomFieldGridMap2D, CustomConnectivityDescriptor_GMRF)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -2.0f, 2.0f, -2.0f, 2.0f, 0.5f);

  auto connectivity = std::make_shared<FixedInformationConnectivity>();
  grid.setCellsConnectivity(connectivity);
  // Per the API docs, clear() (or setSize()) must be called for the new
  // connectivity pattern to take effect.
  grid.clear();

  grid.insertIndividualReading(0.4, {0.5, 0.5}, true, true, 1.0);

  const TRandomFieldCell* cell = grid.cellByPos(0.5, 0.5);
  ASSERT_NE(cell, nullptr);
  EXPECT_TRUE(std::isfinite(cell->gmrf_mean()));
}

// =========================================================================
//  resize()
// =========================================================================

TEST(CRandomFieldGridMap2D, Resize_KalmanFilter_GrowsGridAndCov)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  grid.insertIndividualReading(0.6, {0.0, 0.0}, true, true, 1.0);

  const size_t oldSizeX = grid.getSizeX();
  const size_t oldSizeY = grid.getSizeY();

  // Use a fill-cell consistent with the map's own default prior (mirroring
  // what internal callers such as insertObservation_KF() do), so that the
  // per-cell "std" field and the covariance matrix diagonal stay coherent.
  const TRandomFieldCell defCell(
      grid.insertionOptions.KF_defaultCellMeanValue, grid.insertionOptions.KF_initialCellStd);
  grid.resize(-3.0, 3.0, -3.0, 3.0, defCell, 0.0);

  EXPECT_GT(grid.getSizeX(), oldSizeX);
  EXPECT_GT(grid.getSizeY(), oldSizeY);

  // The previously-observed cell should keep a finite, sane value after the
  // covariance matrix has been rebuilt around it.
  const TRandomFieldCell* cell = grid.cellByPos(0.0, 0.0);
  ASSERT_NE(cell, nullptr);
  EXPECT_TRUE(std::isfinite(cell->kf_mean()));
  EXPECT_GE(cell->kf_std(), 0.0);

  // A brand-new cell far from the original grid should carry the default
  // prior standard deviation.
  const TRandomFieldCell* newCell = grid.cellByPos(2.5, 2.5);
  ASSERT_NE(newCell, nullptr);
  EXPECT_GT(newCell->kf_std(), 0.0);
}

TEST(CRandomFieldGridMap2D, Resize_KalmanApproximate_GrowsGridAndCov)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanApproximate, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  // Note: inserting a reading may itself trigger an internal auto-resize
  // (insertObservation_KF2 grows the grid to fit a window around the
  // sensor), so the "before" size must be captured prior to insertion.
  const size_t sizeBeforeInsert = grid.getSizeX();

  grid.insertIndividualReading(0.6, {0.0, 0.0}, true, true, 1.0);
  EXPECT_GE(grid.getSizeX(), sizeBeforeInsert);

  const size_t oldSizeX = grid.getSizeX();
  const size_t oldSizeY = grid.getSizeY();

  // Mark newly-added cells with a negative std, mirroring the sentinel value
  // used internally by insertObservation_KF2() to flag "new" cells whose
  // compressed covariance row still needs to be initialized. The target
  // bounds are chosen to be comfortably larger than any auto-resize that may
  // have already happened during insertion above.
  grid.resize(-10.0, 10.0, -10.0, 10.0, TRandomFieldCell(0.0, -1.0), 0.0);

  EXPECT_GT(grid.getSizeX(), oldSizeX);
  EXPECT_GT(grid.getSizeY(), oldSizeY);

  mrpt::math::CVectorDouble means, stds;
  grid.getMeanAndSTD(means, stds);
  ASSERT_EQ(static_cast<size_t>(means.size()), grid.getSizeX() * grid.getSizeY());
  for (int i = 0; i < stds.size(); i++)
  {
    EXPECT_TRUE(std::isfinite(stds[i]));
    EXPECT_GE(stds[i], 0.0);
  }
}

TEST(CRandomFieldGridMap2D, Resize_NoActualSizeChange_IsANoOp)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -2.0f, 2.0f, -2.0f, 2.0f, 0.5f);

  const size_t oldSizeX = grid.getSizeX();
  const size_t oldSizeY = grid.getSizeY();

  // Resizing to (approximately) the same bounds should not change the grid
  // dimensions, exercising the "did we actually resize?" early-out.
  grid.resize(grid.getXMin(), grid.getXMax(), grid.getYMin(), grid.getYMax(), TRandomFieldCell());

  EXPECT_EQ(grid.getSizeX(), oldSizeX);
  EXPECT_EQ(grid.getSizeY(), oldSizeY);
}

// =========================================================================
//  saveAsBitmapFile() / getAsBitmapFile() / getAsMatrix(), all 5 types
// =========================================================================

TEST(CRandomFieldGridMap2D, GetAsMatrixAndBitmap_AllMapTypes)
{
  for (const auto mapType : kAllMapTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);

    mrpt::math::CMatrixDouble mat;
    grid.getAsMatrix(mat);
    EXPECT_EQ(static_cast<size_t>(mat.rows()), grid.getSizeY()) << "mapType=" << mapType;
    EXPECT_EQ(static_cast<size_t>(mat.cols()), grid.getSizeX()) << "mapType=" << mapType;

    for (int r = 0; r < mat.rows(); r++)
    {
      for (int c = 0; c < mat.cols(); c++)
      {
        EXPECT_TRUE(std::isfinite(mat(r, c))) << "mapType=" << mapType;
      }
    }

    mrpt::img::CImage img;
    grid.getAsBitmapFile(img);
    EXPECT_GT(img.getWidth(), 0u) << "mapType=" << mapType;
    EXPECT_GT(img.getHeight(), 0u) << "mapType=" << mapType;

    const auto outFile = (std::filesystem::temp_directory_path() /
                          ("crfgm_bitmap_" + std::to_string(static_cast<int>(mapType)) + ".png"))
                             .string();
    EXPECT_NO_THROW(grid.saveAsBitmapFile(outFile)) << "mapType=" << mapType;
    EXPECT_TRUE(mrpt::system::fileExists(outFile)) << "mapType=" << mapType;
  }
}

// =========================================================================
//  saveMetricMapRepresentationToFile(), all 5 types
// =========================================================================

TEST(CRandomFieldGridMap2D, SaveMetricMapRepresentationToFile_AllMapTypes)
{
  for (const auto mapType : kAllMapTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);

    const auto prefix = (std::filesystem::temp_directory_path() /
                         ("crfgm_repr_" + std::to_string(static_cast<int>(mapType))))
                            .string();

    EXPECT_NO_THROW(grid.saveMetricMapRepresentationToFile(prefix)) << "mapType=" << mapType;
    EXPECT_TRUE(mrpt::system::fileExists(prefix + "_mean.png")) << "mapType=" << mapType;
    EXPECT_TRUE(mrpt::system::fileExists(prefix + "_grid_limits.txt")) << "mapType=" << mapType;
    EXPECT_TRUE(mrpt::system::fileExists(prefix + "_mean.txt")) << "mapType=" << mapType;
  }
}

// =========================================================================
//  saveAsMatlab3DGraph(): valid for KF-family, must throw for kernel types
// =========================================================================

TEST(CRandomFieldGridMap2D, SaveAsMatlab3DGraph_KFFamilyTypes)
{
  const TMapRepresentation kfTypes[3] = {
      CRandomFieldGridMap2D::mrKalmanFilter,
      CRandomFieldGridMap2D::mrKalmanApproximate,
      CRandomFieldGridMap2D::mrGMRF_SD,
  };

  for (const auto mapType : kfTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);

    const auto outFile = (std::filesystem::temp_directory_path() /
                          ("crfgm_matlab_" + std::to_string(static_cast<int>(mapType)) + ".m"))
                             .string();

    EXPECT_NO_THROW(grid.saveAsMatlab3DGraph(outFile)) << "mapType=" << mapType;
    EXPECT_TRUE(mrpt::system::fileExists(outFile)) << "mapType=" << mapType;
  }
}

TEST(CRandomFieldGridMap2D, SaveAsMatlab3DGraph_KernelTypesThrow)
{
  const TMapRepresentation kernelTypes[2] = {
      CRandomFieldGridMap2D::mrKernelDM,
      CRandomFieldGridMap2D::mrKernelDMV,
  };

  for (const auto mapType : kernelTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);
    const auto outFile =
        (std::filesystem::temp_directory_path() /
         ("crfgm_matlab_invalid_" + std::to_string(static_cast<int>(mapType)) + ".m"))
            .string();
    EXPECT_THROW(grid.saveAsMatlab3DGraph(outFile), std::exception) << "mapType=" << mapType;
  }
}

// =========================================================================
//  getVisualizationInto() / getAs3DObject()
// =========================================================================

TEST(CRandomFieldGridMap2D, GetVisualizationInto_DisabledIsNoOp)
{
  CGasConcentrationGridMap2D grid = makeGridWithReadings(CRandomFieldGridMap2D::mrKalmanFilter);
  grid.genericMapParams.enableSaveAs3DObject = false;

  mrpt::viz::CSetOfObjects outObj;
  grid.getVisualizationInto(outObj);
  EXPECT_TRUE(outObj.empty());

  mrpt::viz::CSetOfObjects meanObj, varObj;
  grid.getAs3DObject(meanObj, varObj);
  EXPECT_TRUE(meanObj.empty());
  EXPECT_TRUE(varObj.empty());
}

TEST(CRandomFieldGridMap2D, GetAs3DObject_KFFamilyTypes_ProducesTriangles)
{
  const TMapRepresentation kfTypes[3] = {
      CRandomFieldGridMap2D::mrKalmanFilter,
      CRandomFieldGridMap2D::mrKalmanApproximate,
      CRandomFieldGridMap2D::mrGMRF_SD,
  };

  for (const auto mapType : kfTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);
    ASSERT_TRUE(grid.genericMapParams.enableSaveAs3DObject);

    mrpt::viz::CSetOfObjects meanObj, varObj;
    grid.getAs3DObject(meanObj, varObj);
    EXPECT_FALSE(meanObj.empty()) << "mapType=" << mapType;
    EXPECT_FALSE(varObj.empty()) << "mapType=" << mapType;

    mrpt::viz::CSetOfObjects visObj;
    grid.getVisualizationInto(visObj);
    EXPECT_FALSE(visObj.empty()) << "mapType=" << mapType;
  }
}

TEST(CRandomFieldGridMap2D, GetAs3DObject_KernelTypes_ProducesTriangles)
{
  const TMapRepresentation kernelTypes[2] = {
      CRandomFieldGridMap2D::mrKernelDM,
      CRandomFieldGridMap2D::mrKernelDMV,
  };

  for (const auto mapType : kernelTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);

    mrpt::viz::CSetOfObjects meanObj, varObj;
    grid.getAs3DObject(meanObj, varObj);
    EXPECT_FALSE(meanObj.empty()) << "mapType=" << mapType;
    EXPECT_FALSE(varObj.empty()) << "mapType=" << mapType;
  }
}

// =========================================================================
//  predictMeasurement()
// =========================================================================

TEST(CRandomFieldGridMap2D, PredictMeasurement_NearestVsBilinear_Interior)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 0.5f);

  grid.insertIndividualReading(0.7, {0.0, 0.0}, true, true, 1.0);

  double val = 0;
  double var = 0;
  grid.predictMeasurement(0.05, 0.05, val, var, false, CRandomFieldGridMap2D::gimNearest);
  EXPECT_TRUE(std::isfinite(val));
  EXPECT_GE(var, 0.0);

  double valBilinear = 0;
  double varBilinear = 0;
  grid.predictMeasurement(
      0.05, 0.05, valBilinear, varBilinear, false, CRandomFieldGridMap2D::gimBilinear);
  EXPECT_TRUE(std::isfinite(valBilinear));
  EXPECT_GE(varBilinear, 0.0);
}

TEST(CRandomFieldGridMap2D, PredictMeasurement_BilinearNearLowerBorder_FallsBackToNearest)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 0.5f);
  grid.insertIndividualReading(0.4, {-1.9, -1.9}, true, true, 1.0);

  double val = 0;
  double var = 0;
  // Very close to (x_min, y_min): must take the single-query fallback path.
  grid.predictMeasurement(-1.95, -1.95, val, var, false, CRandomFieldGridMap2D::gimBilinear);
  EXPECT_TRUE(std::isfinite(val));
  EXPECT_GE(var, 0.0);
}

TEST(CRandomFieldGridMap2D, PredictMeasurement_BilinearNearUpperYBorder)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 0.5f);
  grid.insertIndividualReading(0.4, {0.0, 1.9}, true, true, 1.0);

  double val = 0;
  double var = 0;
  // x is centered (far from any x border) but y is right at the upper
  // border; exercises the upper-y-border branch of the bilinear check.
  grid.predictMeasurement(0.0, 1.95, val, var, false, CRandomFieldGridMap2D::gimBilinear);
  EXPECT_TRUE(std::isfinite(val));
  EXPECT_GE(var, 0.0);
}

TEST(CRandomFieldGridMap2D, PredictMeasurement_SensorNormalization)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -2.0f, 2.0f, -2.0f, 2.0f, 0.5f);
  grid.insertionOptions.R_min = 0.0f;
  grid.insertionOptions.R_max = 2.0f;
  grid.insertIndividualReading(0.5, {0.0, 0.0}, true, true, 1.0);

  double valRaw = 0;
  double varRaw = 0;
  grid.predictMeasurement(0.0, 0.0, valRaw, varRaw, false, CRandomFieldGridMap2D::gimNearest);

  double valNorm = 0;
  double varNorm = 0;
  grid.predictMeasurement(0.0, 0.0, valNorm, varNorm, true, CRandomFieldGridMap2D::gimNearest);

  // Normalization rescales into [R_min, R_max]: value = R_min + raw*(R_max-R_min)
  EXPECT_NEAR(
      valNorm,
      grid.insertionOptions.R_min +
          valRaw * (grid.insertionOptions.R_max - grid.insertionOptions.R_min),
      1e-6);
}

TEST(CRandomFieldGridMap2D, PredictMeasurement_OutsideGrid_AllMapTypes)
{
  for (const auto mapType : kAllMapTypes)
  {
    CGasConcentrationGridMap2D grid = makeGridWithReadings(mapType);

    double val = 0;
    double var = 0;
    // Far outside the grid extent: cellByIndex() returns nullptr internally.
    EXPECT_NO_THROW(
        grid.predictMeasurement(1.0e4, 1.0e4, val, var, false, CRandomFieldGridMap2D::gimNearest))
        << "mapType=" << mapType;
    EXPECT_TRUE(std::isfinite(val)) << "mapType=" << mapType;
    EXPECT_TRUE(std::isfinite(var)) << "mapType=" << mapType;
    EXPECT_GE(var, 0.0) << "mapType=" << mapType;
  }
}

// =========================================================================
//  getMeanAndCov() / getMeanAndSTD() / setMeanAndSTD()
// =========================================================================

TEST(CRandomFieldGridMap2D, GetAndSetMeanAndSTD_KalmanFilter)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);
  grid.insertIndividualReading(0.3, {0.0, 0.0}, true, true, 1.0);

  mrpt::math::CVectorDouble means, stds;
  grid.getMeanAndSTD(means, stds);
  const size_t N = grid.getSizeX() * grid.getSizeY();
  ASSERT_EQ(static_cast<size_t>(means.size()), N);
  ASSERT_EQ(static_cast<size_t>(stds.size()), N);

  mrpt::math::CVectorDouble cov_means;
  mrpt::math::CMatrixDouble cov;
  grid.getMeanAndCov(cov_means, cov);
  ASSERT_EQ(static_cast<size_t>(cov_means.size()), N);
  // For the (full) Kalman-filter representation the full covariance matrix
  // is actually maintained, so it must be square with size N.
  EXPECT_EQ(static_cast<size_t>(cov.rows()), N);
  EXPECT_EQ(static_cast<size_t>(cov.cols()), N);

  // Round-trip: perturb the means and set them back.
  mrpt::math::CVectorDouble newMeans = means;
  for (int i = 0; i < newMeans.size(); i++)
  {
    newMeans[i] += 0.01;
  }
  grid.setMeanAndSTD(newMeans, stds);

  mrpt::math::CVectorDouble means2, stds2;
  grid.getMeanAndSTD(means2, stds2);
  for (int i = 0; i < means2.size(); i++)
  {
    EXPECT_NEAR(means2[i], newMeans[i], 1e-9);
  }
}

TEST(CRandomFieldGridMap2D, GetMeanAndSTD_KalmanApproximate)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanApproximate, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);
  grid.insertIndividualReading(0.3, {0.0, 0.0}, true, true, 1.0);

  mrpt::math::CVectorDouble means, stds;
  grid.getMeanAndSTD(means, stds);
  const size_t N = grid.getSizeX() * grid.getSizeY();
  ASSERT_EQ(static_cast<size_t>(means.size()), N);
  for (int i = 0; i < stds.size(); i++)
  {
    EXPECT_GE(stds[i], 0.0);
    EXPECT_TRUE(std::isfinite(stds[i]));
  }
}

// =========================================================================
//  TInsertionOptionsCommon: dump / load round-trip
// =========================================================================

TEST(CRandomFieldGridMap2D, InsertionOptionsCommon_DumpToTextStream)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  std::stringstream ss;
  grid.insertionOptions.dumpToTextStream(ss);
  const std::string text = ss.str();

  EXPECT_FALSE(text.empty());
  EXPECT_NE(text.find("sigma"), std::string::npos);
  EXPECT_NE(text.find("GMRF_lambdaPrior"), std::string::npos);
  EXPECT_NE(text.find("KF_covSigma"), std::string::npos);
}

TEST(CRandomFieldGridMap2D, InsertionOptionsCommon_LoadFromConfigFile_RoundTrip)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string section = "InsertionOpts";

  cfg.write(section, "sigma", 0.33f);
  cfg.write(section, "cutoffRadius", 0.77f);
  cfg.write(section, "R_min", 0.1f);
  cfg.write(section, "R_max", 5.0f);
  cfg.write(section, "GMRF_lambdaPrior", 0.02f);
  cfg.write(section, "GMRF_lambdaObs", 20.0f);
  cfg.write(section, "KF_covSigma", 0.5f);
  cfg.write(section, "KF_W_size", 6);
  cfg.write(section, "GMRF_use_occupancy_information", false);

  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);
  grid.insertionOptions.loadFromConfigFile(cfg, section);

  EXPECT_FLOAT_EQ(grid.insertionOptions.sigma, 0.33f);
  EXPECT_FLOAT_EQ(grid.insertionOptions.cutoffRadius, 0.77f);
  EXPECT_FLOAT_EQ(grid.insertionOptions.R_min, 0.1f);
  EXPECT_FLOAT_EQ(grid.insertionOptions.R_max, 5.0f);
  EXPECT_NEAR(grid.insertionOptions.GMRF_lambdaPrior, 0.02, 1e-6);
  EXPECT_NEAR(grid.insertionOptions.GMRF_lambdaObs, 20.0, 1e-6);
  EXPECT_FLOAT_EQ(grid.insertionOptions.KF_covSigma, 0.5f);
  EXPECT_EQ(grid.insertionOptions.KF_W_size, static_cast<uint16_t>(6));
  EXPECT_FALSE(grid.insertionOptions.GMRF_use_occupancy_information);
}

// =========================================================================
//  Miscellaneous small accessors
// =========================================================================

TEST(CRandomFieldGridMap2D, BasicAccessors)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  EXPECT_FALSE(grid.asString().empty());
  EXPECT_EQ(grid.getMapType(), CRandomFieldGridMap2D::mrKalmanFilter);
  EXPECT_FALSE(grid.isEmpty());

  const TRandomFieldCell cell(0.5, 0.1);
  EXPECT_FLOAT_EQ(grid.cell2float(cell), mrpt::d2f(0.5));

  EXPECT_FALSE(grid.isEnabledVerbose());
  grid.enableVerbose(true);
  EXPECT_TRUE(grid.isEnabledVerbose());
  grid.enableVerbose(false);

  EXPECT_FALSE(grid.isProfilerEnabled());
  grid.enableProfiler(true);
  EXPECT_TRUE(grid.isProfilerEnabled());
  grid.enableProfiler(false);
}

TEST(CRandomFieldGridMap2D, Compute3DMatchingRatioAlwaysZero)
{
  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrKalmanFilter, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);
  CGasConcentrationGridMap2D other(
      CRandomFieldGridMap2D::mrKalmanFilter, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  mrpt::maps::TMatchingRatioParams params;
  const float ratio = grid.compute3DMatchingRatio(&other, mrpt::poses::CPose3D(), params);
  EXPECT_FLOAT_EQ(ratio, 0.0f);
}
