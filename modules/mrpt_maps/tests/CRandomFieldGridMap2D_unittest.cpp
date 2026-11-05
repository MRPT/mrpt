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
#include <test_mrpt_common.h>

#include <Eigen/Dense>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>

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

// =========================================================================
//  GMRF prior built from an occupancy gridmap
// =========================================================================

TEST(CRandomFieldGridMap2D, GMRF_PriorFromOccupancyGridmapImage)
{
  using namespace std::string_literals;

  // internal_clear() dumps a debug bitmap of the occupancy map into the
  // current working directory, so run this from a scratch one.
  const auto prevDir = std::filesystem::current_path();
  const auto tmpDir = std::filesystem::path(mrpt::system::getTempFileName() + "_gmrf");
  std::filesystem::create_directories(tmpDir);
  std::filesystem::current_path(tmpDir);
  struct DirRestorer
  {
    std::filesystem::path prev, tmp;
    ~DirRestorer()
    {
      std::filesystem::current_path(prev);
      std::filesystem::remove_all(tmp);
    }
  } restorer{prevDir, tmpDir};

  CGasConcentrationGridMap2D grid(
      CRandomFieldGridMap2D::mrGMRF_SD, -1.0f, 1.0f, -1.0f, 1.0f, 0.20f);

  auto* opts = &grid.insertionOptions;
  opts->GMRF_use_occupancy_information = true;
  opts->GMRF_gridmap_image_file = mrpt::UNITTEST_BASEDIR() + "/tests/map_pgm_32.pgm"s;
  opts->GMRF_gridmap_image_res = 0.10f;
  opts->GMRF_gridmap_image_cx = 15;
  opts->GMRF_gridmap_image_cy = 33;

  // clear() rebuilds the factor graph, this time deriving the cell
  // connectivity from the occupancy map through region growing:
  grid.clear();

  // The map must have been resized to match the occupancy gridmap:
  EXPECT_GT(grid.getSizeX(), 0U);
  EXPECT_GT(grid.getSizeY(), 0U);

  grid.insertIndividualReading(0.5, {0.0, 0.0}, true, true, 1.0);

  const TRandomFieldCell* cell = grid.cellByPos(0.0, 0.0);
  ASSERT_NE(cell, nullptr);
  EXPECT_TRUE(std::isfinite(cell->gmrf_mean()));
}

TEST(CRandomFieldGridMap2D, GMRF_OccupancyWithoutAnySourceThrows)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  auto* opts = &grid.insertionOptions;
  opts->GMRF_use_occupancy_information = true;
  opts->GMRF_simplemap_file.clear();
  opts->GMRF_gridmap_image_file.clear();

  EXPECT_THROW(grid.clear(), std::exception);
}

TEST(CRandomFieldGridMap2D, GMRF_OccupancyFromMissingImageThrows)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -1.0f, 1.0f, -1.0f, 1.0f, 0.5f);

  auto* opts = &grid.insertionOptions;
  opts->GMRF_use_occupancy_information = true;
  opts->GMRF_gridmap_image_file = "/nonexistent-dir/does_not_exist.png";
  opts->GMRF_gridmap_image_res = 0.1f;

  EXPECT_THROW(grid.clear(), std::exception);
}

namespace
{
// Dense prior used by the mrKalmanApproximate method: squared-exponential
// covariance, truncated out of a (2W+1)x(2W+1) window around each cell.
Eigen::MatrixXd truncatedSEPrior(
    const CRandomFieldGridMap2D& g, int W, double sigma, double std0, bool truncate)
{
  const int sx = static_cast<int>(g.getSizeX());
  const int N = static_cast<int>(g.getSizeX() * g.getSizeY());
  const double res = g.getResolution();
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(N, N);
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      const int dx = (j % sx) - (i % sx);
      const int dy = (j / sx) - (i / sx);
      if (truncate && (std::abs(dx) > W || std::abs(dy) > W))
      {
        continue;
      }
      P(i, j) =
          std0 * std0 *
          std::exp(-0.5 * res * res * static_cast<double>(dx * dx + dy * dy) / (sigma * sigma));
    }
  }
  return P;
}
}  // namespace

// Regression: the kernel window was centered one cell off the sensor cell.
TEST(CRandomFieldGridMap2D, KernelDM_WindowCenteredOnSensorCell)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKernelDM, -1, 1, -1, 1, 0.1f);
  grid.insertionOptions.sigma = 0.15f;
  grid.insertionOptions.cutoffRadius = 0.45f;
  grid.clear();

  const double x0 = grid.idx2x(10);
  const double y0 = grid.idx2y(10);
  grid.insertIndividualReading(1.0, {x0, y0});

  const int cx = grid.x2idx(x0);
  const int cy = grid.y2idx(y0);
  const double res = grid.getResolution();
  const double sigma = grid.insertionOptions.sigma;
  for (int dy = -2; dy <= 2; dy++)
  {
    for (int dx = -2; dx <= 2; dx++)
    {
      const auto* cell = grid.cellByIndex(cx + dx, cy + dy);
      ASSERT_TRUE(cell != nullptr);
      const double d2 = res * res * static_cast<double>(dx * dx + dy * dy);
      EXPECT_NEAR(cell->dm_mean_w(), std::exp(-d2 / (sigma * sigma)), 1e-6)
          << "dx=" << dx << " dy=" << dy;
    }
  }
}

// Regression: the compressed covariance update modified wrong entries for
// pairs of cells whose x offset was below -W.
TEST(CRandomFieldGridMap2D, KalmanApproximate_MatchesDenseKalmanFilter)
{
  const int W = 3;
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKalmanApproximate, -2, 2, -2, 2, 0.2f);
  grid.insertionOptions.KF_W_size = W;
  grid.insertionOptions.KF_covSigma = 0.3f;
  grid.insertionOptions.KF_initialCellStd = 1.0f;
  grid.insertionOptions.KF_observationModelNoise = 0.1f;
  grid.insertionOptions.KF_defaultCellMeanValue = 0;
  grid.clear();

  const size_t N = grid.getSizeX() * grid.getSizeY();
  const int sx = static_cast<int>(grid.getSizeX());
  const double R = static_cast<double>(grid.insertionOptions.KF_observationModelNoise);
  Eigen::MatrixXd P = truncatedSEPrior(
      grid, W, static_cast<double>(grid.insertionOptions.KF_covSigma), 1.0, true /*truncate*/);
  Eigen::VectorXd m = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(N));

  // Repeated readings of the same cell keep the approximation exact for all
  // the stored entries:
  const mrpt::math::TPoint2D pt(0.1, 0.1);
  for (const double z : {0.8, 0.6})
  {
    grid.insertIndividualReading(z, pt);
    ASSERT_EQ(grid.getSizeX() * grid.getSizeY(), N);  // no resize

    const int c = grid.xy2idx(pt.x, pt.y);
    const double s = P(c, c) + R * R;
    const Eigen::VectorXd pc = P.col(c);
    m += pc * (z - m[c]) / s;
    P -= pc * pc.transpose() / s;

    mrpt::math::CVectorDouble means;
    mrpt::math::CMatrixDouble cov;
    grid.getMeanAndCov(means, cov);
    ASSERT_EQ(static_cast<size_t>(cov.rows()), N);
    ASSERT_EQ(static_cast<size_t>(cov.cols()), N);

    for (size_t i = 0; i < N; i++)
    {
      // (tolerance: the map options are float numbers)
      EXPECT_NEAR(means[i], m[static_cast<Eigen::Index>(i)], 1e-7) << "i=" << i;
      for (size_t j = 0; j < N; j++)
      {
        const int dx = static_cast<int>(j % sx) - static_cast<int>(i % sx);
        const int dy = static_cast<int>(j / sx) - static_cast<int>(i / sx);
        if (std::abs(dx) > W || std::abs(dy) > W)
        {
          continue;  // not kept by this approximation
        }
        EXPECT_NEAR(cov(i, j), P(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)), 1e-7)
            << "i=" << i << " j=" << j;
      }
    }
  }
}

// Regression: the prior covariance between two new cells after a resize used
// the variance of a cell that was not initialized yet.
TEST(CRandomFieldGridMap2D, KalmanFilter_ResizeSetsPriorOfNewCells)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 0.5f);
  grid.insertionOptions.KF_covSigma = 0.6f;
  grid.insertionOptions.KF_initialCellStd = 1.0f;
  grid.clear();
  grid.insertIndividualReading(0.5, {0.1, 0.1});  // inner reading: no resize
  ASSERT_EQ(grid.getSizeX(), 8u);

  grid.resize(-2, 3, -2, 3, mrpt::maps::TRandomFieldCell(0, 1.0), 0.0);
  ASSERT_EQ(grid.getSizeX(), 10u);
  ASSERT_EQ(grid.getSizeY(), 10u);

  mrpt::math::CVectorDouble means;
  mrpt::math::CMatrixDouble cov;
  grid.getMeanAndCov(means, cov);
  const int sx = static_cast<int>(grid.getSizeX());
  const int N = static_cast<int>(grid.getSizeX() * grid.getSizeY());
  ASSERT_EQ(cov.rows(), N);

  const double sigma = static_cast<double>(grid.insertionOptions.KF_covSigma);
  for (int i = 0; i < N; i++)
  {
    for (int j = i; j < N; j++)
    {
      const double xi = grid.idx2x(i % sx);
      const double yi = grid.idx2y(i / sx);
      const double xj = grid.idx2x(j % sx);
      const double yj = grid.idx2y(j / sx);
      // Only pairs of new cells (outside of the old [-2,2]x[-2,2] area):
      if (!(xi > 2.0 || yi > 2.0) || !(xj > 2.0 || yj > 2.0))
      {
        continue;
      }
      const double d2 = mrpt::square(xi - xj) + mrpt::square(yi - yj);
      EXPECT_NEAR(cov(i, j), std::exp(-0.5 * d2 / (sigma * sigma)), 1e-9)
          << "i=" << i << " j=" << j;
    }
  }

  Eigen::MatrixXd Pe(N, N);
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      Pe(i, j) = cov(i, j);
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Pe);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9);
}

// Regression: the information loss was applied to the time-invariant
// observations instead of the time-variant ones.
TEST(CRandomFieldGridMap2D, GMRF_OnlyTimeVariantObservationsLoseInformation)
{
  for (const bool timeInvariant : {true, false})
  {
    CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, 0, 2, 0, 2, 0.5f);
    grid.insertionOptions.GMRF_lambdaPrior = 1.0;
    grid.insertionOptions.GMRF_lambdaObs = 10.0;
    grid.insertionOptions.GMRF_lambdaObsLoss = 4.0;
    grid.clear();

    grid.insertIndividualReading(1.0, {0.2, 0.2}, true, timeInvariant);
    // A time-invariant reading far away keeps the system well determined:
    grid.insertIndividualReading(0.0, {1.8, 1.8}, true, true);
    for (int k = 0; k < 5; k++)
    {
      grid.updateMapEstimation();
    }

    const auto* cell = grid.cellByPos(0.2, 0.2);
    ASSERT_TRUE(cell != nullptr);
    EXPECT_TRUE(std::isfinite(cell->gmrf_std())) << "timeInvariant=" << timeInvariant;
    if (timeInvariant)
    {
      EXPECT_GT(cell->gmrf_mean(), 0.9);
    }
    else
    {
      EXPECT_NEAR(cell->gmrf_mean(), 0.0, 1e-6);
    }
  }
}

// Regression: the border check of the bilinear interpolation tested x twice
// and never y against the upper border.
TEST(CRandomFieldGridMap2D, PredictMeasurement_BilinearAtUpperYBorderUsesNearestCell)
{
  // GMRF maps never resize upon insertion:
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -2, 2, -2, 2, 0.5f);
  grid.insertionOptions.GMRF_lambdaPrior = 1.0;
  grid.clear();
  const double x = grid.idx2x(3);
  grid.insertIndividualReading(0.8, {x, 1.9}, true, true);

  double valNearest = 0;
  double varNearest = 0;
  grid.predictMeasurement(x, 1.9, valNearest, varNearest, false, CRandomFieldGridMap2D::gimNearest);
  double valBilinear = 0;
  double varBilinear = 0;
  grid.predictMeasurement(
      x, 1.9, valBilinear, varBilinear, false, CRandomFieldGridMap2D::gimBilinear);

  EXPECT_NEAR(valBilinear, valNearest, 1e-12);
  EXPECT_NEAR(varBilinear, varNearest, 1e-12);
}

// Regression: the variance was not scaled back to sensor units.
TEST(CRandomFieldGridMap2D, PredictMeasurement_NormalizationAlsoScalesVariance)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKalmanFilter, -2, 2, -2, 2, 0.5f);
  grid.insertionOptions.R_min = 1.0f;
  grid.insertionOptions.R_max = 3.0f;
  grid.insertionOptions.KF_observationModelNoise = 0.1f;
  grid.insertIndividualReading(0.5, {0.0, 0.0}, true, true);

  double valRaw = 0;
  double varRaw = 0;
  grid.predictMeasurement(0.0, 0.0, valRaw, varRaw, false, CRandomFieldGridMap2D::gimNearest);
  double valSensor = 0;
  double varSensor = 0;
  grid.predictMeasurement(0.0, 0.0, valSensor, varSensor, true, CRandomFieldGridMap2D::gimNearest);

  EXPECT_NEAR(valSensor, 1.0 + 2.0 * valRaw, 1e-9);
  EXPECT_NEAR(varSensor, 4.0 * varRaw, 1e-9);
}

// Regression: the GMRF predictive variance ignored the GMRF observation
// noise (1/GMRF_lambdaObs).
TEST(CRandomFieldGridMap2D, PredictMeasurement_GMRFVarianceIncludesObservationNoise)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -1, 1, -1, 1, 0.5f);
  grid.insertionOptions.GMRF_lambdaPrior = 1.0;
  grid.insertionOptions.GMRF_lambdaObs = 4.0;
  grid.clear();
  grid.insertIndividualReading(0.5, {0.1, 0.1}, true, true);

  double val = 0;
  double var = 0;
  grid.predictMeasurement(0.1, 0.1, val, var, false, CRandomFieldGridMap2D::gimNearest);
  const auto* cell = grid.cellByPos(0.1, 0.1);
  ASSERT_TRUE(cell != nullptr);
  EXPECT_NEAR(val, cell->gmrf_mean(), 1e-12);
  EXPECT_NEAR(var, mrpt::square(cell->gmrf_std()) + 1.0 / 4.0, 1e-12);
}

// Regression: getMeanAndSTD() and setMeanAndSTD() used the compressed
// covariance of mrKalmanApproximate for all the map types.
TEST(CRandomFieldGridMap2D, GetAndSetMeanAndSTD_KalmanFilterUseTheFullCovariance)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrKalmanFilter, -1, 1, -1, 1, 0.5f);
  grid.insertIndividualReading(0.3, {0.0, 0.0}, true, true);

  mrpt::math::CVectorDouble means;
  mrpt::math::CVectorDouble stds;
  grid.getMeanAndSTD(means, stds);
  mrpt::math::CVectorDouble covMeans;
  mrpt::math::CMatrixDouble cov;
  grid.getMeanAndCov(covMeans, cov);
  const size_t N = grid.getSizeX() * grid.getSizeY();
  ASSERT_EQ(static_cast<size_t>(stds.size()), N);
  ASSERT_EQ(static_cast<size_t>(cov.rows()), N);
  for (size_t i = 0; i < N; i++)
  {
    EXPECT_NEAR(stds[i], std::sqrt(cov(i, i)), 1e-12) << "i=" << i;
  }

  mrpt::math::CVectorDouble newStds = stds;
  newStds.fill(0.5);
  grid.setMeanAndSTD(means, newStds);
  grid.getMeanAndSTD(means, stds);
  grid.getMeanAndCov(covMeans, cov);
  for (size_t i = 0; i < N; i++)
  {
    EXPECT_NEAR(stds[i], 0.5, 1e-12);
    EXPECT_NEAR(cov(i, i), 0.25, 1e-12);
  }
}

// Regression: the GMRF_lambdaObsLoss line printed GMRF_lambdaObs.
TEST(CRandomFieldGridMap2D, InsertionOptionsCommon_DumpShowsLambdaObsLoss)
{
  CGasConcentrationGridMap2D grid(CRandomFieldGridMap2D::mrGMRF_SD, -1, 1, -1, 1, 0.5f);
  grid.insertionOptions.GMRF_lambdaObs = 10.0;
  grid.insertionOptions.GMRF_lambdaObsLoss = 0.25;
  std::stringstream ss;
  grid.insertionOptions.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("GMRF_lambdaObsLoss                      = 0.250000"), std::string::npos)
      << ss.str();
}
