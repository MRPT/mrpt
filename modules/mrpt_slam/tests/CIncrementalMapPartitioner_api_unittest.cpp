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

/** Unit tests for the non-dataset part of CIncrementalMapPartitioner: its
 *  options, the three similarity methods, node removal, coordinate origin
 *  changes, the 3D representation and serialization.
 *  \sa CIncrementalMapPartitioner_unittest.cpp for the end-to-end test on a
 *      real dataset.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/archiveFrom_std_streams.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>

#include <sstream>

using mrpt::slam::CIncrementalMapPartitioner;

namespace
{
mrpt::obs::CSensoryFrame makeFrame(int scanIndex = 0)
{
  auto scan = mrpt::obs::CObservation2DRangeScan::Create();
  mrpt::obs::stock_observations::example2DRangeScan(*scan, scanIndex);

  mrpt::obs::CSensoryFrame sf;
  sf.insert(scan);
  return sf;
}

mrpt::poses::CPose3DPDFGaussian poseAt(double x, double y = 0, double phi = 0)
{
  mrpt::poses::CPose3DPDFGaussian p;
  p.mean = mrpt::poses::CPose3D(x, y, 0, phi, 0, 0);
  return p;
}

/** Adds n keyframes at x=0,1,2,... */
void addNodes(CIncrementalMapPartitioner& imp, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    const auto sf = makeFrame(static_cast<int>(i % 2));
    imp.addMapFrame(sf, poseAt(static_cast<double>(i)));
  }
}
}  // namespace

TEST(CIncrementalMapPartitionerAPI, optionsSaveLoadRoundTrip)
{
  CIncrementalMapPartitioner::TOptions o;
  o.partitionThreshold = 0.75;
  o.forceBisectionOnly = true;
  o.simil_method = mrpt::slam::smOBSERVATION_OVERLAP;
  o.minimumNumberElementsEachCluster = 4;
  o.maxKeyFrameDistanceToEval = 20;
  o.mrp.maxDistForCorr = 0.33f;
  o.mrp.maxMahaDistForCorr = 7.5f;

  mrpt::config::CConfigFileMemory cfg;
  o.saveToConfigFile(cfg, "PART");

  CIncrementalMapPartitioner::TOptions o2;
  o2.loadFromConfigFile(cfg, "PART");

  EXPECT_NEAR(o2.partitionThreshold, 0.75, 1e-9);
  EXPECT_TRUE(o2.forceBisectionOnly);
  EXPECT_EQ(o2.simil_method, mrpt::slam::smOBSERVATION_OVERLAP);
  EXPECT_EQ(o2.minimumNumberElementsEachCluster, 4U);
  EXPECT_EQ(o2.maxKeyFrameDistanceToEval, 20U);
  EXPECT_NEAR(o2.mrp.maxDistForCorr, 0.33f, 1e-6);
  EXPECT_NEAR(o2.mrp.maxMahaDistForCorr, 7.5f, 1e-6);

  // The default map is one CSimplePointsMap:
  EXPECT_EQ(o2.metricmap.size(), 1U);

  std::stringstream ss;
  o.dumpToTextStream(ss);
  EXPECT_FALSE(ss.str().empty());
}

TEST(CIncrementalMapPartitionerAPI, addMapFrameFillsAdjacencyMatrix)
{
  CIncrementalMapPartitioner imp;
  imp.options.simil_method = mrpt::slam::smMETRIC_MAP_MATCHING;

  EXPECT_EQ(imp.getNodesCount(), 0U);

  EXPECT_EQ(imp.addMapFrame(makeFrame(0), poseAt(0)), 0U);
  EXPECT_EQ(imp.addMapFrame(makeFrame(0), poseAt(0)), 1U);

  EXPECT_EQ(imp.getNodesCount(), 2U);

  const auto& A = imp.getAdjacencyMatrix();
  ASSERT_EQ(A.rows(), 2);
  ASSERT_EQ(A.cols(), 2);
  // Same scan at the same pose: maximum similarity, and self-similarity is
  // not used (left at zero):
  EXPECT_NEAR(A(0, 1), 1.0, 1e-6);
  EXPECT_NEAR(A(1, 0), 1.0, 1e-6);
  EXPECT_EQ(A(0, 0), 0.0);

  mrpt::math::CMatrixDouble Acopy;
  imp.getAdjacencyMatrix(Acopy);
  EXPECT_EQ(Acopy.rows(), A.rows());

  EXPECT_EQ(imp.getSequenceOfFrames()->size(), 2U);
  const auto& cimp = imp;
  EXPECT_EQ(cimp.getSequenceOfFrames()->size(), 2U);

  imp.clear();
  EXPECT_EQ(imp.getNodesCount(), 0U);
  EXPECT_EQ(imp.getAdjacencyMatrix().rows(), 0);
}

TEST(CIncrementalMapPartitionerAPI, observationOverlapSimilarity)
{
  CIncrementalMapPartitioner imp;
  imp.setSimilarityMethod(mrpt::slam::smOBSERVATION_OVERLAP);
  EXPECT_EQ(imp.options.simil_method, mrpt::slam::smOBSERVATION_OVERLAP);

  imp.addMapFrame(makeFrame(0), poseAt(0));
  imp.addMapFrame(makeFrame(0), poseAt(0));
  imp.addMapFrame(makeFrame(0), poseAt(100));  // way too far to overlap

  const auto& A = imp.getAdjacencyMatrix();
  EXPECT_NEAR(A(0, 1), 1.0, 1e-6);
  EXPECT_NEAR(A(0, 2), 0.0, 1e-6);
}

TEST(CIncrementalMapPartitionerAPI, invalidSimilarityMethodThrows)
{
  CIncrementalMapPartitioner imp;
  imp.addMapFrame(makeFrame(0), poseAt(0));

  imp.options.simil_method = static_cast<mrpt::slam::similarity_method_t>(99);
  EXPECT_THROW(imp.addMapFrame(makeFrame(0), poseAt(1)), std::exception);
}

// The similarity function is always called with "kf2 with respect to kf1", so
// the symmetrized evaluation must invert the relative pose when swapping the
// two keyframes.
TEST(CIncrementalMapPartitionerAPI, customSimilarityGetsConsistentRelativePoses)
{
  struct Call
  {
    uint32_t id1, id2;
    mrpt::poses::CPose3D relPose;
  };
  std::vector<Call> calls;

  CIncrementalMapPartitioner imp;
  imp.setSimilarityMethod(
      [&calls](
          const mrpt::slam::map_keyframe_t& kf1, const mrpt::slam::map_keyframe_t& kf2,
          const mrpt::poses::CPose3D& relPose2wrt1)
      {
        calls.push_back({kf1.kf_id, kf2.kf_id, relPose2wrt1});
        EXPECT_TRUE(kf1.metric_map);
        EXPECT_TRUE(kf1.raw_observations);
        return 0.5;
      });
  EXPECT_EQ(imp.options.simil_method, mrpt::slam::smCUSTOM_FUNCTION);

  imp.addMapFrame(makeFrame(0), poseAt(0));
  imp.addMapFrame(makeFrame(0), poseAt(3.0, 0, 0.0));

  ASSERT_EQ(calls.size(), 2U);

  // Both keyframes were compared in both directions:
  EXPECT_EQ(calls[0].id1, 1U);
  EXPECT_EQ(calls[0].id2, 0U);
  EXPECT_EQ(calls[1].id1, 0U);
  EXPECT_EQ(calls[1].id2, 1U);

  // ...and the two relative poses are the inverse of each other:
  const auto composed = calls[0].relPose + calls[1].relPose;
  EXPECT_NEAR(composed.norm(), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(calls[0].relPose.x()), 3.0, 1e-9);

  EXPECT_NEAR(imp.getAdjacencyMatrix()(0, 1), 0.5, 1e-9);
}

TEST(CIncrementalMapPartitionerAPI, maxKeyFrameDistanceToEvalSkipsFarKeyframes)
{
  CIncrementalMapPartitioner imp;
  imp.options.maxKeyFrameDistanceToEval = 1;
  imp.setSimilarityMethod([](const mrpt::slam::map_keyframe_t&, const mrpt::slam::map_keyframe_t&,
                             const mrpt::poses::CPose3D&) { return 1.0; });

  addNodes(imp, 3);

  const auto& A = imp.getAdjacencyMatrix();
  EXPECT_NEAR(A(0, 1), 1.0, 1e-9);
  EXPECT_NEAR(A(1, 2), 1.0, 1e-9);
  // |2-0| > 1 => not evaluated at all:
  EXPECT_EQ(A(0, 2), 0.0);
}

TEST(CIncrementalMapPartitionerAPI, updatePartitionsSplitsTwoClusters)
{
  CIncrementalMapPartitioner imp;
  imp.options.partitionThreshold = 1.0;
  imp.setSimilarityMethod([](const mrpt::slam::map_keyframe_t& kf1,
                             const mrpt::slam::map_keyframe_t& kf2, const mrpt::poses::CPose3D&)
                          { return (kf1.kf_id < 3) == (kf2.kf_id < 3) ? 1.0 : 0.0; });

  addNodes(imp, 6);

  std::vector<std::vector<uint32_t>> parts;
  imp.updatePartitions(parts);

  ASSERT_EQ(parts.size(), 2U);
  const std::vector<uint32_t> expected_a = {0, 1, 2}, expected_b = {3, 4, 5};
  EXPECT_TRUE(
      (parts[0] == expected_a && parts[1] == expected_b) ||
      (parts[0] == expected_b && parts[1] == expected_a));

  // With bisection forced, the result is the same first split:
  imp.options.forceBisectionOnly = true;
  std::vector<std::vector<uint32_t>> parts2;
  imp.updatePartitions(parts2);
  EXPECT_EQ(parts2.size(), 2U);
}

TEST(CIncrementalMapPartitionerAPI, removeSetOfNodes)
{
  CIncrementalMapPartitioner imp;
  imp.setSimilarityMethod(
      [](const mrpt::slam::map_keyframe_t& kf1, const mrpt::slam::map_keyframe_t& kf2,
         const mrpt::poses::CPose3D&)
      { return 1.0 / (1.0 + std::abs(static_cast<double>(kf1.kf_id) - kf2.kf_id)); });

  addNodes(imp, 4);
  const auto A_before = imp.getAdjacencyMatrix();

  imp.removeSetOfNodes({1}, false /* changeCoordsRef */);

  EXPECT_EQ(imp.getNodesCount(), 3U);
  const auto& A = imp.getAdjacencyMatrix();
  ASSERT_EQ(A.rows(), 3);
  // Old nodes 0,2,3 became 0,1,2:
  EXPECT_NEAR(A(0, 1), A_before(0, 2), 1e-12);
  EXPECT_NEAR(A(1, 2), A_before(2, 3), 1e-12);

  // The first surviving keyframe keeps its global pose:
  EXPECT_NEAR(imp.getSequenceOfFrames()->get(0).pose->getMeanVal().x(), 0.0, 1e-9);

  // Removing with changeCoordsRef leaves the first node at the origin:
  imp.removeSetOfNodes({0}, true);
  EXPECT_EQ(imp.getNodesCount(), 2U);
  EXPECT_NEAR(imp.getSequenceOfFrames()->get(0).pose->getMeanVal().x(), 0.0, 1e-9);
  EXPECT_NEAR(imp.getSequenceOfFrames()->get(1).pose->getMeanVal().x(), 1.0, 1e-9);
}

TEST(CIncrementalMapPartitionerAPI, removeAllNodesIsRejected)
{
  CIncrementalMapPartitioner imp;
  addNodes(imp, 2);
  EXPECT_THROW(imp.removeSetOfNodes({0, 1}), std::exception);
}

TEST(CIncrementalMapPartitionerAPI, changeCoordinatesOrigin)
{
  CIncrementalMapPartitioner imp;
  addNodes(imp, 3);

  imp.changeCoordinatesOrigin(mrpt::poses::CPose3D(1.0, 0, 0, 0, 0, 0));
  EXPECT_NEAR(imp.getSequenceOfFrames()->get(0).pose->getMeanVal().x(), 1.0, 1e-9);

  // Now put the origin at keyframe #2 (which is at x=1+2=3):
  imp.changeCoordinatesOriginPoseIndex(2);
  EXPECT_NEAR(imp.getSequenceOfFrames()->get(2).pose->getMeanVal().x(), 0.0, 1e-9);
  EXPECT_NEAR(imp.getSequenceOfFrames()->get(0).pose->getMeanVal().x(), -2.0, 1e-9);
}

TEST(CIncrementalMapPartitionerAPI, getAs3DScene)
{
  CIncrementalMapPartitioner imp;
  imp.setSimilarityMethod(
      [](const mrpt::slam::map_keyframe_t& kf1, const mrpt::slam::map_keyframe_t& kf2,
         const mrpt::poses::CPose3D&) {
        return std::abs(static_cast<int>(kf1.kf_id) - static_cast<int>(kf2.kf_id)) == 1 ? 0.9 : 0.0;
      });
  addNodes(imp, 3);

  auto objs = mrpt::viz::CSetOfObjects::Create();
  imp.getAs3DScene(objs);
  // 1 grid + 3 spheres + 2 links (the 0-2 pair is below the 0.01 threshold):
  EXPECT_EQ(objs->size(), 6U);
  EXPECT_TRUE(objs->getByName("1"));

  // With renamed indices:
  std::map<uint32_t, int64_t> renames;
  for (uint32_t i = 0; i < 3; i++) renames[i] = 100 + i;
  imp.getAs3DScene(objs, &renames);
  EXPECT_EQ(objs->size(), 6U);
  EXPECT_TRUE(objs->getByName("101"));

  // A missing entry in the rename map is an error:
  renames.erase(1);
  EXPECT_THROW(imp.getAs3DScene(objs, &renames), std::exception);
}

TEST(CIncrementalMapPartitionerAPI, serializationRoundTrip)
{
  CIncrementalMapPartitioner imp;
  addNodes(imp, 3);
  std::vector<std::vector<uint32_t>> parts;
  imp.updatePartitions(parts);

  std::stringstream ss;
  auto arch = mrpt::serialization::archiveFrom<std::iostream>(ss);
  arch << imp;

  CIncrementalMapPartitioner imp2;
  arch >> imp2;

  EXPECT_EQ(imp2.getNodesCount(), imp.getNodesCount());
  EXPECT_EQ(imp2.getAdjacencyMatrix().rows(), imp.getAdjacencyMatrix().rows());
  for (int r = 0; r < imp.getAdjacencyMatrix().rows(); r++)
    for (int c = 0; c < imp.getAdjacencyMatrix().cols(); c++)
      EXPECT_NEAR(imp2.getAdjacencyMatrix()(r, c), imp.getAdjacencyMatrix()(r, c), 1e-12);

  std::vector<std::vector<uint32_t>> parts2;
  imp2.updatePartitions(parts2);
  EXPECT_EQ(parts2, parts);
}
