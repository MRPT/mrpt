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

#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/config.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>
#include <iostream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::tfest;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
          saveAsBitmapFile
  ---------------------------------------------------------------*/
bool COccupancyGridMap2D::saveAsBitmapFile(const std::string& file) const
{
  MRPT_START
  CImage img;
  getAsImage(img);
  return img.saveToFile(file);
  MRPT_END
}

uint8_t COccupancyGridMap2D::serializeGetVersion() const { return 7; }
void COccupancyGridMap2D::serializeTo(mrpt::serialization::CArchive& out) const
{
  // v7: always 8-bit cells (the 16-bit compile-time switch was removed).
  // v2+: first byte = bitsPerCell (8 or 16). Always 8 now.
  out << uint8_t(8);

  out << m_size_x << m_size_y << m_xMin << m_xMax << m_yMin << m_yMax << m_resolution;
  ASSERT_(m_size_x * m_size_y == m_map.size());

  out.WriteBuffer(m_map.data(), sizeof(cellType) * m_size_x * m_size_y);

  // insertionOptions:
  out << insertionOptions.mapAltitude << insertionOptions.useMapAltitude
      << insertionOptions.maxDistanceInsertion << insertionOptions.maxOccupancyUpdateCertainty
      << insertionOptions.considerInvalidRangesAsFreeSpace << insertionOptions.decimation
      << insertionOptions.horizontalTolerance;

  // Likelihood:
  out << static_cast<int32_t>(likelihoodOptions.likelihoodMethod) << likelihoodOptions.LF_stdHit
      << likelihoodOptions.LF_zHit << likelihoodOptions.LF_zRandom << likelihoodOptions.LF_maxRange
      << likelihoodOptions.LF_decimation << likelihoodOptions.LF_maxCorrsDistance
      << likelihoodOptions.LF_alternateAverageMethod << likelihoodOptions.MI_exponent
      << likelihoodOptions.MI_skip_rays << likelihoodOptions.MI_ratio_max_distance
      << likelihoodOptions.rayTracing_useDistanceFilter << likelihoodOptions.rayTracing_decimation
      << likelihoodOptions.rayTracing_stdHit << likelihoodOptions.consensus_takeEachRange
      << likelihoodOptions.consensus_pow << likelihoodOptions.OWA_weights
      << likelihoodOptions.enableLikelihoodCache;

  out << genericMapParams;  // v6+

  out << insertionOptions.CFD_features_gaussian_size << insertionOptions.CFD_features_median_size;

  out << insertionOptions.wideningBeamsWithDistance;
}

void COccupancyGridMap2D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  m_is_empty = false;

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    {
      uint8_t bitsPerCellStream;

      // v2+: bitsPerCell byte (8 or 16). v0/v1: assumed 8-bit.
      if (version >= 2)
        in >> bitsPerCellStream;
      else
        bitsPerCellStream = 8;

      uint32_t new_size_x, new_size_y;
      float new_x_min, new_x_max, new_y_min, new_y_max;
      float new_resolution;

      in >> new_size_x >> new_size_y >> new_x_min >> new_x_max >> new_y_min >> new_y_max >>
          new_resolution;

      setSize(new_x_min, new_x_max, new_y_min, new_y_max, new_resolution, 0.5);
      ASSERT_(m_size_x * m_size_y == m_map.size());

      if (bitsPerCellStream == 8)
      {
        in.ReadBuffer(m_map.data(), sizeof(cellType) * m_map.size());
      }
      else
      {
        // Historical 16-bit stream: down-convert to 8-bit (>> 8)
        ASSERT_(bitsPerCellStream == 16);
        std::vector<uint16_t> auxMap(m_map.size());
        in.ReadBuffer(auxMap.data(), sizeof(uint16_t) * auxMap.size());
        for (size_t i = 0, N = m_map.size(); i < N; i++)
          m_map[i] = static_cast<cellType>(static_cast<uint8_t>(auxMap[i] >> 8));
      }

      // v0/v1/v2: cells stored as raw probabilities [0,255] → convert to log-odds
      if (version < 3)
      {
        for (auto& c : m_map)
        {
          double p = cellTypeUnsigned(c) * (1.0 / 0xFF);
          p = std::clamp(p, 0.0, 1.0);
          c = p2l(static_cast<float>(p));
        }
      }

      m_likelihoodCacheOutDated = true;

      if (version >= 1)
      {
        in >> insertionOptions.mapAltitude >> insertionOptions.useMapAltitude >>
            insertionOptions.maxDistanceInsertion >> insertionOptions.maxOccupancyUpdateCertainty >>
            insertionOptions.considerInvalidRangesAsFreeSpace >> insertionOptions.decimation >>
            insertionOptions.horizontalTolerance;

        int32_t lm;
        in >> lm;
        likelihoodOptions.likelihoodMethod = static_cast<TLikelihoodMethod>(lm);
        in >> likelihoodOptions.LF_stdHit >> likelihoodOptions.LF_zHit >>
            likelihoodOptions.LF_zRandom >> likelihoodOptions.LF_maxRange >>
            likelihoodOptions.LF_decimation >> likelihoodOptions.LF_maxCorrsDistance >>
            likelihoodOptions.LF_alternateAverageMethod >> likelihoodOptions.MI_exponent >>
            likelihoodOptions.MI_skip_rays >> likelihoodOptions.MI_ratio_max_distance >>
            likelihoodOptions.rayTracing_useDistanceFilter >>
            likelihoodOptions.rayTracing_decimation >> likelihoodOptions.rayTracing_stdHit >>
            likelihoodOptions.consensus_takeEachRange >> likelihoodOptions.consensus_pow >>
            likelihoodOptions.OWA_weights >> likelihoodOptions.enableLikelihoodCache;

        if (version >= 6)
          in >> genericMapParams;
        else
        {
          bool disableSaveAs3DObject;
          in >> disableSaveAs3DObject;
          genericMapParams.enableSaveAs3DObject = !disableSaveAs3DObject;
        }
      }

      if (version >= 4)
        in >> insertionOptions.CFD_features_gaussian_size >>
            insertionOptions.CFD_features_median_size;

      if (version >= 5) in >> insertionOptions.wideningBeamsWithDistance;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

bool COccupancyGridMap2D::loadFromBitmapFile(
    const std::string& file, float res, const TPoint2D& origin)
{
  MRPT_START

  CImage imgFl;
  if (!imgFl.loadFromFile(file))
  {
    return false;
  }

  m_is_empty = false;
  return loadFromBitmap(imgFl, res, origin);

  MRPT_END
}

bool COccupancyGridMap2D::loadFromBitmap(
    const mrpt::img::CImage& imgFl, float res, const mrpt::math::TPoint2D& origin_)
{
  MRPT_START

  // For the precomputed likelihood trick:
  m_likelihoodCacheOutDated = true;

  size_t bmpWidth = imgFl.getWidth();
  size_t bmpHeight = imgFl.getHeight();

  if (m_size_x != bmpWidth || m_size_y != bmpHeight)
  {
    auto origin = origin_;
    // Middle of bitmap?
    if (origin.x == std::numeric_limits<double>::max())
    {
      origin = mrpt::math::TPoint2D(imgFl.getWidth() / 2.0, imgFl.getHeight() / 2.0);
    }

    // Resize grid:
    float new_x_max = static_cast<float>((static_cast<double>(imgFl.getWidth()) - origin.x) * res);
    float new_x_min = static_cast<float>(-origin.x * res);
    float new_y_max = static_cast<float>((static_cast<double>(imgFl.getHeight()) - origin.y) * res);
    float new_y_min = static_cast<float>(-origin.y * res);

    setSize(new_x_min, new_x_max, new_y_min, new_y_max, res);
  }

  // And load cells content:
  for (size_t x = 0; x < bmpWidth; x++)
    for (size_t y = 0; y < bmpHeight; y++)
    {
      float f = imgFl.getAsFloat({int(x), int(bmpHeight - 1 - y)});
      f = std::max(0.01f, f);
      f = std::min(0.99f, f);
      setCell(static_cast<int>(x), static_cast<int>(y), f);
    }

  m_is_empty = false;
  return true;

  MRPT_END
}

/*---------------------------------------------------------------
        saveAsBitmapTwoMapsWithCorrespondences
  ---------------------------------------------------------------*/
bool COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(
    const std::string& fileName,
    const COccupancyGridMap2D& m1,
    const COccupancyGridMap2D& m2,
    const TMatchingPairList& corrs)
{
  MRPT_START

  CImage img1, img2;
  unsigned int i, n, Ay1, Ay2;
  unsigned int px, py;

  // The individual maps:
  // ---------------------------------------------
  m1.getAsImage(img1);
  m2.getAsImage(img2);
  unsigned int lx1 = img1.getWidth();
  unsigned int ly1 = img1.getHeight();

  unsigned int lx2 = img2.getWidth();
  unsigned int ly2 = img2.getHeight();

  // The map with the lowest height has to be vertically aligned:
  if (ly1 > ly2)
  {
    Ay1 = 0;
    Ay2 = (ly1 - ly2) / 2;
  }
  else
  {
    Ay2 = 0;
    Ay1 = (ly2 - ly1) / 2;
  }

  // Compute the size of the composite image:
  // ---------------------------------------------
  CImage img(int32_t(lx1 + lx2 + 1), int32_t(max(ly1, ly2)), mrpt::img::CH_RGB);
  img.filledRectangle(
      {0, 0}, {int(img.getWidth() - 1), int(img.getHeight() - 1)},
      TColor::black());  // background: black
  img.drawImage({0, int(Ay1)}, img1);
  img.drawImage({int(lx1 + 1), int(Ay2)}, img2);

  // Draw the features:
  // ---------------------------------------------
  n = static_cast<unsigned int>(corrs.size());
  TColor lineColor = TColor::black();
  for (i = 0; i < n; i++)
  {
    // In M1:
    px = m1.x2idx(corrs[i].global.x);
    py = Ay1 + ly1 - 1 - m1.y2idx(corrs[i].global.y);
    img.rectangle({int(px - 10), int(py - 10)}, {int(px + 10), int(py + 10)}, lineColor);
    img.rectangle({int(px - 11), int(py - 11)}, {int(px + 11), int(py + 11)}, lineColor);

    // In M2:
    px = lx1 + 1 + m2.x2idx(corrs[i].local.x);
    py = Ay2 + ly2 - 1 - m2.y2idx(corrs[i].local.y);
    img.rectangle({int(px - 10), int(py - 10)}, {int(px + 10), int(py + 10)}, lineColor);
    img.rectangle({int(px - 11), int(py - 11)}, {int(px + 11), int(py + 11)}, lineColor);
  }

  // Draw the correspondences as lines:
  // ---------------------------------------------
  for (i = 0; i < n; i++)
  {
    lineColor = TColor(
        static_cast<uint8_t>(getRandomGenerator().drawUniform(0, 255.0f)),
        static_cast<uint8_t>(getRandomGenerator().drawUniform(0, 255.0f)),
        static_cast<uint8_t>(getRandomGenerator().drawUniform(0, 255.0f)));

    img.line(
        {int(m1.x2idx(corrs[i].global.x)), int(Ay1 + ly1 - 1 - m1.y2idx(corrs[i].global.y))},
        {int(lx1 + 1 + m2.x2idx(corrs[i].local.x)),
         int(Ay2 + ly2 - 1 - m2.y2idx(corrs[i].local.y))},
        lineColor);
  }  // i

  return img.saveToFile(fileName.c_str());

  MRPT_END
}

void COccupancyGridMap2D::saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const
{
  std::string fil(filNamePrefix + std::string(".png"));
  saveAsBitmapFile(fil);

  fil = filNamePrefix + std::string("_limits.txt");
  CMatrixF LIMITS(1, 4);
  LIMITS(0, 0) = m_xMin;
  LIMITS(0, 1) = m_xMax;
  LIMITS(0, 2) = m_yMin;
  LIMITS(0, 3) = m_yMax;
  LIMITS.saveToTextFile(
      fil, MATRIX_FORMAT_FIXED, false /* add mrpt header */,
      "% Grid limits: [x_min x_max y_min y_max]\n");
}

bool COccupancyGridMap2D::loadFromROSMapServerYAML(const std::string& yamlFilePath)
{
  // See format:
  // http://wiki.ros.org/map_server#YAML_format

  try
  {
    const auto f = mrpt::containers::yaml::FromFile(yamlFilePath);
    ASSERT_(f.isMap());
    ASSERT_(!f.empty());

    // relative to absolute:
    const auto imgFile = mrpt::system::pathJoin(
        {mrpt::system::extractFileDirectory(yamlFilePath), f["image"].as<std::string>()});

    ASSERT_FILE_EXISTS_(imgFile);

    const double resolution = f["resolution"].as<double>();
    ASSERT_(resolution > 0);

    const auto originPose = f["origin"].toStdVector<double>();
    ASSERT_GE_(originPose.size(), 2);

    const double xMin = originPose.at(0);
    const double yMin = originPose.at(1);

    const float occupied_thresh = f["occupied_thresh"].as<float>();
    const float free_thresh = f["free_thresh"].as<float>();
    const bool negate = f["negate"].as<bool>();
    const std::string mode = f.getOrDefault<std::string>("mode", "trinary");

    bool isScale = false;

    if (mode == "trinary")
      isScale = false;
    else if (mode == "scale")
      isScale = true;
    else
    {
      THROW_EXCEPTION_FMT(
          "Unsupported value for 'mode'='%s' (supported: 'trinary', "
          "'scale')",
          mode.c_str());
    }

    // 1st: load image and convert to float:
    const auto im = mrpt::img::CImage::LoadFromFile(imgFile);

    // For the precomputed likelihood:
    m_likelihoodCacheOutDated = true;

    const size_t w = im.getWidth(), h = im.getHeight();

    // Resize grid:
    float xMax = static_cast<float>(xMin + static_cast<double>(w) * resolution);
    float yMax = static_cast<float>(yMin + static_cast<double>(h) * resolution);

    setSize(
        static_cast<float>(xMin), xMax, static_cast<float>(yMin), yMax,
        static_cast<float>(resolution));

    // And load cells content:
    for (size_t y = 0; y < h; y++)
      for (size_t x = 0; x < w; x++)
      {
        float v = im.getAsFloat({int(x), int(h - 1 - y)});
        if (negate) v = 1.0f - v;
        v = std::max(0.01f, v);
        v = std::min(0.99f, v);

        if (isScale)
          setCell(static_cast<int>(x), static_cast<int>(y), v);
        else
        {
          if (1 - v > occupied_thresh)
            setCell(static_cast<int>(x), static_cast<int>(y), 0.0f);
          else if (1 - v < free_thresh)
            setCell(static_cast<int>(x), static_cast<int>(y), 1.0f);
          else
            setCell(static_cast<int>(x), static_cast<int>(y), 0.5f);
        }
      }

    m_is_empty = false;
    return true;  // ok
  }
  catch (const std::exception& e)
  {
    std::cerr << "[COccupancyGridMap2D::loadFromROSMapServerYAML] Error:\n" << e.what();
    return false;
  }
}
