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

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/img/DistortionModel.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/serialization/CSerializable.h>

#include <array>
#include <functional>  // hash

namespace mrpt::img
{
/** Intrinsic parameters for a pinhole or fisheye camera model, along with the
 *  associated lens distortion model.
 *
 * The camera projection follows the standard pinhole model:
 * - 3D point in camera frame: P_cam = (X, Y, Z)
 * - Normalized image coordinates: x = X/Z, y = Y/Z
 * - Apply distortion to get: (x_d, y_d)
 * - Project to pixel coordinates: u = fx*x_d + cx, v = fy*y_d + cy
 *
 * The distortion model is specified by the \a distortion field and can be:
 * - DistortionModel::none: No distortion (already rectified images)
 * - DistortionModel::plumb_bob: Standard radial-tangential distortion (OpenCV model)
 * - DistortionModel::kannala_brandt: Fish-eye camera model
 *
 * Parameters calibrated for one camera resolution can be scaled to other
 * resolutions using TCamera::scaleToResolution(), which preserves the aspect ratio.
 *
 * \sa The application camera-calib-gui for camera calibration
 * \sa mrpt::img::camera_geometry for projection and undistortion functions
 * \ingroup mrpt_img_grp
 */
class TCamera : public mrpt::serialization::CSerializable
{
  DEFINE_SERIALIZABLE(TCamera, mrpt::img)

 public:
  /** Default constructor: all intrinsic parameters set to zero. */
  TCamera();

  /** Parse camera parameters from YAML in OpenCV calibration format.
   * Refer to [this example](https://wiki.ros.org/camera_calibration_parsers#YAML).
   *
   * For supported distortion models see mrpt::img::DistortionModel
   */
  static TCamera FromYAML(const mrpt::containers::yaml& params);

  /** Export camera parameters as YAML in OpenCV calibration format.
   * Refer to [this example](http://wiki.ros.org/camera_calibration_parsers#YAML).
   */
  mrpt::containers::yaml asYAML() const;

  /** @name Camera parameters
    @{ */

  /** Image resolution in pixels */
  uint32_t ncols = 640, nrows = 480;

  /** Camera intrinsic matrix (calibration matrix) in the standard form:
   *
   *          [ fx   0  cx ]
   *      K = [  0  fy  cy ]
   *          [  0   0   1 ]
   *
   * where:
   * - fx, fy: focal length in pixels (horizontal and vertical)
   * - cx, cy: principal point coordinates in pixels
   *
   * See: https://www.mrpt.org/Camera_Parameters
   */
  mrpt::math::CMatrixDouble33 intrinsicParams;

  /** The lens distortion model type. Determines how to interpret the \a dist coefficients.
   */
  DistortionModel distortion = DistortionModel::none;

  /** Distortion coefficients. Interpretation depends on \a distortion:
   *
   * - **DistortionModel::none**: Coefficients ignored (rectified image)
   *
   * - **DistortionModel::plumb_bob**: Standard radial-tangential model (OpenCV compatible)
   *   - Format: [k1, k2, p1, p2, k3, k4, k5, k6]
   *   - k1, k2, k3, k4, k5, k6: Radial distortion coefficients
   *   - p1, p2: Tangential distortion coefficients
   *   - Distortion equations:
   *     - r² = x² + y²
   *     - x_d = x(1 + k1·r² + k2·r⁴ + k3·r⁶)/(1 + k4·r² + k5·r⁴ + k6·r⁶) + 2p1·xy + p2(r² + 2x²)
   *     - y_d = y(1 + k1·r² + k2·r⁴ + k3·r⁶)/(1 + k4·r² + k5·r⁴ + k6·r⁶) + p1(r² + 2y²) + 2p2·xy
   *
   * - **DistortionModel::kannala_brandt**: Fish-eye model
   *   - Format: [k1, k2, *, *, k3, k4, *, *] (indices 2,3,6,7 unused)
   *   - Distortion equations:
   *     - θ = atan(r), where r = sqrt(x² + y²)
   *     - θ_d = θ(1 + k1·θ² + k2·θ⁴ + k3·θ⁶ + k4·θ⁸)
   *     - x_d = (θ_d/r)·x, y_d = (θ_d/r)·y
   *
   * **Recommended**: Use getter/setter methods k1(), k2(), p1(), p2(), etc. to avoid indexing
   * errors.
   *
   * Default: all zeros (no distortion)
   */
  std::array<double, 8> dist{
      {.0, .0, .0, .0, .0, .0, .0, .0}
  };

  /** Physical focal length in meters (optional).
   * Can be used with \a intrinsicParams to compute physical pixel size.
   */
  double focalLengthMeters = .0;

  /** Optional descriptive camera name (e.g., "left_camera", "front_cam").
   */
  std::string cameraName = "camera1";

  /** @} */

  /** Scale all parameters to a new image resolution.
   * Throws an exception if the aspect ratio changes, as this would invalidate the calibration.
   * \param new_ncols New image width in pixels
   * \param new_nrows New image height in pixels
   */
  void scaleToResolution(unsigned int new_ncols, unsigned int new_nrows);

  /** Save camera parameters to a configuration file section in the format:
   *  \code
   *  [SECTION]
   *  resolution = [NCOLS NROWS]
   *  cx         = CX
   *  cy         = CY
   *  fx         = FX
   *  fy         = FY
   *  dist       = [K1 K2 P1 P2 K3 K4 K5 K6]
   *  camera_name = camera1
   *  distortion = {none|plumb_bob|kannala_brandt}
   *  focal_length = FOCAL_LENGTH  [optional]
   *  \endcode
   */
  void saveToConfigFile(const std::string& section, mrpt::config::CConfigFileBase& cfg) const;

  /** Load camera parameters from a configuration file section.
   * See saveToConfigFile() for the expected format.
   * \exception std::exception on missing required fields
   */
  void loadFromConfigFile(const std::string& section, const mrpt::config::CConfigFileBase& cfg);

  /** Overload with swapped parameter order (consistent with other MRPT APIs) */
  void loadFromConfigFile(const mrpt::config::CConfigFileBase& cfg, const std::string& section)
  {
    loadFromConfigFile(section, cfg);
  }

  /** Return all parameters as a formatted text block in INI file format.
   * \sa asYAML()
   */
  std::string dumpAsText() const;

  /** Set intrinsic matrix from individual focal length and principal point values.
   * \param fx Focal length in pixels (horizontal)
   * \param fy Focal length in pixels (vertical)
   * \param cx Principal point x-coordinate in pixels
   * \param cy Principal point y-coordinate in pixels
   */
  void setIntrinsicParamsFromValues(double fx, double fy, double cx, double cy)
  {
    intrinsicParams.setZero();
    intrinsicParams(0, 0) = fx;
    intrinsicParams(1, 1) = fy;
    intrinsicParams(0, 2) = cx;
    intrinsicParams(1, 2) = cy;
    intrinsicParams(2, 2) = 1.0;
  }

  /** Get distortion coefficients as a vector.
   * Returns an appropriately-sized vector based on \a distortion:
   * - DistortionModel::none: empty vector
   * - DistortionModel::plumb_bob: 5 or 8 elements
   * - DistortionModel::kannala_brandt: 4 elements
   */
  std::vector<double> getDistortionParamsAsVector() const;

  /** Equivalent to getDistortionParamsAsVector() but returns a row matrix */
  [[nodiscard]] mrpt::math::CMatrixDouble getDistortionParamsAsRowVector() const;

  /** Set distortion coefficients from a vector.
   * \param distParVector Vector of 4, 5, or 8 distortion coefficients
   */
  template <class VECTORLIKE>
  void setDistortionParamsVector(const VECTORLIKE& distParVector)
  {
    auto N = static_cast<size_t>(distParVector.size());
    ASSERT_(N == 4 || N == 5 || N == 8);
    dist.fill(0);  // Default values
    for (size_t i = 0; i < N; i++)
    {
      dist[i] = distParVector[i];
    }
  }

  /** Configure plumb_bob distortion model with specified parameters.
   * \param k1_ First radial distortion coefficient
   * \param k2_ Second radial distortion coefficient
   * \param p1_ First tangential distortion coefficient
   * \param p2_ Second tangential distortion coefficient
   * \param k3_ Third radial distortion coefficient (optional, default=0)
   */
  void setDistortionPlumbBob(double k1_, double k2_, double p1_, double p2_, double k3_ = 0)
  {
    distortion = DistortionModel::plumb_bob;
    dist.fill(0);
    k1(k1_);
    k2(k2_);
    k3(k3_);
    p1(p1_);
    p2(p2_);
  }

  /** Configure Kannala-Brandt fish-eye distortion model.
   * \param k1_ First distortion coefficient
   * \param k2_ Second distortion coefficient
   * \param k3_ Third distortion coefficient
   * \param k4_ Fourth distortion coefficient
   */
  void setDistortionKannalaBrandt(double k1_, double k2_, double k3_, double k4_)
  {
    distortion = DistortionModel::kannala_brandt;
    dist.fill(0);
    k1(k1_);
    k2(k2_);
    k3(k3_);
    k4(k4_);
  }

  /** @name Intrinsic parameter accessors
   * Convenience methods for getting/setting individual intrinsic parameters
   * @{ */

  [[nodiscard]] double cx() const { return intrinsicParams(0, 2); }
  [[nodiscard]] double cy() const { return intrinsicParams(1, 2); }
  [[nodiscard]] double fx() const { return intrinsicParams(0, 0); }
  [[nodiscard]] double fy() const { return intrinsicParams(1, 1); }

  void cx(double val) { intrinsicParams(0, 2) = val; }
  void cy(double val) { intrinsicParams(1, 2) = val; }
  void fx(double val) { intrinsicParams(0, 0) = val; }
  void fy(double val) { intrinsicParams(1, 1) = val; }

  /** @} */

  /** @name Distortion coefficient accessors
   * Convenience methods for getting/setting individual distortion coefficients.
   * Interpretation depends on the \a distortion model.
   * @{ */

  [[nodiscard]] double k1() const { return dist[0]; }
  [[nodiscard]] double k2() const { return dist[1]; }
  [[nodiscard]] double p1() const { return dist[2]; }
  [[nodiscard]] double p2() const { return dist[3]; }
  [[nodiscard]] double k3() const { return dist[4]; }
  [[nodiscard]] double k4() const { return dist[5]; }
  [[nodiscard]] double k5() const { return dist[6]; }
  [[nodiscard]] double k6() const { return dist[7]; }

  void k1(double val) { dist[0] = val; }
  void k2(double val) { dist[1] = val; }
  void p1(double val) { dist[2] = val; }
  void p2(double val) { dist[3] = val; }
  void k3(double val) { dist[4] = val; }
  void k4(double val) { dist[5] = val; }
  void k5(double val) { dist[6] = val; }
  void k6(double val) { dist[7] = val; }

  /** @} */

};  // end class TCamera

bool operator==(const mrpt::img::TCamera& a, const mrpt::img::TCamera& b);
bool operator!=(const mrpt::img::TCamera& a, const mrpt::img::TCamera& b);

}  // namespace mrpt::img

namespace std
{
template <>
struct hash<mrpt::img::TCamera>
{
  size_t operator()(const mrpt::img::TCamera& k) const
  {
    size_t res = 17;
    res = res * 31 + hash<double>()(k.cx());
    res = res * 31 + hash<double>()(k.cy());
    res = res * 31 + hash<double>()(k.fx());
    res = res * 31 + hash<double>()(k.fy());
    res = res * 31 + hash<uint32_t>()(k.ncols);
    res = res * 31 + hash<uint32_t>()(k.nrows);
    for (unsigned int i = 0; i < k.dist.size(); i++)
    {
      res = res * 31 + hash<double>()(k.dist[i]);
    }
    res = res * 31 + hash<std::string>()(k.cameraName);
    res = res * 31 + hash<double>()(k.focalLengthMeters);
    res = res * 31 + hash<uint8_t>()(static_cast<uint8_t>(k.distortion));
    return res;
  }
};

}  // namespace std