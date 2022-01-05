/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
/** Intrinsic parameters for a pinhole or fisheye camera model, plus its
 *  associated lens distortion model. The type of camera distortion is defined
 *  by the \a distortion field.
 *
 * Parameters for one camera resolution can be used for any other resolutions by
 * means of the method TCamera::scaleToResolution()
 *
 * \sa The application camera-calib-gui for calibrating a camera
 * \ingroup mrpt_img_grp
 */
class TCamera : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(TCamera, mrpt::img)

	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION

   public:
	/** Default ctor: all intrinsic parameters set to zero. */
	TCamera();

	/** Parse from yaml, in OpenCV calibration model.
	 * Refer to
	 * [this example](https://wiki.ros.org/camera_calibration_parsers#YAML).
	 *
	 * For known distortion models see mrpt::img::DistortionModel
	 *
	 */
	static TCamera FromYAML(const mrpt::containers::yaml& params);

	/** Stores as yaml, in OpenCV calibration model.
	 * Refer to
	 * [this example](http://wiki.ros.org/camera_calibration_parsers#YAML).
	 */
	mrpt::containers::yaml asYAML() const;

	/** @name Camera parameters
		@{ */

	/** Camera resolution */
	uint32_t ncols = 640, nrows = 480;

	/** Camera matrix (intrinsic parameters), containing the focal length and
	 * principal point coordinates:
	 *
	 *          [ fx  0 cx ]
	 *      A = [  0 fy cy ]
	 *          [  0  0  1 ]
	 *
	 */
	mrpt::math::CMatrixDouble33 intrinsicParams;

	/** The distortion model: none (already rectified), plumb_bob (common
	 * pin-hole radial and tangential distortion), or kannala_brandt (for
	 * fish-eye cameras) */
	DistortionModel distortion = DistortionModel::none;

	/** Depending on the distortion model stored in \a distortion, the indices
	 *  of this array mean:
	 *  - DistortionModel::none: Coefficients are ignored.
	 *  - DistortionModel::`: [k1 k2 t1 t2 k3 k4 k5 k6] -> k_i:
	 * parameters of radial distortion, t_i: parameters of tangential
	 * distortion.
	 *   - DistortionModel::kannala_brandt: [k1 k2 * * k3 k4] -> k_i: model
	 * coefficients.
	 *
	 * It is recommended to access the specific terms via the getters k1(),
	 * k2(),p1(),... to avoid mistakes.
	 *
	 * Default values is all to zero.
	 */
	std::array<double, 8> dist{{.0, .0, .0, .0, .0, .0, .0, .0}};

	/** The focal length of the camera, in meters (can be used among
	 * 'intrinsicParams' to determine the pixel size). */
	double focalLengthMeters = .0;

	/** Optional camera name. \note (New in MRPT 2.1.8) */
	std::string cameraName = "camera1";

	/** @} */

	/** Rescale all the parameters for a new camera resolution (it raises an
	 * exception if the aspect ratio is modified, which is not permitted).
	 */
	void scaleToResolution(unsigned int new_ncols, unsigned int new_nrows);

	/**  Save as a config block:
	 *  \code
	 *  [SECTION]
	 *  resolution = [NCOLS NROWS]
	 *  cx         = CX
	 *  cy         = CY
	 *  fx         = FX
	 *  fy         = FY
	 *  dist       = [K1 K2 ...]
	 *  camera_name = camera1
	 *  distortion = {none|plumb_bob|kannala_brandt}
	 *  \endcode
	 */
	void saveToConfigFile(
		const std::string& section, mrpt::config::CConfigFileBase& cfg) const;

	/**  Load all the params from a config source, in the format used in
	 * saveToConfigFile(), that is:
	 *
	 *  \code
	 *  [SECTION]
	 *  resolution = [NCOLS NROWS]
	 *  cx         = CX
	 *  cy         = CY
	 *  fx         = FX
	 *  fy         = FY
	 *  dist       = { [K1 K2 T1 T2 K3] | [K1 K2 K3 K4]}
	 *  focal_length = FOCAL_LENGTH  [optional]
	 *  camera_name = camera1 [optional]
	 *  distortion = {none|plumb_bob|kannala_brandt}
	 *  \endcode
	 *  \exception std::exception on missing fields
	 */
	void loadFromConfigFile(
		const std::string& section, const mrpt::config::CConfigFileBase& cfg);

	/** overload This signature is consistent with the rest of MRPT APIs */
	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& cfg, const std::string& section)
	{
		loadFromConfigFile(section, cfg);
	}

	/** Returns all parameters as a text block in the INI-file format.
	 *  \sa asYAML()
	 */
	std::string dumpAsText() const;

	/** Set the matrix of intrinsic params of the camera from the individual
	 * values of focal length and principal point coordinates (in pixels)
	 */
	inline void setIntrinsicParamsFromValues(
		double fx, double fy, double cx, double cy)
	{
		intrinsicParams.setZero();
		intrinsicParams(0, 0) = fx;
		intrinsicParams(1, 1) = fy;
		intrinsicParams(0, 2) = cx;
		intrinsicParams(1, 2) = cy;
		intrinsicParams(2, 2) = 1.0;
	}

	/** Get a vector with the distortion params of the camera, which depending
	 * on the distortion model may be an empty vector (Distortion::none),
	 * an 8-vector (Distortion::plumb_bob) or a 4-vector
	 * (Distortion::kannala_brandt)
	 */
	std::vector<double> getDistortionParamsAsVector() const;

	/** Equivalent to getDistortionParamsAsVector()  */
	mrpt::math::CMatrixDouble getDistortionParamsAsRowVector() const;

	/** Set the whole vector of distortion params of the camera from a 4, 5, or
	 * 8-vector (see definition of \a dist for parameter order) */
	template <class VECTORLIKE>
	void setDistortionParamsVector(const VECTORLIKE& distParVector)
	{
		auto N = static_cast<size_t>(distParVector.size());
		ASSERT_(N == 4 || N == 5 || N == 8);
		dist.fill(0);  // Default values
		for (size_t i = 0; i < N; i++)
			dist[i] = distParVector[i];
	}

	/** Defines the distortion model and its parameters, in one call. */
	void setDistortionPlumbBob(
		double k1_, double k2_, double p1_, double p2_, double k3_ = 0)
	{
		distortion = DistortionModel::plumb_bob;
		dist.fill(0);
		k1(k1_);
		k2(k2_);
		k3(k3_);
		p1(p1_);
		p2(p2_);
	}

	/** Defines the distortion model and its parameters, in one call. */
	void setDistortionKannalaBrandt(
		double k1_, double k2_, double k3_, double k4_)
	{
		distortion = DistortionModel::kannala_brandt;
		dist.fill(0);
		k1(k1_);
		k2(k2_);
		k3(k3_);
		k4(k4_);
	}

	/** Get the value of the principal point x-coordinate (in pixels). */
	inline double cx() const { return intrinsicParams(0, 2); }
	/** Get the value of the principal point y-coordinate  (in pixels). */
	inline double cy() const { return intrinsicParams(1, 2); }
	/** Get the value of the focal length x-value (in pixels). */
	inline double fx() const { return intrinsicParams(0, 0); }
	/** Get the value of the focal length y-value (in pixels). */
	inline double fy() const { return intrinsicParams(1, 1); }
	/** Set the value of the principal point x-coordinate (in pixels). */
	inline void cx(double val) { intrinsicParams(0, 2) = val; }
	/** Set the value of the principal point y-coordinate  (in pixels). */
	inline void cy(double val) { intrinsicParams(1, 2) = val; }
	/** Set the value of the focal length x-value (in pixels). */
	inline void fx(double val) { intrinsicParams(0, 0) = val; }
	/** Set the value of the focal length y-value (in pixels). */
	inline void fy(double val) { intrinsicParams(1, 1) = val; }
	/** Get the value of the k1 distortion parameter.  */
	inline double k1() const { return dist[0]; }
	/** Get the value of the k2 distortion parameter.  */
	inline double k2() const { return dist[1]; }
	/** Get the value of the p1 distortion parameter.  */
	inline double p1() const { return dist[2]; }
	/** Get the value of the p2 distortion parameter.  */
	inline double p2() const { return dist[3]; }
	/** Get the value of the k3 distortion parameter.  */
	inline double k3() const { return dist[4]; }
	/** Get the value of the k4 distortion parameter.  */
	inline double k4() const { return dist[5]; }
	/** Get the value of the k5 distortion parameter.  */
	inline double k5() const { return dist[6]; }
	/** Get the value of the k6 distortion parameter.  */
	inline double k6() const { return dist[7]; }

	/** Set the value of the k1 distortion parameter.  */
	inline void k1(double val) { dist[0] = val; }
	/** Set the value of the k2 distortion parameter.  */
	inline void k2(double val) { dist[1] = val; }
	/** Set the value of the p1 distortion parameter.  */
	inline void p1(double val) { dist[2] = val; }
	/** Set the value of the p2 distortion parameter.  */
	inline void p2(double val) { dist[3] = val; }
	/** Set the value of the k3 distortion parameter.  */
	inline void k3(double val) { dist[4] = val; }
	/** Set the value of the k4 distortion parameter.  */
	inline void k4(double val) { dist[5] = val; }
	/** Set the value of the k5 distortion parameter.  */
	inline void k5(double val) { dist[6] = val; }
	/** Set the value of the k6 distortion parameter.  */
	inline void k6(double val) { dist[7] = val; }

};	// end class TCamera

bool operator==(const mrpt::img::TCamera& a, const mrpt::img::TCamera& b);
bool operator!=(const mrpt::img::TCamera& a, const mrpt::img::TCamera& b);

}  // namespace mrpt::img
// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::img::TCamera)  // Not working at the beginning?

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
			res = res * 31 + hash<double>()(k.dist[i]);
		res = res * 31 + hash<std::string>()(k.cameraName);
		res = res * 31 + hash<double>()(k.focalLengthMeters);
		res = res * 31 + hash<uint8_t>()(static_cast<uint8_t>(k.distortion));
		return res;
	}
};

}  // namespace std
