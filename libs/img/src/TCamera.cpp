/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers
//
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/matrix_serialization.h>	 // For "<<" ">>" operators.
#include <mrpt/math/utils_matlab.h>

using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

/* Implements serialization for the TCamera struct as it will be included within
 * CObservations objects */
IMPLEMENTS_SERIALIZABLE(TCamera, CSerializable, mrpt::img)

TCamera::TCamera()
{
	// Ensure intrinsics matrix has a 1 in the bottom-right corner:
	setIntrinsicParamsFromValues(0, 0, 0, 0);
}

/** Dumps all the parameters as a multi-line string, with the same format than
 * \a saveToConfigFile.  \sa saveToConfigFile */
std::string TCamera::dumpAsText() const
{
	mrpt::config::CConfigFileMemory cfg;
	saveToConfigFile("", cfg);
	return cfg.getContent();
}

uint8_t TCamera::serializeGetVersion() const { return 6; }
void TCamera::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << focalLengthMeters;
	// v3: from 5 to 8 dist params:
	for (size_t k = 0; k < dist.size(); k++)
		out << dist[k];
	// v4: only store the 4 relevant values:
	out << fx() << fy() << cx() << cy();
	// version 0 did serialize here a "CMatrixDouble15"
	out << nrows << ncols;	// New in v2
	out << cameraName;	// v5
	out << static_cast<uint8_t>(distortion);  // v6
}
void TCamera::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		{
			in >> focalLengthMeters;

			dist.fill(0);
			for (unsigned int k = 0; k < 5; k++)
				in >> dist[k];
			if (version >= 3)
				for (unsigned int k = 5; k < 8; k++)
					in >> dist[k];

			if (version < 4)
			{
				in >> intrinsicParams;
				// Enforce values with fixed 0 or 1 values:
				intrinsicParams(0, 1) = 0;
				intrinsicParams(1, 0) = 0;
				intrinsicParams(2, 0) = 0;
				intrinsicParams(2, 1) = 0;
				intrinsicParams(2, 2) = 1;
			}
			else
			{
				double vfx, vfy, vcx, vcy;
				in >> vfx >> vfy >> vcx >> vcy;
				setIntrinsicParamsFromValues(vfx, vfy, vcx, vcy);
			}

			if (version == 0)
			{
				CMatrixDouble15 __distortionParams;
				in >> __distortionParams;
			}

			if (version >= 2) in >> nrows >> ncols;
			else
			{
				nrows = 480;
				ncols = 640;
			}
			if (version >= 5) in >> cameraName;

			if (version >= 6)
			{
				distortion = static_cast<DistortionModel>(in.ReadAs<uint8_t>());
			}
			else
			{
				distortion = DistortionModel::plumb_bob;
			}
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM(mrpt::img::TCamera)
#endif

mxArray* TCamera::writeToMatlab() const
{
#if MRPT_HAS_MATLAB
	const char* fields[] = {"K", "dist", "f", "ncols", "nrows"};
	mexplus::MxArray params_struct(
		mexplus::MxArray::Struct(sizeof(fields) / sizeof(fields[0]), fields));
	params_struct.set("K", mrpt::math::convertToMatlab(this->intrinsicParams));
	params_struct.set("dist", mrpt::math::convertVectorToMatlab(this->dist));
	params_struct.set("f", this->focalLengthMeters);
	params_struct.set("ncols", this->ncols);
	params_struct.set("nrows", this->nrows);
	return params_struct.release();
#else
	THROW_EXCEPTION("MRPT built without MATLAB/Mex support");
#endif
}

/**  Save as a config block:
 *  \code
 *  [SECTION]
 *  resolution = NCOLS NROWS
 *  cx         = CX
 *  cy         = CY
 *  fx         = FX
 *  fy         = FY
 *  dist       = K1 K2 T1 T2 T3
 *  focal_length = FOCAL_LENGTH
 *  \endcode
 */
void TCamera::saveToConfigFile(
	const std::string& section, mrpt::config::CConfigFileBase& cfg) const
{
	cfg.write(section, "camera_name", cameraName);
	cfg.write(
		section, "resolution",
		format("[%u %u]", (unsigned int)ncols, (unsigned int)nrows));
	cfg.write(section, "cx", format("%.05f", cx()));
	cfg.write(section, "cy", format("%.05f", cy()));
	cfg.write(section, "fx", format("%.05f", fx()));
	cfg.write(section, "fy", format("%.05f", fy()));

	cfg.write(section, "distortion_model", distortion);
	cfg.write(
		section, "dist", getDistortionParamsAsRowVector().inMatlabFormat());

	if (focalLengthMeters != 0)
		cfg.write(section, "focal_length", focalLengthMeters);
}

/**  Load all the params from a config source, in the format described in
 * saveToConfigFile()
 */
void TCamera::loadFromConfigFile(
	const std::string& section, const mrpt::config::CConfigFileBase& cfg)
{
	*this = TCamera();	// reset to defaults

	vector<uint64_t> out_res;
	cfg.read_vector(section, "resolution", vector<uint64_t>(), out_res, true);
	if (out_res.size() != 2)
		THROW_EXCEPTION("Expected 2-length vector in field 'resolution'");
	ncols = out_res[0];
	nrows = out_res[1];

	double fx, fy, cx, cy;
	fx = cfg.read_double(section, "fx", 0, true);
	fy = cfg.read_double(section, "fy", 0, true);
	cx = cfg.read_double(section, "cx", 0, true);
	cy = cfg.read_double(section, "cy", 0, true);

	if (fx < 2.0) fx *= ncols;
	if (fy < 2.0) fy *= nrows;
	if (cx < 2.0) cx *= ncols;
	if (cy < 2.0) cy *= nrows;

	setIntrinsicParamsFromValues(fx, fy, cx, cy);

	distortion = cfg.read_enum(section, "distortion_model", distortion);

	CVectorDouble v;
	cfg.read_vector(section, "dist", CVectorDouble(), v, true);

	switch (distortion)
	{
		case DistortionModel::none: break;
		case DistortionModel::plumb_bob:
			ASSERTMSG_(
				v.size() == 5 || v.size() == 8,
				"Expected 5 or 8 distortion parameters for plumb_bob");
			for (int i = 0; i < v.size(); i++)
				dist[i] = v[i];
			break;
		case DistortionModel::kannala_brandt:
			ASSERTMSG_(
				v.size() == 4,
				"Expected 4 distortion parameters for kannala_brandt");
			k1(v[0]);
			k2(v[1]);
			k3(v[2]);
			k4(v[3]);
			break;
		default:
		{
			THROW_EXCEPTION("Invalid distortion model enum value.");
		}
	}

	focalLengthMeters =
		cfg.read_double(section, "focal_length", 0, false /* optional value */);

	cameraName = cfg.read_string(
		section, "camera_name", cameraName, false /* optional value */);
}

/** Rescale all the parameters for a new camera resolution (it raises an
 * exception if the aspect ratio is modified, which is not permitted).
 */
void TCamera::scaleToResolution(unsigned int new_ncols, unsigned int new_nrows)
{
	if (ncols == new_ncols && nrows == new_nrows) return;  // already done

	ASSERT_(new_nrows > 0 && new_ncols > 0);

	const double prev_aspect_ratio = ncols / double(nrows);
	const double new_aspect_ratio = new_ncols / double(new_nrows);

	ASSERTMSG_(
		std::abs(prev_aspect_ratio - new_aspect_ratio) < 1e-3,
		"TCamera: Trying to scale camera parameters for a resolution of "
		"different aspect ratio.");

	const double K = new_ncols / double(ncols);

	ncols = new_ncols;
	nrows = new_nrows;

	// fx fy cx cy
	intrinsicParams(0, 0) *= K;
	intrinsicParams(1, 1) *= K;
	intrinsicParams(0, 2) *= K;
	intrinsicParams(1, 2) *= K;

	// distortion params: unmodified.
}

bool mrpt::img::operator==(
	const mrpt::img::TCamera& a, const mrpt::img::TCamera& b)
{
	return a.ncols == b.ncols && a.nrows == b.nrows &&
		a.intrinsicParams == b.intrinsicParams && a.dist == b.dist &&
		a.focalLengthMeters == b.focalLengthMeters &&
		a.cameraName == b.cameraName && a.distortion == b.distortion;
}
bool mrpt::img::operator!=(
	const mrpt::img::TCamera& a, const mrpt::img::TCamera& b)
{
	return !(a == b);
}

TCamera TCamera::FromYAML(const mrpt::containers::yaml& p)
{
	TCamera c;

	c.ncols = p["image_width"].as<uint32_t>();
	c.nrows = p["image_height"].as<uint32_t>();

	c.cameraName = p["camera_name"].as<std::string>();
	p["camera_matrix"].toMatrix(c.intrinsicParams);

	{
		DistortionModel distortion_model = c.distortion;
		MCP_LOAD_OPT(p, distortion_model);
		c.distortion = distortion_model;
	}

	mrpt::math::CMatrixDouble v;
	p["distortion_coefficients"].toMatrix(v);

	switch (c.distortion)
	{
		case DistortionModel::none: break;
		case DistortionModel::plumb_bob:
			ASSERTMSG_(
				v.size() == 5 || v.size() == 8,
				"Expected 5 or 8 distortion parameters for plumb_bob");
			for (size_t i = 0; i < v.size(); i++)
				c.dist[i] = v[i];
			break;
		case DistortionModel::kannala_brandt:
			ASSERTMSG_(
				v.size() == 4,
				"Expected 4 distortion parameters for kannala_brandt");
			c.k1(v[0]);
			c.k2(v[1]);
			c.k3(v[2]);
			c.k4(v[3]);
			break;
		default:
		{
			THROW_EXCEPTION("Invalid distortion model enum value.");
		}
	}

	c.focalLengthMeters = p.getOrDefault<double>("focal_length_meters", 0.0);

	return c;
}

/* Example:
 ( From: http://wiki.ros.org/camera_calibration_parsers#YAML )

image_width: 2448
image_height: 2050
camera_name: prosilica
camera_matrix:
  rows: 3
  cols: 3
  data: [4827.94, 0, 1223.5, 0, 4835.62, 1024.5, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41527, 0.31874, -0.00197, 0.00071, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [4827.94, 0, 1223.5, 0, 0, 4835.62, 1024.5, 0, 0, 0, 1, 0]

*/
mrpt::containers::yaml TCamera::asYAML() const
{
	mrpt::containers::yaml p = mrpt::containers::yaml::Map();

	p["image_width"] = ncols;
	p["image_height"] = nrows;
	p["camera_name"] = cameraName;
	p["camera_matrix"] = mrpt::containers::yaml::FromMatrix(intrinsicParams);
	p["distortion_model"] = mrpt::typemeta::enum2str(distortion);
	p["distortion_coefficients"] =
		mrpt::containers::yaml::FromMatrix(getDistortionParamsAsRowVector());
	p["rectification_matrix"] = mrpt::containers::yaml::FromMatrix(
		mrpt::math::CMatrixDouble33::Identity());

	mrpt::math::CMatrixDouble proj(3, 4);
	proj.setZero();
	proj.insertMatrix(0, 0, intrinsicParams);
	p["projection_matrix"] = mrpt::containers::yaml::FromMatrix(proj);

	p["focal_length_meters"] = focalLengthMeters;

	return p;
}

std::vector<double> TCamera::getDistortionParamsAsVector() const
{
	switch (distortion)
	{
		case DistortionModel::none: return {};
		case DistortionModel::plumb_bob:
			return {k1(), k2(), p1(), p2(), k3(), k4(), k5(), k6()};
		case DistortionModel::kannala_brandt: return {k1(), k2(), k3(), k4()};
		default:
		{
			THROW_EXCEPTION("Invalid distortion model enum value.");
		}
	}
}

mrpt::math::CMatrixDouble TCamera::getDistortionParamsAsRowVector() const
{
	const auto vals = getDistortionParamsAsVector();
	mrpt::math::CMatrixDouble v;
	if (!vals.empty())
	{
		v.resize(1, vals.size());
		for (size_t i = 0; i < vals.size(); i++)
			v(0, i) = vals[i];
	}
	return v;
}
