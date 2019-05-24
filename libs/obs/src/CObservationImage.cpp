/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/math/ops_vectors.h>  // << of std::vector()
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/serialization/CArchive.h>
#include <Eigen/Dense>
#include <iostream>
#if MRPT_HAS_MATLAB
#include <mexplus/mxarray.h>
#endif

using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::img;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationImage, CObservation, mrpt::obs)

uint8_t CObservationImage::serializeGetVersion() const { return 4; }
void CObservationImage::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << cameraPose << cameraParams << image << timestamp << sensorLabel;
}

void CObservationImage::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		{
			in >> cameraPose;

			if (version >= 4)
			{
				in >> cameraParams;
			}
			else
			{
				CMatrixF intrinsicParams, distortionParams;
				in >> distortionParams >> intrinsicParams;

				if (distortionParams.rows() == 1 &&
					distortionParams.cols() == 5)
				{
					CMatrixDouble15 p;
					p = distortionParams.cast_double();
					cameraParams.setDistortionParamsVector(p);
				}
				else
					cameraParams.dist.fill(0);

				cameraParams.intrinsicParams = mrpt::math::CMatrixDouble33(
					intrinsicParams.block<3, 3>(0, 0).cast<double>());
			}

			in >> image;

			if (version >= 1) in >> timestamp;

			if (version >= 2)
			{
				if (version < 4) in >> cameraParams.focalLengthMeters;
			}
			else
				cameraParams.focalLengthMeters = 0.002;

			if (version >= 3)
				in >> sensorLabel;
			else
				sensorLabel = "";
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM(mrpt::obs::CObservationImage)
#endif

mxArray* CObservationImage::writeToMatlab() const
{
#if MRPT_HAS_MATLAB
	const char* fields[] = {"class", "ts",   "sensorLabel",
							"image", "pose", "params"};
	mexplus::MxArray obs_struct(
		mexplus::MxArray::Struct(sizeof(fields) / sizeof(fields[0]), fields));

	obs_struct.set("class", this->GetRuntimeClass()->className);
	obs_struct.set("ts", this->timestamp);
	obs_struct.set("sensorLabel", this->sensorLabel);
	obs_struct.set("image", this->image);
	obs_struct.set("pose", this->cameraPose);
	obs_struct.set("params", this->cameraParams);
	return obs_struct.release();
#else
	THROW_EXCEPTION("MRPT built without MATLAB/Mex support");
#endif
}

void CObservationImage::getUndistortedImage(CImage& out_img) const
{
	image.undistort(out_img, cameraParams);
}

void CObservationImage::getDescriptionAsText(std::ostream& o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot "
		 "base:\n";
	o << cameraPose.getHomogeneousMatrixVal<CMatrixDouble44>() << cameraPose
	  << endl;

	o << format(
		"Focal length: %.03f mm\n", cameraParams.focalLengthMeters * 1000);

	o << "Intrinsic parameters matrix for the camera:" << endl
	  << cameraParams.intrinsicParams.inMatlabFormat() << endl
	  << cameraParams.intrinsicParams << endl;

	o << "Distorsion parameters for the camera: "
	  << cameraParams.getDistortionParamsAsVector() << endl;

	if (image.isExternallyStored())
		o << " Image is stored externally in file: "
		  << image.getExternalStorageFile() << endl;

	o << format(
		" Image size: %ux%u pixels\n", (unsigned int)image.getWidth(),
		(unsigned int)image.getHeight());

	o << " Channels order: " << image.getChannelsOrder() << endl;

	o << format(
		" Rows are stored in top-bottom order: %s\n",
		image.isOriginTopLeft() ? "YES" : "NO");
}
