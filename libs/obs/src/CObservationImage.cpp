/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationImage.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/math/ops_vectors.h>  // << of std::vector()
#include <iostream>
#if MRPT_HAS_MATLAB
#	include <mexplus/mxarray.h>
#endif

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationImage, CObservation,mrpt::obs)

/** Constructor
 */
CObservationImage::CObservationImage( void *iplImage  ) :
	cameraPose(),
	image( iplImage )
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationImage::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 4;
	else
	{
		// The data
		out << cameraPose << cameraParams << image
		    << timestamp
		    << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationImage::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		{
			in >> cameraPose;

			if (version>=4)
			{
				in >> cameraParams;
			}
			else
			{
				CMatrix	 intrinsicParams, distortionParams;
				in >> distortionParams >> intrinsicParams;

				if (size(distortionParams,1)==1 && size(distortionParams,2)==5)
				{
					const CMatrixDouble15 p = distortionParams.cast<double>();
					cameraParams.setDistortionParamsVector(p);
				}
				else 	cameraParams.dist.assign(0);

				cameraParams.intrinsicParams = intrinsicParams.block(0,0,3,3).cast<double>();
			}

			in >> image;

			if (version>=1)
				in >> timestamp;

			if (version>=2)
			{
				if (version<4)
					in >> cameraParams.focalLengthMeters ;
			}
			else
				cameraParams.focalLengthMeters = 0.002;

			if (version>=3)
					in >> sensorLabel;
			else	sensorLabel = "";

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM( mrpt::obs::CObservationImage )

mxArray* CObservationImage::writeToMatlab() const
{
	const char* fields[] = {"class","ts","sensorLabel","image","pose","params"};
	mexplus::MxArray obs_struct( mexplus::MxArray::Struct(sizeof(fields)/sizeof(fields[0]),fields) );

	obs_struct.set("class", this->GetRuntimeClass()->className);
	obs_struct.set("ts", this->timestamp);
	obs_struct.set("sensorLabel", this->sensorLabel);
	obs_struct.set("image", this->image);
	obs_struct.set("pose", this->cameraPose);
	obs_struct.set("params", this->cameraParams);
	return obs_struct.release();
}
#endif

/*---------------------------------------------------------------
						getRectifiedImage
 ---------------------------------------------------------------*/
void  CObservationImage::getRectifiedImage( CImage &out_img ) const
{
	image.rectifyImage(out_img, cameraParams );
}

void CObservationImage::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << cameraPose.getHomogeneousMatrixVal()
	<< cameraPose << endl;

	o << format("Focal length: %.03f mm\n",cameraParams.focalLengthMeters*1000);

	o << "Intrinsic parameters matrix for the camera:"<< endl
	<< cameraParams.intrinsicParams.inMatlabFormat() << endl << cameraParams.intrinsicParams << endl;

	o << "Distorsion parameters for the camera: " << cameraParams.getDistortionParamsAsVector() << endl;

	if (image.isExternallyStored())
		o << " Image is stored externally in file: " << image.getExternalStorageFile() << endl;

	o << format(" Image size: %ux%u pixels\n", (unsigned int)image.getWidth(), (unsigned int)image.getHeight() );

	o << " Channels order: " << image.getChannelsOrder() << endl;

	o << format(" Rows are stored in top-bottom order: %s\n",
					image.isOriginTopLeft() ? "YES" : "NO");

}
