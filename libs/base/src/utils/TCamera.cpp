/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/math/matrix_serialization.h>  // For "<<" ">>" operators.
#include <mrpt/math/utils_matlab.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


/* Implements serialization for the TCamera struct as it will be included within CObservations objects */
IMPLEMENTS_SERIALIZABLE( TCamera, CSerializable, mrpt::utils )

/** Dumps all the parameters as a multi-line string, with the same format than \a saveToConfigFile.  \sa saveToConfigFile */
std::string TCamera::dumpAsText() const
{
	mrpt::utils::CConfigFileMemory cfg;
	saveToConfigFile("",cfg);
	return cfg.getContent();
}


// WriteToStream
void TCamera::writeToStream( CStream &out, int *version ) const
{
	if( version )
		*version = 2;
	else
	{
		out << focalLengthMeters;
		for(unsigned int k = 0; k < 5; k++) out << dist[k];
		out << intrinsicParams;
		// version 0 did serialize here a "CMatrixDouble15"
		out << nrows << ncols; // New in v2
	} // end else
}

// ReadFromStream
void TCamera::readFromStream( CStream &in, int version )
{
	switch( version )
	{
	case 0:
	case 1:
	case 2:
		{
			in >> focalLengthMeters;

			for(unsigned int k = 0; k < 5; k++)
				in >> dist[k];

			in >> intrinsicParams;

			if (version==0)
			{
				CMatrixDouble15 __distortionParams;
				in >> __distortionParams;
			}

			if (version>=2)
				in >> nrows >> ncols;
			else {
				nrows = 480;
				ncols = 640;
			}


		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION( version )
	}
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM( mrpt::utils::TCamera )

mxArray* TCamera::writeToMatlab() const
{
	const char* fields[] = {"K","dist","f","ncols","nrows"};
	mexplus::MxArray params_struct( mexplus::MxArray::Struct(sizeof(fields)/sizeof(fields[0]),fields) );
	params_struct.set("K", mrpt::math::convertToMatlab(this->intrinsicParams));
	params_struct.set("dist", mrpt::math::convertToMatlab(this->dist));
	params_struct.set("f", this->focalLengthMeters);
	params_struct.set("ncols", this->ncols);
	params_struct.set("nrows", this->nrows);
	return params_struct.release();
}
#endif

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
void TCamera::saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const
{
	cfg.write(section,"resolution",format("[%u %u]",(unsigned int)ncols,(unsigned int)nrows));
	cfg.write(section,"cx", format("%.05f",cx()) );
	cfg.write(section,"cy", format("%.05f",cy()) );
	cfg.write(section,"fx", format("%.05f",fx()) );
	cfg.write(section,"fy", format("%.05f",fy()) );
	cfg.write(section,"dist", format("[%e %e %e %e %e]", dist[0],dist[1],dist[2],dist[3],dist[4] ) );
	if (focalLengthMeters!=0) cfg.write(section,"focal_length", focalLengthMeters );
}

/**  Load all the params from a config source, in the format described in saveToConfigFile()
  */
void TCamera::loadFromConfigFile(const std::string &section,  const mrpt::utils::CConfigFileBase &cfg )
{
	vector<uint64_t>  out_res;
	cfg.read_vector(section,"resolution",vector<uint64_t>(),out_res,true);
	if (out_res.size()!=2) THROW_EXCEPTION("Expected 2-length vector in field 'resolution'");
	ncols = out_res[0];
	nrows = out_res[1];

	double fx, fy, cx, cy;
    fx = cfg.read_double(section,"fx",0, true);
    fy = cfg.read_double(section,"fy",0, true);
    cx = cfg.read_double(section,"cx",0, true);
    cy = cfg.read_double(section,"cy",0, true);

    if( fx < 2.0 ) fx *= ncols;
    if( fy < 2.0 ) fy *= nrows;
    if( cx < 2.0 ) cx *= ncols;
    if( cy < 2.0 ) cy *= nrows;

	setIntrinsicParamsFromValues( fx, fy, cx, cy );

	CVectorDouble dists;
	cfg.read_vector(section,"dist",CVectorDouble(), dists, true);
	if (dists.size()!=4 && dists.size()!=5) THROW_EXCEPTION("Expected 4 or 5-length vector in field 'dist'");

	dist.Constant(0);
	for (CVectorDouble::Index i=0;i<dists.size();i++)
		dist[i] = dists[i];

	focalLengthMeters = cfg.read_double(section,"focal_length",0, false /* optional value */ );

}

/** Rescale all the parameters for a new camera resolution (it raises an exception if the aspect ratio is modified, which is not permitted).
  */
void TCamera::scaleToResolution(unsigned int new_ncols, unsigned int new_nrows)
{
	if (ncols == new_ncols && nrows == new_nrows)
		return; // already done

	ASSERT_(new_nrows>0 && new_ncols>0)

	const double prev_aspect_ratio = ncols/double(nrows);
	const double new_aspect_ratio  = new_ncols/double(new_nrows);

	ASSERTMSG_(std::abs(prev_aspect_ratio-new_aspect_ratio)<1e-3, "TCamera: Trying to scale camera parameters for a resolution of different aspect ratio." )

	const double K = new_ncols / double(ncols);

	ncols = new_ncols;
	nrows = new_nrows;

	// fx fy cx cy
	intrinsicParams(0,0)*=K;
	intrinsicParams(1,1)*=K;
	intrinsicParams(0,2)*=K;
	intrinsicParams(1,2)*=K;

	// distortion params: unmodified.
}

bool mrpt::utils::operator ==(const mrpt::utils::TCamera& a, const mrpt::utils::TCamera& b)
{
	return
		a.ncols==b.ncols &&
		a.nrows==b.nrows &&
		a.intrinsicParams==b.intrinsicParams &&
		a.dist==b.dist &&
		a.focalLengthMeters==b.focalLengthMeters;
}
bool mrpt::utils::operator !=(const mrpt::utils::TCamera& a, const mrpt::utils::TCamera& b)
{
	return !(a==b);
}
