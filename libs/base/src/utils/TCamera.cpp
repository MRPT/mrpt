/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/TCamera.h>
#include <mrpt/math/ops_matrices.h>  // For "<<" ">>" operators.

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


IMPLEMENTS_SERIALIZABLE( TStereoCamera, CSerializable, mrpt::utils )

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
void TStereoCamera::saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const
{
	cfg.write(section,"model", model );
	cfg.write(section,"resolution",format("[%u %u]",(unsigned int)leftCamera.ncols,(unsigned int)leftCamera.nrows));
	cfg.write(section,"left_cx", leftCamera.cx() );
	cfg.write(section,"left_cy", leftCamera.cy() );
	cfg.write(section,"left_fx", leftCamera.fx() );
	cfg.write(section,"left_fy", leftCamera.fy() );
	cfg.write(section,"left_dist", format("[%e %e %e %e %e]", leftCamera.k1(),leftCamera.k2(),leftCamera.p1(),leftCamera.p2(),leftCamera.k3() ) );
	cfg.write(section,"left_focal_length", leftCamera.focalLengthMeters );
	cfg.write(section,"right_cx", rightCamera.cx() );
	cfg.write(section,"right_cy", rightCamera.cy() );
	cfg.write(section,"right_fx", rightCamera.fx() );
	cfg.write(section,"right_fy", rightCamera.fy() );
	cfg.write(section,"right_dist", format("[%e %e %e %e %e]", rightCamera.k1(),rightCamera.k2(),rightCamera.p1(),rightCamera.p2(),rightCamera.k3() ) );
	cfg.write(section,"right_focal_length", rightCamera.focalLengthMeters );
}

/**  Load all the params from a config source, in the format described in saveToConfigFile()
  */
void TStereoCamera::loadFromConfigFile(const std::string &section,  const mrpt::utils::CConfigFileBase &cfg )
{
    TStereoCameraModel model = (TStereoCameraModel)cfg.read_int( section, "model", 0, true );
	vector<uint64_t>  out_res;
	cfg.read_vector( section, "resolution", vector<uint64_t>(), out_res, true );
	if (out_res.size()!=2) THROW_EXCEPTION("Expected 2-length vector in field 'resolution'");
	leftCamera.ncols = rightCamera.ncols = out_res[0];
	leftCamera.nrows = rightCamera.nrows = out_res[1];

    if( model == Bumblebee )
    {
        // Bumblebee Intrinsic & distortion parameters
        leftCamera.setIntrinsicParamsFromValues(
                    0.81945957*leftCamera.ncols, 1.09261276*leftCamera.nrows,
                    0.499950781*leftCamera.ncols, 0.506134245*leftCamera.nrows );
        leftCamera.setDistortionParamsFromValues( -3.627383e-001, 2.099672e-001, 0, 0, -8.575903e-002 );

        rightCamera.setIntrinsicParamsFromValues(
                    0.822166309*leftCamera.ncols, 1.096221745*leftCamera.nrows,
                    0.507065918*leftCamera.ncols, 0.524686589*leftCamera.nrows );
        rightCamera.setDistortionParamsFromValues( -3.782850e-001, 2.539438e-001, 0, 0, -1.279638e-001 );

        leftCamera.focalLengthMeters = rightCamera.focalLengthMeters = 0.0038;      // 3.8 mm

        // Camera pose
        CMatrixDouble44 A;
        A.set_unsafe(0,0,9.999777e-001);    A.set_unsafe(0,1,-6.262494e-003);   A.set_unsafe(0,2,2.340592e-003);    A.set_unsafe(0,3,1.227338e-001);
        A.set_unsafe(1,0,6.261120e-003);    A.set_unsafe(1,1,9.999802e-001);    A.set_unsafe(1,2,5.939072e-004);    A.set_unsafe(1,3,-3.671682e-004);
        A.set_unsafe(2,0,-2.344265e-003);   A.set_unsafe(2,1,-5.792392e-004);   A.set_unsafe(2,2,9.999971e-001);    A.set_unsafe(2,3,-1.499571e-004);
        A.set_unsafe(3,0,0);                A.set_unsafe(3,1,0);                A.set_unsafe(3,2,0);                A.set_unsafe(3,3,0);
        rightCameraPose = CPose3DQuat( A );
    }
    else if( model == Custom )
    {
        // Intrinsic & distortion parameters
        double fx, fy, cx, cy;
        fx = cfg.read_double(section,"left_fx",0, true);
        fy = cfg.read_double(section,"left_fy",0, true);
        cx = cfg.read_double(section,"left_cx",0, true);
        cy = cfg.read_double(section,"left_cy",0, true);

        if( fx < 2.0 ) fx *= leftCamera.ncols;
        if( fy < 2.0 ) fy *= leftCamera.nrows;
        if( cx < 2.0 ) cx *= leftCamera.ncols;
        if( cy < 2.0 ) cy *= leftCamera.nrows;

        leftCamera.setIntrinsicParamsFromValues( fx, fy, cx, cy );

        fx = cfg.read_double(section,"right_fx",0, true);
        fy = cfg.read_double(section,"right_fy",0, true);
        cx = cfg.read_double(section,"right_cx",0, true);
        cy = cfg.read_double(section,"right_cy",0, true);

        if( fx < 2.0 ) fx *= rightCamera.ncols;
        if( fy < 2.0 ) fy *= rightCamera.nrows;
        if( cx < 2.0 ) cx *= rightCamera.ncols;
        if( cy < 2.0 ) cy *= rightCamera.nrows;

        rightCamera.setIntrinsicParamsFromValues( fx, fy, cx, cy );

        vector_double dists;
        cfg.read_vector(section,"left_dist",vector_double(), dists, true);
        if (dists.size()!=4 && dists.size()!=5) THROW_EXCEPTION("Expected 4 or 5-length vector in field 'dist'");

        leftCamera.setDistortionParamsVector( dists );

        cfg.read_vector(section,"right_dist",vector_double(), dists, true);
        if (dists.size()!=4 && dists.size()!=5) THROW_EXCEPTION("Expected 4 or 5-length vector in field 'dist'");

        rightCamera.setDistortionParamsVector( dists );

//        dist.Constant(0);
//        for (vector_double::Index i=0;i<dists.size();i++)
//            leftCamera.dist[i] = dists[i];

        leftCamera.focalLengthMeters = cfg.read_double(section,"left_focal_length",0.002, false /* optional value */ );
        rightCamera.focalLengthMeters = cfg.read_double(section,"right_focal_length",0.002, false /* optional value */ );

        CMatrixDouble44 A;
        vector_double rcRot, rcTrans;
        cfg.read_vector(section,"rcRot",vector_double(), rcRot, true);
        cfg.read_vector(section,"rcTrans",vector_double(), rcTrans, true);

        A.set_unsafe(0,0,rcRot[0]);     A.set_unsafe(0,1,rcRot[1]);     A.set_unsafe(0,2,rcRot[2]);     A.set_unsafe(0,3,rcTrans[0]);
        A.set_unsafe(1,0,rcRot[3]);     A.set_unsafe(1,1,rcRot[4]);     A.set_unsafe(1,2,rcRot[5]);     A.set_unsafe(1,3,rcTrans[1]);
        A.set_unsafe(2,0,rcRot[6]);     A.set_unsafe(2,1,rcRot[7]);     A.set_unsafe(2,2,rcRot[8]);     A.set_unsafe(2,3,rcTrans[2]);
        A.set_unsafe(3,0,0);            A.set_unsafe(3,1,0);            A.set_unsafe(3,2,0);            A.set_unsafe(3,3,0);
        rightCameraPose = CPose3DQuat( A );
    }
} // loadFromConfigFile

// WriteToStream
void TStereoCamera::writeToStream( CStream &out, int *version ) const
{
	if( version )
		*version = 0;
	else
	{
	    out << (uint8_t)model
            << leftCamera
            << rightCamera
            << rightCameraPose;
	} // end else
}

// ReadFromStream
void TStereoCamera::readFromStream( CStream &in, int version )
{
	switch( version )
	{
	case 0:
    {
        uint8_t _model;
        in  >> _model;
        model = (TStereoCameraModel)_model;

        in  >> leftCamera
            >> rightCamera
            >> rightCameraPose;
    } break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION( version )
	}
}


/* Implements serialization for the TCamera struct as it will be included within CObservations objects */
IMPLEMENTS_SERIALIZABLE( TCamera, CSerializable, mrpt::utils )

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
	cfg.write(section,"cx", cx() );
	cfg.write(section,"cy", cy() );
	cfg.write(section,"fx", fx() );
	cfg.write(section,"fy", fy() );
	cfg.write(section,"dist", format("[%e %e %e %e %e]", dist[0],dist[1],dist[2],dist[3],dist[4] ) );
	cfg.write(section,"focal_length", focalLengthMeters );
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

	vector_double dists;
	cfg.read_vector(section,"dist",vector_double(), dists, true);
	if (dists.size()!=4 && dists.size()!=5) THROW_EXCEPTION("Expected 4 or 5-length vector in field 'dist'");

	dist.Constant(0);
	for (vector_double::Index i=0;i<dists.size();i++)
		dist[i] = dists[i];

	focalLengthMeters = cfg.read_double(section,"focal_length",0.002, false /* optional value */ );

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
