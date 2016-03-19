/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/base.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef mrpt_base_H
#	include "base-precomp.h"
#endif

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;

/*
Note: Why do we need "CStartUpClassesRegister" and "registerAllClasses_mrpt_base()" at all?
 One idea is to use static const members in every CObject, initialized via a call to registerClass(). 
 In this way, there is NO NEED to build the ugly list of classes to be registered below, and it works...
 but when building as STATIC LIBRARIES, the compiler optimizes out all non-directly used classes, 
 thus not registering them!!
 So, it seems we must live with this ugly method for the sake of static libs...
(Jose Luis Blanco, April 2013)
*/

void registerAllClasses_mrpt_base();

CStartUpClassesRegister  mrpt_base_class_reg(&registerAllClasses_mrpt_base);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_base
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_base()
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	// Abstract classes are not registered since they can not be
	//   instanciated, nor loaded from streams.
	registerClass( CLASS_ID( CMatrix ) );
	registerClass( CLASS_ID( CMatrixD ) );
	registerClass( CLASS_ID( CMatrixB ) );
	registerClass( CLASS_ID( CPolygon ) );

//   Hack to enable compatibility with an older name of this class:
	registerClass( CLASS_ID( CImage ) );
	registerClassCustomName( "CMRPTImage", CLASS_ID( CImage ) );

	registerClass( CLASS_ID( CSimpleDatabase ) );
	registerClass( CLASS_ID( CSimpleDatabaseTable ) );
	registerClass( CLASS_ID( CPropertiesValuesList ) );
	registerClass( CLASS_ID( CMHPropertiesValuesList ) );
	registerClass( CLASS_ID( CTypeSelector ) );
	registerClass( CLASS_ID( CMemoryChunk ) );

	registerClass( CLASS_ID( CPoint2D ) );
	registerClass( CLASS_ID( CPoint3D ) );
	registerClass( CLASS_ID( CPose2D ) );
	registerClass( CLASS_ID( CPose3D ) );
	registerClass( CLASS_ID( CPose3DQuat ) );
	registerClass( CLASS_ID( CPoses2DSequence ) );
	registerClass( CLASS_ID( CPoses3DSequence ) );


	registerClass( CLASS_ID( CPosePDF ) );
	registerClass( CLASS_ID( CPosePDFGaussian ) );
	registerClass( CLASS_ID( CPosePDFGaussianInf ) );
	registerClass( CLASS_ID( CPosePDFParticles ) );
	registerClass( CLASS_ID( CPosePDFGrid ) );
	registerClass( CLASS_ID( CPosePDFSOG ) );

	registerClass( CLASS_ID( CPointPDF ) );
	registerClass( CLASS_ID( CPointPDFGaussian ) );
	registerClass( CLASS_ID( CPointPDFParticles ) );
	registerClass( CLASS_ID( CPointPDFSOG ) );

	registerClass( CLASS_ID( CPosePDF ) );
	registerClass( CLASS_ID( CPose3DPDF ) );
	registerClass( CLASS_ID( CPose3DQuatPDF ) );
	registerClass( CLASS_ID( CPose3DPDFGaussian ) );
	registerClass( CLASS_ID( CPose3DPDFGaussianInf ) );
	registerClass( CLASS_ID( CPose3DPDFParticles ) );
	registerClass( CLASS_ID( CPose3DPDFSOG ) );

	registerClass( CLASS_ID( CPose3DQuatPDF ) );
	registerClass( CLASS_ID( CPose3DQuatPDFGaussian ) );
	registerClass( CLASS_ID( CPose3DQuatPDFGaussianInf ) );

	registerClass( CLASS_ID( CPose3DInterpolator ) );

	registerClass( CLASS_ID( TCamera ) );
	registerClass( CLASS_ID( TStereoCamera ) );
	registerClass( CLASS_ID( CSplineInterpolator1D  ) );
	registerClass( CLASS_ID( CStringList ) );
#endif
}

