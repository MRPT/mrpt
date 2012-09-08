/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#	define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#	undef mrpt_base_H
#	include <mrpt/base.h>
#endif


using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::poses;


void registerAllClasses_mrpt_base();

CStartUpClassesRegister  mrpt_base_class_reg(&registerAllClasses_mrpt_base);


/*---------------------------------------------------------------
					registerAllClasses_mrpt_base
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_base()
{
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
	registerClass( CLASS_ID( CSplineInterpolator1D  ) );
	registerClass( CLASS_ID( CStringList ) );
}


