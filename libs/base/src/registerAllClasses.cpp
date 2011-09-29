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


