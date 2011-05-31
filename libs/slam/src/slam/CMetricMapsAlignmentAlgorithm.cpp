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

#include <mrpt/slam.h>  // Precompiled header



#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;


/*---------------------------------------------------------------
					Align
  ---------------------------------------------------------------*/
CPosePDFPtr CMetricMapsAlignmentAlgorithm::Align(
    const CMetricMap		*m1,
    const CMetricMap		*m2,
    const CPose2D			&grossEst,
    float					*runningTime,
    void					*info )
{
	CPosePDFGaussian    posePDF(grossEst, CMatrixDouble33() );
	return AlignPDF(m1,m2,posePDF,runningTime,info);
}

/*---------------------------------------------------------------
					Align3D
  ---------------------------------------------------------------*/
CPose3DPDFPtr CMetricMapsAlignmentAlgorithm::Align3D(
    const CMetricMap		*m1,
    const CMetricMap		*m2,
    const CPose3D			&grossEst,
    float					*runningTime,
    void					*info )
{
	CPose3DPDFGaussian    posePDF;
	posePDF.mean = grossEst;
	return Align3DPDF(m1,m2,posePDF,runningTime,info);
}


