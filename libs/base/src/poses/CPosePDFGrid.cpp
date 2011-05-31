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


#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <mrpt/random.h>
#include <mrpt/math/utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE( CPosePDFGrid, CPosePDF, mrpt::poses )

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGrid::CPosePDFGrid(
	double		xMin,
	double		xMax,
	double		yMin,
	double		yMax,
	double		resolutionXY,
	double		resolutionPhi,
	double		phiMin,
	double		phiMax
	) : CPose2DGridTemplate<double>(xMin,xMax,yMin,yMax,resolutionXY,resolutionPhi,phiMin,phiMax)
{
	uniformDistribution();
}

/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPosePDFGrid::copyFrom(const CPosePDF &o)
{
//	int		i;

	if (this == &o) return;		// It may be used sometimes

	THROW_EXCEPTION("Not implemented yet!");

}

/*---------------------------------------------------------------
	Destructor
  ---------------------------------------------------------------*/
CPosePDFGrid::~CPosePDFGrid( )
{

}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed
		as a weighted average over all particles.
 ---------------------------------------------------------------*/
void CPosePDFGrid::getMean(CPose2D &p) const
{
	CPosePDFParticles	auxParts;

	auxParts.resetDeterministic( CPose2D(0,0,0), m_sizePhi*m_sizeY*m_sizeX );

	size_t idx=0;

	for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
		for (size_t y=0;y<m_sizeY;y++)
			for (size_t x=0;x<m_sizeX;x++)
			{
				auxParts.m_particles[idx].log_w = log( *getByIndex(x,y,phiInd) );
				*auxParts.m_particles[idx].d = CPose2D( idx2x(x),idx2y(y), idx2phi(phiInd) );
			}

	auxParts.getMean(p);
}

/*---------------------------------------------------------------
						getCovarianceAndMean
  ---------------------------------------------------------------*/
void CPosePDFGrid::getCovarianceAndMean(CMatrixDouble33 &cov, CPose2D &p) const
{
	CPosePDFParticles	auxParts;

	auxParts.resetDeterministic( CPose2D(0,0,0), m_sizePhi*m_sizeY*m_sizeX );

	size_t idx=0;

	for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
	{
		for (size_t y=0;y<m_sizeY;y++)
			for (size_t x=0;x<m_sizeX;x++)
			{
				auxParts.m_particles[idx].log_w = log( *getByIndex(x,y,phiInd) );
				*auxParts.m_particles[idx].d = CPose2D( idx2x(x),idx2y(y), idx2phi(phiInd) );
			}
	}

	auxParts.getCovarianceAndMean(cov,p);
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPosePDFGrid::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// The size:
		out << m_xMin << m_xMax
			<< m_yMin << m_yMax
			<< m_phiMin << m_phiMax
			<< m_resolutionXY << m_resolutionPhi
			<< static_cast<int32_t>(m_sizeX) << static_cast<int32_t>(m_sizeY) << static_cast<int32_t>(m_sizePhi) << static_cast<int32_t>(m_sizeXY)
			<< static_cast<int32_t>(m_idxLeftX) << static_cast<int32_t>(m_idxLeftY) << static_cast<int32_t>(m_idxLeftPhi);

		// The data:
		out << m_data;
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPosePDFGrid::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			// The size:
			in  >> m_xMin >> m_xMax
				>> m_yMin >> m_yMax
				>> m_phiMin >> m_phiMax
				>> m_resolutionXY >> m_resolutionPhi;

			int32_t	sizeX,sizeY,sizePhi,sizeXY,idxLeftX,idxLeftY,idxLeftPhi;

			in >> sizeX >> sizeY >> sizePhi >> sizeXY >> idxLeftX >> idxLeftY >> idxLeftPhi;

			m_sizeX   = sizeX;
			m_sizeY   = sizeY;
			m_sizePhi = sizePhi;
			m_sizeXY  = sizeXY;
			m_idxLeftX= idxLeftX;
			m_idxLeftY= idxLeftY;
			m_idxLeftPhi=idxLeftPhi;

			// The data:
			in >> m_data;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
void  CPosePDFGrid::saveToTextFile(
	const std::string &dataFile) const
{
	char	dimsFile[1000];
	os::sprintf(dimsFile,1000,"%s_dims.txt",dataFile.c_str());

	FILE		*f_d = os::fopen(dataFile.c_str(),"wt");
	if (!f_d) return;

	FILE		*f_s = os::fopen(dimsFile,"wt");
	if (!f_s)
	{
		os::fclose(f_d);
		return;
	}

	// Save dims:
	os::fprintf(f_s,"%u %u %u %f %f %f %f %f %f\n",
		(unsigned)m_sizeX,
		(unsigned)m_sizeY,
		(unsigned)m_sizePhi,
		m_xMin, m_xMax,
		m_yMin, m_yMax,
		m_phiMin, m_phiMax
		);


	// Save one rectangular matrix each time:
	for (unsigned int phiInd = 0; phiInd < m_sizePhi; phiInd++)
	{
		for (unsigned int y=0;y<m_sizeY;y++)
		{
			for (unsigned int x=0;x<m_sizeX;x++)
				os::fprintf(f_d,"%.5e ", *getByIndex(x,y,phiInd) );
			os::fprintf(f_d,"\n");
		}
	}

	os::fclose(f_s);
	os::fclose(f_d);
	return; // Done!
}

/*---------------------------------------------------------------
						changeCoordinatesReference
  ---------------------------------------------------------------*/
void  CPosePDFGrid::changeCoordinatesReference(const CPose3D &newReferenceBase )
{
	MRPT_UNUSED_PARAM(newReferenceBase);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPosePDFGrid::bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop )
{
	MRPT_UNUSED_PARAM(p1);MRPT_UNUSED_PARAM(p2);MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void  CPosePDFGrid::inverse(CPosePDF &o) const
{
	MRPT_UNUSED_PARAM(o);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFGrid::drawSingleSample( CPose2D &outPart ) const
{
	MRPT_UNUSED_PARAM(outPart);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFGrid::drawManySamples(
	size_t						N,
	std::vector<vector_double>	&outSamples ) const
{
	MRPT_UNUSED_PARAM(N); MRPT_UNUSED_PARAM(outSamples);

	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					CPosePDFGrid
 ---------------------------------------------------------------*/
void  CPosePDFGrid::normalize()
{
	double						SUM = 0;

	// SUM:
	for (vector<double>::const_iterator it=m_data.begin();it!=m_data.end();it++)	SUM += *it;

	if (SUM>0)
	{
		// Normalize:
		for (vector<double>::iterator it=m_data.begin();it!=m_data.end();it++)	*it /= SUM;
	}
}

/*---------------------------------------------------------------
						uniformDistribution
  ---------------------------------------------------------------*/
void  CPosePDFGrid::uniformDistribution()
{
	double						val =  1.0f / m_data.size();

	for (vector<double>::iterator it=m_data.begin();it!=m_data.end();it++)
		*it = val;
}
