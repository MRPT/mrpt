/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/random.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/SO_SE_average.h>
#include <fstream>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPosePDFGrid, CPosePDF, mrpt::poses)

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGrid::CPosePDFGrid(
	double xMin, double xMax, double yMin, double yMax, double resolutionXY,
	double resolutionPhi, double phiMin, double phiMax)
	: CPose2DGridTemplate<double>(
		  xMin, xMax, yMin, yMax, resolutionXY, resolutionPhi, phiMin, phiMax)
{
	uniformDistribution();
}

void CPosePDFGrid::copyFrom(const CPosePDF& o)
{
	if (this == &o) return;  // It may be used sometimes

	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
	Destructor
  ---------------------------------------------------------------*/
CPosePDFGrid::~CPosePDFGrid() = default;
/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF), computed
		as a weighted average over all particles.
 ---------------------------------------------------------------*/
void CPosePDFGrid::getMean(CPose2D& p) const
{
	// Calc average on SE(2)
	mrpt::poses::SE_average<2> se_averager;
	for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
		for (size_t y = 0; y < m_sizeY; y++)
			for (size_t x = 0; x < m_sizeX; x++)
			{
				const double w = *getByIndex(x, y, phiInd);
				se_averager.append(
					CPose2D(idx2x(x), idx2y(y), idx2phi(phiInd)), w);
			}
	se_averager.get_average(p);
}

/*---------------------------------------------------------------
						getCovarianceAndMean
  ---------------------------------------------------------------*/
void CPosePDFGrid::getCovarianceAndMean(CMatrixDouble33& cov, CPose2D& p) const
{
	CPosePDFParticles auxParts;
	auxParts.resetDeterministic(
		TPose2D(0, 0, 0), m_sizePhi * m_sizeY * m_sizeX);
	size_t idx = 0;
	for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
	{
		for (size_t y = 0; y < m_sizeY; y++)
			for (size_t x = 0; x < m_sizeX; x++)
			{
				auxParts.m_particles[idx].log_w =
					log(*getByIndex(x, y, phiInd));
				auxParts.m_particles[idx].d =
					TPose2D(idx2x(x), idx2y(y), idx2phi(phiInd));
			}
	}
	auxParts.getCovarianceAndMean(cov, p);
}

uint8_t CPosePDFGrid::serializeGetVersion() const { return 0; }
void CPosePDFGrid::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The size:
	out << m_xMin << m_xMax << m_yMin << m_yMax << m_phiMin << m_phiMax
		<< m_resolutionXY << m_resolutionPhi << static_cast<int32_t>(m_sizeX)
		<< static_cast<int32_t>(m_sizeY) << static_cast<int32_t>(m_sizePhi)
		<< static_cast<int32_t>(m_sizeXY) << static_cast<int32_t>(m_idxLeftX)
		<< static_cast<int32_t>(m_idxLeftY)
		<< static_cast<int32_t>(m_idxLeftPhi);

	// The data:
	out << m_data;
}
void CPosePDFGrid::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			// The size:
			in >> m_xMin >> m_xMax >> m_yMin >> m_yMax >> m_phiMin >>
				m_phiMax >> m_resolutionXY >> m_resolutionPhi;

			int32_t sizeX, sizeY, sizePhi, sizeXY, idxLeftX, idxLeftY,
				idxLeftPhi;

			in >> sizeX >> sizeY >> sizePhi >> sizeXY >> idxLeftX >> idxLeftY >>
				idxLeftPhi;

			m_sizeX = sizeX;
			m_sizeY = sizeY;
			m_sizePhi = sizePhi;
			m_sizeXY = sizeXY;
			m_idxLeftX = idxLeftX;
			m_idxLeftY = idxLeftY;
			m_idxLeftPhi = idxLeftPhi;

			// The data:
			in >> m_data;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

bool CPosePDFGrid::saveToTextFile(const std::string& dataFile) const
{
	const auto dimsFile = dataFile + std::string("_dims.txt");

	std::ofstream f_d(dataFile), f_s(dimsFile);
	if (!f_d.is_open() || !f_s.is_open()) return false;

	// Save dims:
	f_s << mrpt::format(
		"%u %u %u %f %f %f %f %f %f\n", (unsigned)m_sizeX, (unsigned)m_sizeY,
		(unsigned)m_sizePhi, m_xMin, m_xMax, m_yMin, m_yMax, m_phiMin,
		m_phiMax);

	// Save one rectangular matrix each time:
	for (unsigned int phiInd = 0; phiInd < m_sizePhi; phiInd++)
	{
		for (unsigned int y = 0; y < m_sizeY; y++)
		{
			for (unsigned int x = 0; x < m_sizeX; x++)
				f_d << mrpt::format("%.5e ", *getByIndex(x, y, phiInd));
			f_d << std::endl;
		}
	}

	return true;  // Done!
}

/*---------------------------------------------------------------
						changeCoordinatesReference
  ---------------------------------------------------------------*/
void CPosePDFGrid::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
	MRPT_UNUSED_PARAM(newReferenceBase);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void CPosePDFGrid::bayesianFusion(
	const CPosePDF& p1, const CPosePDF& p2,
	const double minMahalanobisDistToDrop)
{
	MRPT_UNUSED_PARAM(p1);
	MRPT_UNUSED_PARAM(p2);
	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPosePDFGrid::inverse(CPosePDF& o) const
{
	MRPT_UNUSED_PARAM(o);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPosePDFGrid::drawSingleSample(CPose2D& outPart) const
{
	MRPT_UNUSED_PARAM(outPart);
	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPosePDFGrid::drawManySamples(
	size_t N, std::vector<CVectorDouble>& outSamples) const
{
	MRPT_UNUSED_PARAM(N);
	MRPT_UNUSED_PARAM(outSamples);

	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					CPosePDFGrid
 ---------------------------------------------------------------*/
void CPosePDFGrid::normalize()
{
	double SUM = 0;

	// SUM:
	for (auto it = m_data.begin(); it != m_data.end(); ++it) SUM += *it;

	if (SUM > 0)
	{
		// Normalize:
		for (double& it : m_data) it /= SUM;
	}
}

/*---------------------------------------------------------------
						uniformDistribution
  ---------------------------------------------------------------*/
void CPosePDFGrid::uniformDistribution()
{
	double val = 1.0f / m_data.size();

	for (double& it : m_data) it = val;
}
