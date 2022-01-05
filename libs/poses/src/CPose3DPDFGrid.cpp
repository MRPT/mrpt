/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"	// Precompiled headers
//
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGrid.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <fstream>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPose3DPDFGrid, CPose3DPDF, mrpt::poses)

CPose3DPDFGrid::CPose3DPDFGrid(
	const mrpt::math::TPose3D& bb_min, const mrpt::math::TPose3D& bb_max,
	double resolution_XYZ, double resolution_YPR)
	: CPose3DGridTemplate<double>(
		  bb_min, bb_max, resolution_XYZ, resolution_YPR)
{
	uniformDistribution();
}

void CPose3DPDFGrid::copyFrom(const CPose3DPDF& o)
{
	if (this == &o) return;	 // It may be used sometimes

	THROW_EXCEPTION("Not implemented yet!");
}

void CPose3DPDFGrid::getMean(CPose3D& p) const
{
	// Calc average on SE(3)
	mrpt::poses::SE_average<3> se_averager;

	for (size_t cR = 0; cR < m_sizeRoll; cR++)
		for (size_t cP = 0; cP < m_sizePitch; cP++)
			for (size_t cY = 0; cY < m_sizeYaw; cY++)
				for (size_t cz = 0; cz < m_sizeZ; cz++)
					for (size_t cy = 0; cy < m_sizeY; cy++)
						for (size_t cx = 0; cx < m_sizeX; cx++)
						{
							const double w =
								*getByIndex(cx, cy, cz, cY, cP, cR);
							se_averager.append(
								CPose3D(
									idx2x(cx), idx2y(cy), idx2z(cz),
									idx2yaw(cY), idx2pitch(cP), idx2roll(cR)),
								w);
						}
	se_averager.get_average(p);
}

std::tuple<CMatrixDouble66, CPose3D> CPose3DPDFGrid::getCovarianceAndMean()
	const
{
	CPose3DPDFParticles auxParts;
	auxParts.resetDeterministic(TPose3D(), m_size_xyzYPR);

	size_t idx = 0;
	for (size_t cR = 0; cR < m_sizeRoll; cR++)
		for (size_t cP = 0; cP < m_sizePitch; cP++)
			for (size_t cY = 0; cY < m_sizeYaw; cY++)
				for (size_t cz = 0; cz < m_sizeZ; cz++)
					for (size_t cy = 0; cy < m_sizeY; cy++)
						for (size_t cx = 0; cx < m_sizeX; cx++)
						{
							const double w =
								*getByIndex(cx, cy, cz, cY, cP, cR);
							auxParts.m_particles[idx].log_w = std::log(w);
							auxParts.m_particles[idx].d = mrpt::math::TPose3D(
								idx2x(cx), idx2y(cy), idx2z(cz), idx2yaw(cY),
								idx2pitch(cP), idx2roll(cR));

							++idx;
						}
	return auxParts.getCovarianceAndMean();
}

uint8_t CPose3DPDFGrid::serializeGetVersion() const { return 0; }
void CPose3DPDFGrid::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The size:
	out << m_bb_min << m_bb_max << m_resolutionXYZ << m_resolutionYPR;
	out.WriteAs<int32_t>(m_sizeX);
	out.WriteAs<int32_t>(m_sizeY);
	out.WriteAs<int32_t>(m_sizeZ);
	out.WriteAs<int32_t>(m_sizeYaw);
	out.WriteAs<int32_t>(m_sizePitch);
	out.WriteAs<int32_t>(m_sizeRoll);
	out << m_sizeX << m_sizeY << m_sizeZ << m_sizeYaw << m_sizePitch
		<< m_sizeRoll;
	out << m_min_cidX << m_min_cidY << m_min_cidZ << m_min_cidYaw
		<< m_min_cidPitch << m_min_cidRoll;

	// The data:
	out << m_data;
}
void CPose3DPDFGrid::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> m_bb_min >> m_bb_max >> m_resolutionXYZ >> m_resolutionYPR;
			m_sizeX = in.ReadAs<int32_t>();
			m_sizeY = in.ReadAs<int32_t>();
			m_sizeZ = in.ReadAs<int32_t>();
			m_sizeYaw = in.ReadAs<int32_t>();
			m_sizePitch = in.ReadAs<int32_t>();
			m_sizeRoll = in.ReadAs<int32_t>();
			in >> m_sizeX >> m_sizeY >> m_sizeZ >> m_sizeYaw >> m_sizePitch >>
				m_sizeRoll;
			in >> m_min_cidX >> m_min_cidY >> m_min_cidZ >> m_min_cidYaw >>
				m_min_cidPitch >> m_min_cidRoll;

			// The data:
			in >> m_data;

			update_cached_size_products();

			ASSERT_EQUAL_(m_data.size(), m_size_xyzYPR);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

bool CPose3DPDFGrid::saveToTextFile(const std::string& dataFile) const
{
	THROW_EXCEPTION("Not implemented yet");

	//	return true;  // Done!
}

void CPose3DPDFGrid::changeCoordinatesReference([
	[maybe_unused]] const CPose3D& newReferenceBase)
{
	THROW_EXCEPTION("Not implemented yet!");
}

void CPose3DPDFGrid::bayesianFusion(
	[[maybe_unused]] const CPose3DPDF& p1,
	[[maybe_unused]] const CPose3DPDF& p2)
{
	THROW_EXCEPTION("Not implemented yet!");
}

void CPose3DPDFGrid::inverse([[maybe_unused]] CPose3DPDF& o) const
{
	THROW_EXCEPTION("Not implemented yet!");
}

void CPose3DPDFGrid::drawSingleSample([[maybe_unused]] CPose3D& outPart) const
{
	THROW_EXCEPTION("Not implemented yet!");
}

void CPose3DPDFGrid::drawManySamples(
	[[maybe_unused]] size_t N,
	[[maybe_unused]] std::vector<CVectorDouble>& outSamples) const
{
	THROW_EXCEPTION("Not implemented yet!");
}

void CPose3DPDFGrid::normalize()
{
	double SUM = 0;

	// SUM:
	for (auto it = m_data.begin(); it != m_data.end(); ++it)
		SUM += *it;

	if (SUM > 0)
	{
		const auto f = 1.0 / SUM;
		for (double& it : m_data)
			it *= f;
	}
}

void CPose3DPDFGrid::uniformDistribution()
{
	const double val = 1.0 / m_data.size();

	for (double& it : m_data)
		it = val;
}
