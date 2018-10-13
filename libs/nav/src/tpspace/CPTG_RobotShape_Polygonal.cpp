/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::nav;

CPTG_RobotShape_Polygonal::CPTG_RobotShape_Polygonal() : m_robotShape() {}
CPTG_RobotShape_Polygonal::~CPTG_RobotShape_Polygonal() = default;
void CPTG_RobotShape_Polygonal::setRobotShape(
	const mrpt::math::CPolygon& robotShape)
{
	ASSERT_ABOVEEQ_(robotShape.size(), 3u);
	m_robotShape = robotShape;

	m_robotMaxRadius = .0;  // Default minimum
	for (const auto& v : m_robotShape)
		mrpt::keep_max(m_robotMaxRadius, v.norm());

	internal_processNewRobotShape();
}

void CPTG_RobotShape_Polygonal::loadDefaultParams()
{
	m_robotShape.clear();
	m_robotShape.AddVertex(-0.15, 0.15);
	m_robotShape.AddVertex(0.2, 0.1);
	m_robotShape.AddVertex(0.2, -0.1);
	m_robotShape.AddVertex(-0.15, -0.15);
}

void CPTG_RobotShape_Polygonal::loadShapeFromConfigFile(
	const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
	bool any_pt = false;
	const double BADNUM = std::numeric_limits<double>::max();

	for (unsigned int nPt = 0;; ++nPt)
	{
		const std::string sPtx = mrpt::format("shape_x%u", nPt);
		const std::string sPty = mrpt::format("shape_y%u", nPt);

		const double ptx = cfg.read_double(sSection, sPtx, BADNUM, false);
		const double pty = cfg.read_double(sSection, sPty, BADNUM, false);
		if (ptx == BADNUM && pty == BADNUM) break;
		ASSERTMSG_(
			(ptx != BADNUM && pty != BADNUM),
			"Error: mismatch between number of pts in {x,y} defining robot "
			"shape");

		if (!any_pt)
		{
			m_robotShape.clear();
			any_pt = true;
		}

		m_robotShape.AddVertex(ptx, pty);
	}

	if (any_pt) internal_processNewRobotShape();
}

void CPTG_RobotShape_Polygonal::saveToConfigFile(
	mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
	const int WN = 25, WV = 30;

	for (unsigned int i = 0; i < m_robotShape.size(); i++)
	{
		const std::string sPtx = mrpt::format("shape_x%u", i);
		const std::string sPty = mrpt::format("shape_y%u", i);

		cfg.write(
			sSection, sPtx, m_robotShape[i].x, WN, WV,
			"Robot polygonal shape, `x` [m].");
		cfg.write(
			sSection, sPty, m_robotShape[i].y, WN, WV,
			"Robot polygonal shape, `y` [m].");
	}
}

void CPTG_RobotShape_Polygonal::add_robotShape_to_setOfLines(
	mrpt::opengl::CSetOfLines& gl_shape,
	const mrpt::poses::CPose2D& origin) const
{
	const int N = m_robotShape.size();
	if (N >= 2)
	{
		// Transform coordinates:
		mrpt::math::CVectorDouble shap_x(N), shap_y(N), shap_z(N);
		for (int i = 0; i < N; i++)
		{
			origin.composePoint(
				m_robotShape[i].x, m_robotShape[i].y, 0, shap_x[i], shap_y[i],
				shap_z[i]);
		}

		gl_shape.appendLine(
			shap_x[0], shap_y[0], shap_z[0], shap_x[1], shap_y[1], shap_z[1]);
		for (int i = 0; i <= shap_x.size(); i++)
		{
			const int idx = i % shap_x.size();
			gl_shape.appendLineStrip(shap_x[idx], shap_y[idx], shap_z[idx]);
		}
	}
}

void CPTG_RobotShape_Polygonal::internal_shape_loadFromStream(
	mrpt::serialization::CArchive& in)
{
	uint8_t version;
	in >> version;

	switch (version)
	{
		case 0:
			in >> m_robotShape;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CPTG_RobotShape_Polygonal::internal_shape_saveToStream(
	mrpt::serialization::CArchive& out) const
{
	uint8_t version = 0;
	out << version;

	out << m_robotShape;
}

double CPTG_RobotShape_Polygonal::getMaxRobotRadius() const
{
	return m_robotMaxRadius;
}

bool CPTG_RobotShape_Polygonal::isPointInsideRobotShape(
	const double x, const double y) const
{
	return m_robotShape.contains(mrpt::math::TPoint2D(x, y));
}

double CPTG_RobotShape_Polygonal::evalClearanceToRobotShape(
	const double ox, const double oy) const
{
	// Approximated computation, valid for relatively distant objects, which
	// is where clearance is useful.

	if (isPointInsideRobotShape(ox, oy)) return .0;

	double d = mrpt::hypot_fast(ox, oy) - m_robotMaxRadius;

	// if d<=0, we know from the isPointInsideRobotShape() above that
	// it's a false positive: enforce a minimum "fake" clearance:
	mrpt::keep_max(d, 0.1 * m_robotMaxRadius);

	return d;
}
