/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h>

using namespace mrpt::nav;

CPTG_RobotShape_Circular::CPTG_RobotShape_Circular() : 
	m_robotRadius(.0) 
{
}

CPTG_RobotShape_Circular::~CPTG_RobotShape_Circular() 
{
}

void CPTG_RobotShape_Circular::setRobotShapeRadius(const double robot_radius) 
{
	m_robotRadius = robot_radius;
	internal_processNewRobotShape();
}

void CPTG_RobotShape_Circular::loadDefaultParams()
{
	m_robotRadius = 0.2;
}

void CPTG_RobotShape_Circular::loadShapeFromConfigFile(const mrpt::utils::CConfigFileBase & cfg, const std::string & sSection)
{
	const double old_R = m_robotRadius;
	MRPT_LOAD_HERE_CONFIG_VAR(robot_radius, double, m_robotRadius, cfg, sSection);

	if (m_robotRadius != old_R)
		internal_processNewRobotShape();
}
void CPTG_RobotShape_Circular::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg, const std::string &sSection) const
{
	const int WN = 25, WV = 30;

	cfg.write(sSection, "robot_radius", m_robotRadius, WN, WV, "Robot radius [m].");
}

void CPTG_RobotShape_Circular::add_robotShape_to_setOfLines(
	mrpt::opengl::CSetOfLines &gl_shape,
	const mrpt::poses::CPose2D &origin) const
{
	const double R = m_robotRadius;
	const int N = 17;
	// Transform coordinates:
	mrpt::math::CVectorDouble shap_x(N), shap_y(N), shap_z(N);
	for (int i = 0; i<N; i++) {
		origin.composePoint(
			R*cos(i * 2 * M_PI / (N - 1)), R*sin(i * 2 * M_PI / (N - 1)), 0,
			shap_x[i], shap_y[i], shap_z[i]);
	}
	// Draw a "radius" to identify the "forward" orientation (phi=0)
	gl_shape.appendLine(origin.x(), origin.y(), .0, shap_x[0], shap_y[0], shap_z[0]);
	for (int i = 1; i <= shap_x.size(); i++) {
		const int idx = i % shap_x.size();
		gl_shape.appendLineStrip(shap_x[idx], shap_y[idx], shap_z[idx]);
	}
	// Draw a "cross" to identify the robot center
	gl_shape.appendLine(origin.x() - R*0.02, origin.y(), .0, origin.x() + R*0.02, origin.y(), .0);
	gl_shape.appendLine(origin.x(), origin.y() - R*0.02, .0, origin.x(), origin.y() + R*0.02, .0);
}


void CPTG_RobotShape_Circular::internal_shape_loadFromStream(mrpt::utils::CStream &in)
{
	uint8_t version;
	in >> version;

	switch (version)
	{
	case 0:
		in >> m_robotRadius;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CPTG_RobotShape_Circular::internal_shape_saveToStream(mrpt::utils::CStream &out) const
{
	uint8_t version = 0;
	out << version;

	out << m_robotRadius;
}

double CPTG_RobotShape_Circular::getApproxRobotRadius() const
{
	return m_robotRadius;
}

bool CPTG_RobotShape_Circular::isPointInsideRobotShape(const double x, const double y) const
{
	return ::hypot(x, y) < m_robotRadius;
}

double CPTG_RobotShape_Circular::evalClearanceToRobotShape(const double ox, const double oy) const
{
	return ::hypot(ox, oy) - m_robotRadius;
}



