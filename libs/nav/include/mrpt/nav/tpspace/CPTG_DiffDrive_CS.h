/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

namespace mrpt
{
namespace nav
{
/** A PTG for optimal paths of type "CS", as named in PTG papers.
 * - **Compatible kinematics**: differential-driven / Ackermann steering
 * - **Compatible robot shape**: Arbitrary 2D polygon
 * - **PTG parameters**: Use the app `ptg-configurator`
 *
 *  See "Obstacle Distance for Car-Like Robots", IEEE Trans. Rob. And Autom,
 * 1999.
 * \note [Before MRPT 1.5.0 this was named CPTG5]
 *  \ingroup nav_tpspace
 */
class NAV_IMPEXP CPTG_DiffDrive_CS : public CPTG_DiffDrive_CollisionGridBased
{
	DEFINE_SERIALIZABLE(CPTG_DiffDrive_CS)
   public:
	CPTG_DiffDrive_CS() : R(0), K(0) {}
	CPTG_DiffDrive_CS(
		const mrpt::utils::CConfigFileBase& cfg, const std::string& sSection)
	{
		loadFromConfigFile(cfg, sSection);
	}
	virtual void loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& cfg,
		const std::string& sSection) override;
	virtual void saveToConfigFile(
		mrpt::utils::CConfigFileBase& cfg,
		const std::string& sSection) const override;

	std::string getDescription() const override;
	bool PTG_IsIntoDomain(double x, double y) const override;
	void ptgDiffDriveSteeringFunction(
		float alpha, float t, float x, float y, float phi, float& v,
		float& w) const override;
	void loadDefaultParams() override;

   protected:
	double R, K;
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(
	CPTG_DiffDrive_CS, CParameterizedTrajectoryGenerator, NAV_IMPEXP)
}
}
