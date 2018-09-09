/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

namespace mrpt::nav
{
/** A PTG for optimal paths of type "C|C" , as named in PTG papers.
 * - **Compatible kinematics**: differential-driven / Ackermann steering
 * - **Compatible robot shape**: Arbitrary 2D polygon
 * - **PTG parameters**: Use the app `ptg-configurator`
 *
 *  See also "Obstacle Distance for Car-Like Robots", IEEE Trans. Rob. And
 * Autom, 1999.
 * \note [Before MRPT 1.5.0 this was named CPTG4]
 *  \ingroup nav_tpspace
 */
class CPTG_DiffDrive_CC : public CPTG_DiffDrive_CollisionGridBased
{
	DEFINE_SERIALIZABLE(CPTG_DiffDrive_CC)
   public:
	CPTG_DiffDrive_CC() = default;
	CPTG_DiffDrive_CC(
		const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
	{
		loadFromConfigFile(cfg, sSection);
	}
	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) override;
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) const override;

	std::string getDescription() const override;
	bool PTG_IsIntoDomain(double x, double y) const override;
	void ptgDiffDriveSteeringFunction(
		float alpha, float t, float x, float y, float phi, float& v,
		float& w) const override;
	void loadDefaultParams() override;

   protected:
	double R{0}, K{0};
};
}  // namespace mrpt::nav
