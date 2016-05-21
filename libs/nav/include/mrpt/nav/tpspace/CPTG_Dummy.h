/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

namespace mrpt
{
namespace nav
{
	/** A dummy PTG, used mainly to call loadTrajectories() without knowing the exact derived PTG class and still be able to analyze the trajectories. */
	class NAV_IMPEXP  CPTG_Dummy : public CPTG_DiffDrive_CollisionGridBased
	{
	public:
		// See base class docs
		CPTG_Dummy() : CPTG_DiffDrive_CollisionGridBased() {}
		virtual ~CPTG_Dummy() { }
		virtual std::string getDescription() const { return m_text_description; }
		virtual std::string loadTrajectories( mrpt::utils::CStream &in ) 
		{
			m_text_description = CPTG_DiffDrive_CollisionGridBased::loadTrajectories(in);
			return m_text_description;
		}
		virtual void ptgDiffDriveSteeringFunction( float alpha, float t, float x, float y, float phi, float &v, float &w) { throw std::runtime_error("Should not call this method in a dummy PTG!");  }
		virtual bool PTG_IsIntoDomain( float x, float y )  { throw std::runtime_error("Should not call this method in a dummy PTG!");  }

	private:
		std::string m_text_description;
	};
}
}
