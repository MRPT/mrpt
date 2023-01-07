/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

// JLBC: Unix X headers have these funny things...
#ifdef None
#undef None
#endif

#include <mrpt/obs/customizable_obs_viz.h>

class ViewOptions3DPoints;

struct ParametersView3DPoints : public mrpt::obs::VisualizationParameters
{
	ParametersView3DPoints() = default;

	void to_UI(ViewOptions3DPoints& ui) const;
	void from_UI(const ViewOptions3DPoints& ui);
};
