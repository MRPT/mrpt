/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
