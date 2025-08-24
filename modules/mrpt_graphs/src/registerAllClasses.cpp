/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "graphs-precomp.h"  // Precompiled headers
//
#include <mrpt/core/initializer.h>
#include <mrpt/graphs/registerAllClasses.h>
// Deps:
#include <mrpt/viz/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_graphs)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  //	registerClass( CLASS_ID( ... ) );

#endif
}

void mrpt::graphs::registerAllClasses_mrpt_graphs()
{
  ::registerAllClasses_mrpt_graphs();
  // deps:
  mrpt::viz::registerAllClasses_mrpt_viz();
}
