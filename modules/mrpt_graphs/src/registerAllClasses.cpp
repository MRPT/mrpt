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

#include <mrpt/core/initializer.h>
#include <mrpt/graphs/registerAllClasses.h>
// Deps:
#include <mrpt/poses/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_graphs)  // NOLINT(misc-use-anonymous-namespace)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  //	registerClass( CLASS_ID( ... ) );

#endif
}

void mrpt::graphs::registerAllClasses_mrpt_graphs()
{
  ::registerAllClasses_mrpt_graphs();
  // deps:
  mrpt::poses::registerAllClasses_mrpt_poses();
}
