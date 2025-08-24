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

#ifndef _DSCENEVIEWERAPP_H
#define _DSCENEVIEWERAPP_H

#include <wx/app.h>

class _DSceneViewerApp : public wxApp
{
 public:
  bool OnInit() override;
  int OnExit() override;
};

#endif  // _DSCENEVIEWERAPP_H
