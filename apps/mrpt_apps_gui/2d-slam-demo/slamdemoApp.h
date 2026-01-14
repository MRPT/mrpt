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

#ifndef SLAMDEMOAPP_H
#define SLAMDEMOAPP_H

#include <wx/app.h>

/* Jerome Monceaux : 2011/03/08
 * Include <string> needed under snow leopard
 */
#include <string>

class slamdemoFrame;

class slamdemoApp : public wxApp
{
  bool doCommandLineProcess();
  void DoBatchExperiments(const std::string& cfgFil);

  slamdemoFrame* win;

 public:
  bool OnInit() override;

 private:
  bool m_option_norun;
};

#endif  // SLAMDEMOAPP_H
