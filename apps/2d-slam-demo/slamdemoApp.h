/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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

#endif	// SLAMDEMOAPP_H
