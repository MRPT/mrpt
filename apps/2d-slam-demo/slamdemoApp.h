/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

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
	void DoBatchExperiments(const std::string &cfgFil);

	slamdemoFrame* win;

    public:
        virtual bool OnInit();

	private:
		bool  m_option_norun;
};

#endif // SLAMDEMOAPP_H
