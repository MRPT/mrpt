/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CGPS_NTRIP.h>
#include <mrpt/utils/CConfigFilePrefixer.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CGPS_NTRIP,mrpt::hwdrivers)

/** Constructor. See mrpt::hwdrivers::CGPSInterface for the meaning of params. */
CGPS_NTRIP::CGPS_NTRIP() :
	gps(),
	ntrip()
{
}

/** Destructor */
CGPS_NTRIP::~CGPS_NTRIP()
{
}

void CGPS_NTRIP::initialize()
{
    gps.initialize();
    ntrip.initialize();
}

// See docs in parent class
void  CGPS_NTRIP::doProcess()
{
	// Process GPS:
	gps.doProcess();

	// Move sensed observations to this object:
	{
		TListObservations lst;
		gps.getObservations(lst);

		std::vector<mrpt::utils::CSerializablePtr> vect;
		vect.reserve(lst.size());
		for (TListObservations::const_iterator it=lst.begin();it!=lst.end();++it)
			vect.push_back( it->second );
		this->appendObservations(vect);
	}

	// New GGA frames?
	std::string sLastGGA = gps.getLastGGA();
	if (!sLastGGA.empty())
	{
		if (m_verbose) cout << "[CGPS_NTRIP] Redirecting GGA frame from GPS->NTRIP: '" << sLastGGA << "'" << endl;

		ntrip.getNTRIPClient().sendBackToServer( sLastGGA + std::string("\r\n") );
	}

	// Process NTRIP server comms:
	ntrip.doProcess();
}

void CGPS_NTRIP::loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase &cfg, const std::string &section )
{
	// Load GPS params:
	gps.loadConfig( mrpt::utils::CConfigFilePrefixer(cfg,"","gps_"), section );
	// NTRIP params:
	ntrip.loadConfig( mrpt::utils::CConfigFilePrefixer(cfg,"","ntrip_"), section );

	// Own params:
	// (none yet)
}
