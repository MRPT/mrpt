/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
C2DRangeFinderAbstract::C2DRangeFinderAbstract() :
	m_lastObservation		( ),
	m_lastObservationIsNew	( false ),
	m_hardwareError			( false ),
	m_nextObservation		(),
	m_stream				( NULL  )
{
}

/*-------------------------------------------------------------
						Destructor
-------------------------------------------------------------*/
C2DRangeFinderAbstract::~C2DRangeFinderAbstract()
{
}

/*-------------------------------------------------------------
						bindIO
-------------------------------------------------------------*/
void  C2DRangeFinderAbstract::bindIO( CStream	*streamIO )
{
	m_csChangeStream.enter();
	m_stream = streamIO;
	m_csChangeStream.leave();
}

/*-------------------------------------------------------------
						getObservation
-------------------------------------------------------------*/
void  C2DRangeFinderAbstract::getObservation(
	bool							&outThereIsObservation,
	mrpt::slam::CObservation2DRangeScan	&outObservation,
	bool							&hardwareError )
{
	m_csLastObservation.enter();

	hardwareError			= m_hardwareError;
	outThereIsObservation	= m_lastObservationIsNew;

	if (outThereIsObservation)
		outObservation = m_lastObservation;

	m_csLastObservation.leave();
}

/*-------------------------------------------------------------
						doProcess
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::doProcess()
{
	bool	thereIs, hwError;

	if (!m_nextObservation)
		m_nextObservation =  CObservation2DRangeScan::Create();

	doProcessSimple( thereIs, *m_nextObservation, hwError );

	if (hwError)
	{
		m_state = ssError;
	    THROW_EXCEPTION("Couldn't communicate to the USB board!");
	}

	if (thereIs)
	{
		m_state = ssWorking;

		appendObservation( m_nextObservation );
		m_nextObservation.clear_unique(); // Create a new object in the next call
	}
}

/*-------------------------------------------------------------
						loadExclusionAreas
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::loadExclusionAreas(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{

	// Load exclusion areas:
	m_lstExclusionPolys.clear();
	m_lstExclusionAngles.clear();

	unsigned int N = 1;

	for(;;)
	{
		vector<double> x,y, z_range;
		configSource.read_vector( iniSection, format("exclusionZone%u_x",N), vector<double>(0), x);
		configSource.read_vector( iniSection, format("exclusionZone%u_y",N), vector<double>(0), y);
		configSource.read_vector( iniSection, format("exclusionZone%u_z",N++), vector<double>(0), z_range);

		if (!x.empty() && !y.empty())
		{
			ASSERT_(x.size()==y.size())

			CObservation2DRangeScan::TListExclusionAreasWithRanges::value_type dat;

			dat.first.setAllVertices(x,y);
			if (z_range.empty())
			{
				dat.second.first  = -std::numeric_limits<double>::max();
				dat.second.second =  std::numeric_limits<double>::max();
			}
			else
			{
				ASSERTMSG_(z_range.size()==2,"exclusionZone%u_z must be a range [z_min z_max]");
				ASSERT_(z_range[0]<=z_range[1]);

				dat.second.first  = z_range[0];
				dat.second.second = z_range[1];
			}

			m_lstExclusionPolys.push_back(dat);
		}
		else break;
	}


	// Load forbiden angles;
	N = 1;

	for(;;)
	{
		const double ini = DEG2RAD( configSource.read_double( iniSection, format("exclusionAngles%u_ini",N), -1000 ) );
		const double end = DEG2RAD( configSource.read_double( iniSection, format("exclusionAngles%u_end",N++), -1000 ) );

		if (ini>-M_PI && end>-M_PI)
		     m_lstExclusionAngles.push_back(make_pair(ini,end));
		else break;
	}
}

/*-------------------------------------------------------------
						filterByExclusionAreas
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::filterByExclusionAreas( mrpt::slam::CObservation2DRangeScan &obs) const
{
	obs.filterByExclusionAreas( m_lstExclusionPolys );
}

/*-------------------------------------------------------------
						filterByExclusionAngles
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::filterByExclusionAngles( mrpt::slam::CObservation2DRangeScan &obs) const
{
	obs.filterByExclusionAngles( m_lstExclusionAngles );
}
