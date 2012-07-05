/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
