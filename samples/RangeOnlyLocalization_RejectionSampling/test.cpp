/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include <mrpt/slam.h>
#include <mrpt/base.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace std;

float	SIGMA = 0.03f;

// ------------------------------------------------------
//				TestRS
// ------------------------------------------------------
void TestRS()
{
	randomGenerator.randomize();

	// Load the map:
	CMultiMetricMap									map;
	mrpt::slam::TSetOfMetricMapInitializers				mapInit;
	mapInit.loadFromConfigFile( CConfigFile("_demo_map.ini"),"MetricMap");
	map.setListOfMaps( &mapInit );

	// Create a dummy observation:
	mrpt::slam::CObservationBeaconRanges					obs;
	mrpt::slam::CObservationBeaconRanges::TMeasurement	meas;
	obs.stdError = SIGMA;

	meas.beaconID = 0;
	meas.sensedDistance = 2.1f;
	meas.sensorLocationOnRobot = CPoint3D(0,0,0);
	obs.sensedData.push_back( meas );

	meas.beaconID = 1;
	meas.sensedDistance = 3.1f;
	meas.sensorLocationOnRobot = CPoint3D(0,0,0);
	obs.sensedData.push_back( meas );

	meas.beaconID = 2;
	meas.sensedDistance = 1.1f;
	meas.sensorLocationOnRobot = CPoint3D(0,0,0);
//	obs.sensedData.push_back( meas );

	// Rejection Sampling:
	CRejectionSamplingRangeOnlyLocalization								RS;
	vector<CRejectionSamplingRangeOnlyLocalization::TParticle>		samples;
	CTicTac		tictac;

	// Set data:
	ASSERT_(map.m_landmarksMap);

	printf("Preparing...");
	tictac.Tic();
		CPose2D	dumPose(0,0,0);
		RS.setParams( *map.m_landmarksMap, obs, SIGMA, dumPose );
	printf("Ok! %fms\n",1000*tictac.Tac());

	printf("Computing...");
	tictac.Tic();
		RS.rejectionSampling( 1000,samples, 1000 );
	printf("Ok! %fms\n",1000*tictac.Tac());

	FILE	*f = os::fopen( "_out_samples.txt","wt");
	vector<CRejectionSamplingRangeOnlyLocalization::TParticle>::iterator	it;
	for (it=samples.begin();it!=samples.end();it++)
		os::fprintf(f,"%f %f %f %e\n",it->d->x(),it->d->y(),it->d->phi(),it->log_w );


	os::fclose(f);
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		 TestRS();

		return 0;
	} catch (exception &e)
	{
		cout << "EXCEPCTION: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped excepcion!!");
		return -1;
	}
}

