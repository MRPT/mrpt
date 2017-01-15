/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace mrpt::poses;
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
	mrpt::maps::TSetOfMetricMapInitializers				mapInit;
	mapInit.loadFromConfigFile( CConfigFile("_demo_map.ini"),"MetricMap");
	map.setListOfMaps( &mapInit );

	// Create a dummy observation:
	mrpt::obs::CObservationBeaconRanges					obs;
	mrpt::obs::CObservationBeaconRanges::TMeasurement	meas;
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

