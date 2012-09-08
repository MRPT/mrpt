/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

// Defined in run_unittests.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}


TEST(MonteCarlo2D, RunSampleDataset)
{
// ------------------------------------------------------
// The code below is a simplification of the program "pf-localization"
// ------------------------------------------------------
#if MRPT_IS_BIG_ENDIAN
	MRPT_TODO("Debug this issue in big endian platforms")
	return; // Skip this test for now
#endif

	const string ini_fil = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/montecarlo_test1.ini");
	if (!mrpt::system::fileExists(ini_fil))
	{
		cerr << "WARNING: Skipping test due to missing file: " << ini_fil << "\n";
		return;
	}

	CConfigFile	iniFile(ini_fil);
	vector_int			particles_count;	// Number of initial particles (if size>1, run the experiments N times)

	// Load configuration:
	// -----------------------------------------
	string iniSectionName ( "LocalizationExperiment" );

	// Mandatory entries:
	iniFile.read_vector(iniSectionName, "particles_count", vector_int(1,0), particles_count, /*Fail if not found*/true );
	string		RAWLOG_FILE			= iniFile.read_string(iniSectionName,"rawlog_file","", /*Fail if not found*/true );

	RAWLOG_FILE = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/") + RAWLOG_FILE;

	// Non-mandatory entries:
	string		MAP_FILE			= iniFile.read_string(iniSectionName,"map_file","" );

	MAP_FILE = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/") + MAP_FILE;

	size_t		rawlog_offset		= iniFile.read_int(iniSectionName,"rawlog_offset",0);
	int		NUM_REPS			= iniFile.read_int(iniSectionName,"experimentRepetitions",1);

	// PF-algorithm Options:
	// ---------------------------
	CParticleFilter::TParticleFilterOptions		pfOptions;
	pfOptions.loadFromConfigFile( iniFile, "PF_options" );

	// PDF Options:
	// ------------------
	TMonteCarloLocalizationParams	pdfPredictionOptions;
	pdfPredictionOptions.KLD_params.loadFromConfigFile( iniFile, "KLD_options");

	// Metric map options:
	// -----------------------------
	TSetOfMetricMapInitializers				mapList;
	mapList.loadFromConfigFile( iniFile,"MetricMap");

	// --------------------------------------------------------------------
	//						EXPERIMENT PREPARATION
	// --------------------------------------------------------------------
	CTicTac		tictac,tictacGlobal;
	CSimpleMap	simpleMap;
	CRawlog		rawlog;
	size_t		rawlogEntry, rawlogEntries;
	CParticleFilter::TParticleFilterStats	PF_stats;

	// Load the set of metric maps to consider in the experiments:
	CMultiMetricMap							metricMap;
	metricMap.setListOfMaps( &mapList );

	randomGenerator.randomize();

	// Load the map (if any):
	// -------------------------
	if (MAP_FILE.size())
	{
		ASSERT_( fileExists(MAP_FILE) );

		// Detect file extension:
		// -----------------------------
		string mapExt = lowerCase( extractFileExtension( MAP_FILE, true ) ); // Ignore possible .gz extensions

		if ( !mapExt.compare( "simplemap" ) )
		{
			// It's a ".simplemap":
			// -------------------------
			CFileGZInputStream(MAP_FILE.c_str()) >> simpleMap;

			ASSERT_( simpleMap.size()>0 );

			// Build metric map:
			// ------------------------------
			metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
		}
		else if ( !mapExt.compare( "gridmap" ) )
		{
			// It's a ".gridmap":
			// -------------------------
			ASSERT_( metricMap.m_gridMaps.size()==1 );
			CFileGZInputStream(MAP_FILE) >> (*metricMap.m_gridMaps[0]);
		}
		else
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
		}

	}

	// --------------------------
	// Load the rawlog:
	// --------------------------
	rawlog.loadFromRawLogFile(RAWLOG_FILE);
	rawlogEntries = rawlog.size();

	CPose2D              meanPose;
	CMatrixDouble33      cov;


	for ( vector_int::iterator itNum = particles_count.begin(); itNum!=particles_count.end(); ++itNum )
	{
		int		PARTICLE_COUNT = *itNum;


		// Global stats for all the experiment loops:
		vector<double> 	covergenceErrors;
		covergenceErrors.reserve(NUM_REPS);
		// --------------------------------------------------------------------
		//					EXPERIMENT REPETITIONS LOOP
		// --------------------------------------------------------------------
		tictacGlobal.Tic();
		for (int repetition = 0; repetition <NUM_REPS; repetition++)
		{
			int						M = PARTICLE_COUNT;
			CMonteCarloLocalization2D  pdf(M);

			// PDF Options:
			pdf.options = pdfPredictionOptions;

			pdf.options.metricMap = &metricMap;

			// Create the PF object:
			CParticleFilter	PF;
			PF.m_options = pfOptions;

			size_t	step = 0;
			rawlogEntry = 0;

			// Initialize the PDF:
			// -----------------------------
			tictac.Tic();
			if ( !iniFile.read_bool(iniSectionName,"init_PDF_mode",false, /*Fail if not found*/true) )
				pdf.resetUniformFreeSpace(
					metricMap.m_gridMaps[0].pointer(),
					0.7f,
					PARTICLE_COUNT ,
					iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180))
					);
			else
				pdf.resetUniform(
					iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180)),
					PARTICLE_COUNT
					);


			// -----------------------------
			//		Particle filter
			// -----------------------------
			CActionCollectionPtr action;
			CSensoryFramePtr     observations;
			bool				end = false;

			//TTimeStamp cur_obs_timestamp;

			while (rawlogEntry<(rawlogEntries-1) && !end)
			{
				// Finish if ESC is pushed:
				if (os::kbhit())
					if (os::getch()==27)
						end = true;

				// Load pose change from the rawlog:
				// ----------------------------------------
				if (!rawlog.getActionObservationPair(action, observations, rawlogEntry ))
					THROW_EXCEPTION("End of rawlog");

				CPose2D		expectedPose; // Ground truth

//				if (observations->size()>0)
//					cur_obs_timestamp = observations->getObservationByIndex(0)->timestamp;

				if (step>=rawlog_offset)
				{
					// Do not execute the PF at "step=0", to let the initial PDF to be
					//   reflected in the logs.
					if (step>rawlog_offset)
					{

						// ----------------------------------------
						// RUN ONE STEP OF THE PARTICLE FILTER:
						// ----------------------------------------
						tictac.Tic();

						PF.executeOn(
							pdf,
							action.pointer(),			// Action
							observations.pointer(),	// Obs.
							&PF_stats		// Output statistics
							);

					}

					pdf.getCovarianceAndMean(cov,meanPose);
					//cout << meanPose << " cov trace: "  << cov.trace() <<  endl;

				} // end if rawlog_offset

				step++;

			}; // while rawlogEntries
		} // for repetitions
	} // end of loop for different # of particles



	// TEST =================
	// Actual ending point:
	const CPose2D  GT_endpose(15.904,-10.010,DEG2RAD(4.93));

	const double  final_pf_cov_trace = cov.trace();
	const CPose2D final_pf_pose      = meanPose;

	EXPECT_NEAR( (final_pf_pose-GT_endpose).norm(),0, 0.10 )
		<< "Final pose: " << final_pf_pose << endl << "Expected: " << GT_endpose << endl;

	EXPECT_TRUE(final_pf_cov_trace < 0.01 )
		<< "final_pf_cov_trace = " << final_pf_cov_trace << endl;
}
