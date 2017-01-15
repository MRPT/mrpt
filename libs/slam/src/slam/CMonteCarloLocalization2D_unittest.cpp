/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}


void run_test_pf_localization(CPose2D &meanPose, CMatrixDouble33 &cov)
{
// ------------------------------------------------------
// The code below is a simplification of the program "pf-localization"
// ------------------------------------------------------
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

}

// TEST =================
TEST(MonteCarlo2D, RunSampleDataset)
{
#if MRPT_IS_BIG_ENDIAN
	MRPT_TODO("Debug this issue in big endian platforms")
	return; // Skip this test for now
#endif

	// Actual ending point:
	const CPose2D  GT_endpose(15.904,-10.010,DEG2RAD(4.93));

	// Placeholder for results:
	CPose2D meanPose;
	CMatrixDouble33 cov;

	// Invoke test:
	// Give it 3 opportunities, since it might fail once for bad luck, or even twice in an extreme bad luck:
	for (int op=0;op<3;op++)
	{
		run_test_pf_localization(meanPose,cov);

		const double  final_pf_cov_trace = cov.trace();
		const CPose2D final_pf_pose      = meanPose;

		bool pass1 = (final_pf_pose-GT_endpose).norm() < 0.10; 
		bool pass2 = final_pf_cov_trace < 0.01;

		if (pass1 && pass2) 
			return; // OK! 

		// else: give it another try...
		cout << "\n*Warning: Test failed. Will give it another chance, since after all it's nondeterministic!\n";
	}

	FAIL() << "Failed to converge after 3 opportunities!!" << endl;
}

