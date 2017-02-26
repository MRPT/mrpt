/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/random.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <map>

using namespace mrpt::utils;
using namespace mrpt::bayes;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

double MIN_LOG_WEIG	= -1.0;

unsigned int N_TESTS = 500;
int N_PARTICLES = 100;

// For batch experiment:
CVectorDouble				min_log_ws;
map< string, CVectorDouble > results;


// vectorToTextFile( out_indxs, #ALGOR, true, true); /* By rows, append */

#define TEST_RESAMPLING(ALGOR) \
	mrpt::system::deleteFile(#ALGOR); \
	/*printf(#ALGOR);*/\
	/*printf("\n");*/\
	ERR_MEANs.clear(); \
	ERR_STDs.clear(); \
	for (size_t i=0;i<N_TESTS;i++)\
	{\
		mrpt::random::randomGenerator.drawUniformVector(log_ws, MIN_LOG_WEIG, 0.0); \
		CParticleFilterCapable::log2linearWeights(log_ws, lin_ws); \
		CParticleFilterCapable::computeResampling( CParticleFilter::ALGOR, log_ws, out_indxs );\
		hist_parts = mrpt::math::histogram( out_indxs, 0, M-1, M, true); \
		vector<double> errs_hist = lin_ws - hist_parts; \
		ERR_MEANs.push_back( mrpt::math::mean(errs_hist) );\
		ERR_STDs.push_back ( mrpt::math::stddev(errs_hist) );\
	}\
	printf("%s: ERR_MEAN %e\n", #ALGOR, mrpt::math::mean(ERR_MEANs) ); \
	printf("%s: ERR_STD %f\n", #ALGOR, mrpt::math::mean(ERR_STDs) ); \
	results[#ALGOR].push_back( mrpt::math::mean(ERR_STDs) ); \


// ------------------------------------------------------
//                  TestResampling
// ------------------------------------------------------
void TestResampling()
{
	vector<double>	log_ws;
	mrpt::vector_size_t	out_indxs;


	const size_t M = N_PARTICLES;

	log_ws.resize(M);
	//vectorToTextFile( log_ws, "log_ws.txt");

	// Compute normalized linear weights:
	vector<double> lin_ws;
	vector<double> hist_parts;
	vector<double> ERR_MEANs;
	vector<double> ERR_STDs;

	// prMultinomial
	TEST_RESAMPLING(prMultinomial)
	// prResidual
	TEST_RESAMPLING(prResidual)
	// prStratified
	TEST_RESAMPLING(prStratified)
	// prSystematic
	TEST_RESAMPLING(prSystematic)
}


void TestBatch()
{
	for (double LL=-2;LL<=2.01;LL+=0.08)
	{
		double L = pow(10.0,LL);

		min_log_ws.push_back( L );
		printf("MIN_LOG_W=%f\n", L );

		MIN_LOG_WEIG = L;
		TestResampling();
	}

	// Save results to files:
	CVectorDouble R;

	vectorToTextFile( min_log_ws, "min_log_ws.txt");

	R = results["prMultinomial"];
	vectorToTextFile(R,"prMultinomial.txt");
	R = results["prResidual"];
	vectorToTextFile(R,"prResidual.txt");
	R = results["prStratified"];
	vectorToTextFile(R,"prStratified.txt");
	R = results["prSystematic"];
	vectorToTextFile(R,"prSystematic.txt");
}

// ------------------------------------------------------
//                        MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		randomGenerator.randomize();

		if (argc>1)
			N_PARTICLES = atoi(argv[1]);

		//TestResampling();
		TestBatch();

		return 0;
	} catch (exception &e)
	{
		cerr << "EXCEPCTION: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		cerr << "Untyped excepcion!!";
		return -1;
	}
}


