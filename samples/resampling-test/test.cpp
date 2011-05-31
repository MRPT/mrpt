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

#include <mrpt/base.h>
#include <mrpt/slam.h>

using namespace mrpt::utils;
using namespace mrpt::bayes;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace std;

double MIN_LOG_WEIG	= -1.0;

unsigned int N_TESTS = 500;
int N_PARTICLES = 100;

// For batch experiment:
vector_double				min_log_ws;
map< string, vector_double > results;


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
		vector_double errs_hist = lin_ws - hist_parts; \
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
	vector_double	log_ws;
	vector_size_t	out_indxs;


	const size_t M = N_PARTICLES;

	log_ws.resize(M);
	//vectorToTextFile( log_ws, "log_ws.txt");

	// Compute normalized linear weights:
	vector_double lin_ws;


	vector_double 	hist_parts;
	vector_double	ERR_MEANs;
	vector_double	ERR_STDs;

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
	vector_double R;

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


