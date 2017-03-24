/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CAtan2LookUpTable.h>
#include <mrpt/random.h>
#include <cmath>

#include "common.h"

double atan2_lut_test_build(int , int )
{
	const unsigned int step = 100;
	mrpt::utils::CTicTac tictac;
	mrpt::math::CAtan2LookUpTable lut;
	tictac.Tic();
	for (unsigned int i=0;i<step;i++)
	{
		lut.resize(-10.0,10.0, -10.0,10.0, 0.01);
	}
	return tictac.Tac()/step;
}

double atan2_lut_test_query(int , int )
{
	const unsigned int step = 1000000;
	mrpt::utils::CTicTac tictac;
	mrpt::math::CAtan2LookUpTable lut;
	lut.resize(-10.0,10.0, -10.0,10.0, 0.01);
	tictac.Tic();
	double atan2val;
	double x=-9.0,y=-8.0;
	const double dx = 18.0/step, dy = 12.5/step;
	for (unsigned int i=0;i<step;i++)
	{
		x+=dx; y+=dy;
		//bool valid =
		lut.atan2(y,x,atan2val);
	}
	return tictac.Tac()/step;
}

double atan2_lut_multires_test_build(int , int )
{
	const unsigned int step = 10;

	mrpt::math::CAtan2LookUpTableMultiRes atan2lut;

	std::map<double,double> res2extension;
	res2extension[0.001] = 0.8;   // 0.1 cm resolution
	res2extension[0.01]  = 2.0;   // 1.0 cm resolution
	res2extension[0.02] = 5.0;    // 2.0 cm resolution
	res2extension[0.05] = 11.0;   // 5.0 cm resolution

	mrpt::utils::CTicTac tictac;
	tictac.Tic();
	for (unsigned int i=0;i<step;i++)
	{
	atan2lut.resize(res2extension);
	}
	return tictac.Tac()/step;
}

double atan2_lut_multires_test_query(int , int )
{
	const unsigned int step = 1000000;
	mrpt::utils::CTicTac tictac;
	mrpt::math::CAtan2LookUpTableMultiRes atan2lut;

	std::map<double,double> res2extension;
	res2extension[0.001] = 0.8;   // 0.1 cm resolution
	res2extension[0.01]  = 2.0;   // 1.0 cm resolution
	res2extension[0.02] = 5.0;    // 2.0 cm resolution
	res2extension[0.05] = 11.0;   // 5.0 cm resolution
	atan2lut.resize(res2extension);

	tictac.Tic();
	double atan2val;
	double x=-9.0,y=-8.0;
	const double dx = 18.0/step, dy = 12.5/step;
	for (unsigned int i=0;i<step;i++)
	{
		x+=dx; y+=dy;
		//bool valid =
		atan2lut.atan2(y,x,atan2val);
	}
	return tictac.Tac()/step;
}

double atan2_raw_test_query(int , int )
{
	const unsigned int step = 1000000;
	mrpt::utils::CTicTac tictac;

	double atan2val=.0;
	double x=-9.0,y=-8.0;
	const double dx = 18.0/step, dy = 12.5/step;
	for (unsigned int i=0;i<step;i++)
	{
		x+=dx; y+=dy;
		atan2val+= ::atan2(y,x);
	}
	const double t = tictac.Tac()/step;
	{
		std::stringstream ss;
		ss << atan2val;
	}
	return t;
}


// ------------------------------------------------------
// register_tests_atan2lut
// ------------------------------------------------------
void register_tests_atan2lut()
{
	lstTests.push_back( TestData("CAtan2LUT: 20x20m,1cm cells,build", atan2_lut_test_build ) );
	lstTests.push_back( TestData("CAtan2LUT: 20x20m,1cm cells,query", atan2_lut_test_query ) );

	lstTests.push_back( TestData("CAtan2LUTMultiRes: 22x22m,build", atan2_lut_multires_test_build ) );
	lstTests.push_back( TestData("CAtan2LUTMultiRes: 22x22m,query", atan2_lut_multires_test_query ) );

	lstTests.push_back( TestData("CAtan2LUT: raw ::atan2() call", atan2_raw_test_query ) );
}
