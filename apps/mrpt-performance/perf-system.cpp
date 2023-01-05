/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "common.h"

double clock_now_test(int, int)
{
	const unsigned int step = 10000000;
	mrpt::system::CTicTac tictac;
	std::string s;
	tictac.Tic();
	for (unsigned int i = 0; i < step; i++)
	{
		auto t = mrpt::Clock::now();
		(void)t;
	}
	return tictac.Tac() / step;
}

double clock_nowDouble_test(int, int)
{
	const unsigned int step = 10000000;
	mrpt::system::CTicTac tictac;
	std::string s;
	tictac.Tic();
	double dummy = 0;
	for (unsigned int i = 0; i < step; i++)
	{
		double t = mrpt::Clock::nowDouble();
		dummy += t;
	}
	return (tictac.Tac() / step) + 1e-100 * dummy;	// avoid compiler optim.
}

// ------------------------------------------------------
// register_tests_system
// ------------------------------------------------------
void register_tests_system()
{
	lstTests.emplace_back("core: mrpt::Clock::now()", &clock_now_test);
	lstTests.emplace_back(
		"core: mrpt::Clock::nowDouble()", &clock_nowDouble_test);
}
