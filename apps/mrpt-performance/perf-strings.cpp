/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "common.h"

double format_test1(int, int)
{
	const unsigned int step = 10000;
	mrpt::system::CTicTac tictac;
	std::string s;
	tictac.Tic();
	for (unsigned int i = 0; i < step; i++)
	{
		s = mrpt::format("this is step #%u in the loop", i);
	}
	return tictac.Tac() / step;
}

double format_test2(int a, int b)
{
	const unsigned int step = 10000;
	mrpt::system::CTicTac tictac;
	std::string s;
	tictac.Tic();
	for (unsigned int i = 0; i < step; i++)
	{
		s = mrpt::format(
			"Lorem ipsum dolor sit amet, consectetur adipiscing elit, %i sed "
			"do eiusmod tempor incididunt ut "
			"labore et dolore magna aliqua. Ut enim ad minim veniam, quis "
			"nostrud exercitation ullamco laboris "
			"nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in "
			"%u reprehenderit in voluptate velit "
			"esse cillum dolore eu %i fugiat nulla pariatur. Excepteur sint "
			"occaecat cupidatat non proident, sunt in "
			"culpa qui officia deserunt mollit anim id est laborum."
			"Sed ut perspiciatis unde omnis iste natus error sit voluptatem "
			"accusantium doloremque laudantium, totam "
			"rem aperiam, eaque ipsa quae ab illo inventore veritatis et quasi "
			"architecto beatae vitae dicta sunt explicabo."
			" Nemo enim ipsam voluptatem quia voluptas sit aspernatur aut odit "
			"aut fugit, sed quia consequuntur magni dolores "
			"eos qui ratione voluptatem sequi nesciunt. Neque porro quisquam "
			"est, qui dolorem ipsum quia dolor sit amet, "
			"consectetur, adipisci velit, sed quia non numquam eius modi "
			"tempora incidunt ut labore et dolore magnam "
			"aliquam quaerat voluptatem. Ut enim ad minima veniam, quis "
			"nostrum exercitationem ullam corporis suscipit "
			"laboriosam, nisi ut aliquid ex ea commodi consequatur? Quis autem "
			"vel eum iure reprehenderit qui in ea "
			"voluptate velit esse quam nihil molestiae consequatur, vel illum "
			"qui dolorem eum fugiat quo voluptas nulla pariatur?",
			a, i, b);
	}
	return tictac.Tac() / step;
}
// ------------------------------------------------------
// register_tests_strings
// ------------------------------------------------------
void register_tests_strings()
{
	lstTests.emplace_back("Strings: mrpt::format() short str", &format_test1);
	lstTests.emplace_back("Strings: mrpt::format() long str", &format_test2);
}
