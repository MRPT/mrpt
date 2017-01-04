/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/TTypeName.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;


/* ------------------------------------------------------------------------
					Test: Type Names

		Demonstration of MRPT's compile-time template TTypeName, which
		recursively obtains the name of arbitrarily complex data types.
   ------------------------------------------------------------------------ */
void Test_TypeName()
{
	cout << "Type: " << TTypeName<int32_t>::get() << endl;
	cout << "Type: " << TTypeName<double>::get() << endl;
	cout << "Type: " << TTypeName<CPose2D>::get() << endl;
//	cout << "Type: " << TTypeName<mrpt::maps::COccupancyGridMap2D>::get() << endl;

	cout << "Type: " << TTypeName<vector<double> >::get() << endl;
	cout << "Type: " << TTypeName<vector<int32_t> >::get() << endl;
	cout << "Type: " << TTypeName<set<double> >::get() << endl;

	cout << "Type: " << TTypeName<set< vector<double> > >::get() << endl;

	cout << "Type: " << TTypeName<pair<int32_t,int32_t> >::get() << endl;
	cout << "Type: " << TTypeName<pair<int32_t, pair<int32_t,int32_t> > >::get() << endl;

	cout << "Type: " << TTypeName<map< double, set<int32_t> > >::get() << endl;

	cout << "Type: " << TTypeName<set< multimap<double, pair<CPose3DPDFGaussian, /*COccupancyGridMap2D*/ CPose2D > > > >::get() << endl;

	cout << "Type: " << TTypeName<CMatrixDouble33>::get() << endl;
	cout << "Type: " << TTypeName<map<CMatrixDouble13,CMatrixFloat> >::get() << endl;

	cout << "Type: " << TTypeName<CArrayNumeric<double,3> >::get() << endl;
	cout << "Type: " << TTypeName<CArrayDouble<5> >::get() << endl;

	cout << "Type: " << TTypeName< deque<vector<vector<int32_t> > > >::get() << endl;

	// classes in mrpt::graphs
	cout << "Type: " << TTypeName< mrpt::graphs::CNetworkOfPoses2D >::get() << endl;

}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		Test_TypeName();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}
