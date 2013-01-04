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

#include <mrpt/base.h>
#include <mrpt/graphs.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
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
//	cout << "Type: " << TTypeName<mrpt::slam::COccupancyGridMap2D>::get() << endl;

	cout << "Type: " << TTypeName<vector_double>::get() << endl;
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
