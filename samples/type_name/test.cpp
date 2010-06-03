/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::slam;
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
	cout << "Type: " << TTypeName<mrpt::slam::COccupancyGridMap2D>::get() << endl;

	cout << "Type: " << TTypeName<vector_double>::get() << endl;
	cout << "Type: " << TTypeName<vector<int32_t> >::get() << endl;
	cout << "Type: " << TTypeName<set<double> >::get() << endl;

	cout << "Type: " << TTypeName<set< vector<double> > >::get() << endl;

	cout << "Type: " << TTypeName<pair<int32_t,int32_t> >::get() << endl;
	cout << "Type: " << TTypeName<pair<int32_t, pair<int32_t,int32_t> > >::get() << endl;

	cout << "Type: " << TTypeName<map< double, set<int32_t> > >::get() << endl;

	cout << "Type: " << TTypeName<set< multimap<double, pair<CPose3DPDFGaussian, COccupancyGridMap2D> > > >::get() << endl;

	cout << "Type: " << TTypeName<CMatrixDouble33>::get() << endl;
	cout << "Type: " << TTypeName<map<CMatrixDouble13,CMatrixFloat> >::get() << endl;

	cout << "Type: " << TTypeName<CArrayNumeric<double,3> >::get() << endl;
	cout << "Type: " << TTypeName<CArrayDouble<5> >::get() << endl;

	cout << "Jacob of R^5 => R^3: " << TTypeName<MAT_TYPEDECL_JACOBIAN_OF( CArrayDouble<5>, CArrayDouble<3>)>::get() << endl;
	cout << "Jacob of DYN => R^3: " << TTypeName<MAT_TYPEDECL_JACOBIAN_OF( vector_double, CArrayDouble<3>)>::get() << endl;
	cout << "Jacob of R^4 => DYN: " << TTypeName<MAT_TYPEDECL_JACOBIAN_OF( CArrayDouble<4>,vector_double)>::get() << endl;
	cout << "Jacob of DYN => DYN: " << TTypeName<MAT_TYPEDECL_JACOBIAN_OF( vector_double, vector_double)>::get() << endl;

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
