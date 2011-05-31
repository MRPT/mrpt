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

#include <mrpt/slam.h>
#include <mrpt/base.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


/* ------------------------------------------------------------------------
					Test: STL containers serialize
   ------------------------------------------------------------------------ */
void Test_STL_containers_serialize()
{
	map<uint32_t, CPose2D>  m2, m1;
	map<int16_t, CPose3D>   n2, n1;

	m1[2] = CPose2D(1,2,0);
	m1[9] = CPose2D(-2,-3,1);

	{
		CFileOutputStream f("m1.bin");
		f << m1;
	}
	{
		CFileInputStream f("m1.bin");
		f >> m2;
	}

	ASSERT_(m1==m2)
	cout << "==== map<uint32_t, CPose2D>: Written & read OK" << endl;

	map< double, pair<CPose3DPDFGaussian, pair<TPoint3D, set<double> > > >	 big_var1,big_var2;
	map< double, pair<CPose3DPDFGaussian, set<double> > >	 big_var3;

	big_var1[0.4].first.mean = CPose3D(1,2,3);
	big_var1[0.4].first.cov.setIdentity();
	big_var1[0.8].first.mean = -big_var1[0.4].first.mean;
	big_var1[0.8].second.first= TPoint3D(1,2,3);
	big_var1[0.8].second.second.insert( 5 );

	{
		CFileOutputStream f("m2.bin");
		f << big_var1;
	}
	{
		CFileInputStream f("m2.bin");
		f >> big_var2;
	}

	// Trying to read to a different variable raises an exception:
	try
	{
		cout << endl << " *** An exception will be raised now intentionally *** " << endl;
		CFileInputStream f("m2.bin");
		f >> big_var3;
	}
	catch(std::exception &e)
	{
		cout << "The exception is: " << endl << e.what()
			<< endl << " *** End of intentional exception *** " << endl << endl;
	}

	ASSERT_( big_var1[0.4].first.mean == big_var2[0.4].first.mean )
	ASSERT_( big_var1 == big_var2 )
	cout << "==== map< double, pair<CPose3DPDFGaussian, TPoint3D> >: Written & read OK" << endl;


	vector_double v2, v1;
	v1 = make_vector<5,double>(1.0,2.0,3.0,4.0,5.0);

	{
		CFileOutputStream f("v1.bin");
		f << v1;
	}
	{
		CFileInputStream f("v1.bin");
		f >> v2;
	}

	ASSERT_(v1==v2)
	cout << "==== vector_double: Written & read OK" << endl;



}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		Test_STL_containers_serialize();

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
