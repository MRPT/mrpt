/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/stl_serialization.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/utils.h> // make_vector

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
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


	std::vector<double> v2, v1;
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
	cout << "==== CVectorDouble: Written & read OK" << endl;



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
