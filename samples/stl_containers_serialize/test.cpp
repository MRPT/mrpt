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
