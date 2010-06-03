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
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

void TestMatrixIterators_copy()
{
	// Test of matrix iterators:
	{
		CMatrixDouble   M(2,3);
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		M = numbers;
		cout << "CMatrix (dynamic), std::copy ->  ";
		ostream_iterator<int> out_it (cout,", ");
		copy ( M.begin(), M.end(), out_it );
		cout << endl << "Complete matrix: " << endl << M << endl;
	}
	{
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		CMatrixFixedNumeric<double,2,3> M(numbers);
		cout << "CMatrix (fixed), std::copy ->  ";
		ostream_iterator<int> out_it (cout,", ");
		copy ( M.begin(), M.end(), out_it );
		cout << endl << "Complete matrix: " << endl << M << endl;
	}

	// Test of matrix rev. iterators:
	{
		CMatrixDouble   M(2,3);
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		M = numbers;
		cout << "CMatrix (dyn), std::copy with reverse iters ->  ";
		ostream_iterator<int> out_it (cout,", ");
		copy ( M.rbegin(), M.rend(), out_it );
		cout << endl;
	}
	{
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		CMatrixFixedNumeric<double,2,3> M(numbers);
		cout << "CMatrix (fixed), std::copy with reverse iters ->  ";
		ostream_iterator<int> out_it (cout,", ");
		copy ( M.rbegin(), M.rend(), out_it );
		cout << endl;
	}

	{
		CMatrixDouble   M1(2,3);
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		M1 = numbers;

		CMatrixDouble   M2(2,3);
		const double numbers2[] = {
			5,-1,4.5,
			0.3,-6.7,8.9 };
		M2 = numbers2;

		cout << "CMatrix (dynamic), std::inner_product ->  ";
		cout << inner_product(M1.begin(),M1.end(),M2.begin(),0.0) << " (should be 37.6)" << endl << endl;
	}

}

void TestMatrixIterators_misc()
{
	CMatrixDouble   M(2,3);
	const double numbers[] = {
		1,2,3,
		4,5,6 };
	M = numbers;

	// Test copy:
	{
		CMatrixDouble::iterator 		it1;  // test default constr.
		CMatrixDouble::iterator  it3 = M.begin();
		CMatrixDouble::iterator  it4 = M.end();
		cout << "eq? (no) " << (it1==it3) << endl;
		it1=it3;
		cout << "eq? (yes) " << (it1==it3) << endl;
		it1=it4;
		cout << "eq? (yes) " << (it1==it4) << endl;
	}

	// Test copy of const.
	{
		const CMatrixDouble MM = M;
		CMatrixDouble::const_iterator  it3 = MM.begin();
		CMatrixDouble::const_iterator  it4 = MM.end();
		{
			CMatrixDouble::const_iterator 		it1;  // test default constr.
			cout << "eq? (no) " << (it1==it3) << endl;
		}
		{
			CMatrixDouble::const_iterator 		it1 = it3;
			cout << "eq? (yes) " << (it1==it3) << endl;
		}
		{
			CMatrixDouble::const_iterator 		it1 = it4;
			cout << "eq? (yes) " << (it1==it4) << endl;
		}
	}

	// Test operator --
	{
		CMatrixDouble   M(2,3);
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		M = numbers;
		cout << "(dyn matrix) Test of --it; operator -> ";

		CMatrixDouble::iterator it = M.end();
		do
		{
			cout << *(--it)<< ", ";
		} while (it!=M.begin());
		cout << endl;
	}
	{
		const double numbers[] = {
			1,2,3,
			4,5,6 };
		CMatrixFixedNumeric<double,2,3> M(numbers);
		cout << "(fix matrix) Test of --it; operator -> ";

		CMatrixFixedNumeric<double,2,3>::iterator it = M.end();
		do
		{
			cout << *(--it)<< ", ";
		} while (it!=M.begin());
		cout << endl;
	}
}

void TestMatrixIterators_incrOps()
{
	CMatrixDouble   M(2,3);
	const double numbers[] = {
		1,2,3,
		4,5,6 };
	M = numbers;

	{
		cout << "Increment operator tests:"<<endl;
		CMatrixDouble::iterator i = M.end();
		cout << *(--i) << " ";
		cout << *(i-=3) << " ";
		cout << *(i+=1) << " ";
		cout << *(i-=2) << " ";
		i+=4;
		cout << *i << " ";
		i++;
		cout << "should be end(): (1=ok) " << (i==M.end()) << endl;
		cout << endl;
	}

	{
		const CMatrixDouble A=M;
		cout << "Increment const operator tests:"<<endl;
		CMatrixDouble::const_iterator i = A.end();
		cout << *(--i) << " ";
		cout << *(i-=3) << " ";
		cout << *(i+=1) << " ";
		cout << *(i-=2) << " ";
		i+=4;
		cout << *i << " ";
		i++;
		cout << "should be end(): (1=ok) " << (i==A.end()) << endl;
		cout << endl;
	}



}


void testContainerOps()
{
	CMatrixDouble x(10,10);
	vector_float  y(100,3.4f);
	CTicTac t;

	t.Tic();
	for (int i=0;i<10000;i++)
	{
		x+=y;
	}
	cout << mrpt::system::unitsFormat(t.Tac()/10000) << endl;
	cout << x << endl;

	// sum:
	double Dumm1 = sum(x);
	double Dumm2 = sum(y);
	cumsum(x,y);
	CMatrixFixedNumeric<float,10,10> Z;
	cumsum(x,Z);
	cumsum(Z,x);
	CMatrixFixedNumeric<float,10,10> Z2 = cumsum(Z);
	CMatrixDouble X2 = cumsum(x);
	vector_float  Y2 = cumsum(y);

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestMatrixIterators_copy();
		TestMatrixIterators_misc();
		TestMatrixIterators_incrOps();
		testContainerOps();

		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}




