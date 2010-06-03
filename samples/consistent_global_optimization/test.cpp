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
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;



// ------------------------------------------------------
//				TestGlobalOptimization
// ------------------------------------------------------
void TestGlobalOptimization()
{
	CConsistentObservationAlignment					CA;
	CMatrixTemplateObjects<CPosePDFGaussian>		inData;
	CMatrixTemplateObjects<CPosePDFGaussian>		outPoses;

	// COV:
	CMatrix		stdCov(3,3),bigCov(3,3);
	stdCov.unit(); stdCov *= 1e-4f;
	bigCov.unit(); bigCov *= 1e+6f;

	// Data:
	inData.setSize(4,4);
	inData.allocAllObjects();

	inData(0,1)->cov = stdCov;	inData(0,1)->mean = CPose2D( 1, 0, DEG2RAD(70) ); inData(0,1)->cov*= 2;
	inData(0,2)->cov = stdCov;	inData(0,2)->mean = CPose2D( 0, 1, 0 );	
	inData(0,3)->cov = bigCov;

	inData(1,2)->cov = stdCov;  inData(1,2)->mean = CPose2D( -1, 1, DEG2RAD(-90) );	
	inData(1,3)->cov = stdCov;	inData(1,3)->mean = CPose2D( 0, 1, DEG2RAD(-90) );	//CPose2D( 0, -1, 0 );	

	inData(2,3)->cov = stdCov;	inData(2,3)->mean = CPose2D( 1, 0, 0 );	


	// Optimize:
	CA.optimizeUserSuppliedData( inData,outPoses );

	// Output:
	cout << "OPTIMIZED POSES:" << endl;
	cout << "[0]: "<< outPoses(0,0)->mean << endl;
	cout << "[1]: "<< outPoses(0,1)->mean << endl;
	cout << "[2]: "<< outPoses(0,2)->mean << endl;
	cout << "[3]: "<< outPoses(0,3)->mean << endl;

}

int main()
{
	try
	{
		TestGlobalOptimization();

		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}

