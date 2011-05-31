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

#include <mrpt/mrpt.h>
using namespace UTILS;
using namespace MRML;
using namespace std;

// ------------------------------------------------------
//				TestMatchingCovariance
// ------------------------------------------------------
void TestMatchingCovariance()
{
	CTicTac							tictac;
	CSimplePointsMap				m1,m2,m3;
	CPoint3D						p;
	CMatrix							cov,Z,M_psdInv,dataScans,dataOdo, aux;
	float							ecm,corrsRatio;
	CMetricMap::TMatchingPairList	corrs;

	m1.load2D_from_text_file("map1.txt"); CFileStream("map1.bin",fomWrite) << m1;
	m1.load2D_from_text_file("map2.txt"); CFileStream("map2.bin",fomWrite) << m1;
	m1.load2D_from_text_file("map3.txt"); CFileStream("map3.bin",fomWrite) << m1;
	m1.load2D_from_text_file("map4.txt"); CFileStream("map4.bin",fomWrite) << m1;
	m1.load2D_from_text_file("map5.txt"); CFileStream("map5.bin",fomWrite) << m1;

	// Load the point maps to align
	CFileStream("map1.bin",fomRead) >> m1;
	CFileStream("map4.bin",fomRead) >> m2;

	tictac.Tic();

	for (int j=0;j<100;j++)
	{
		CPose2D 	otherMapPose(0,0,0);
		CPose2D		dumm(0,0,0);
		m1.computeMatchingWith2D( 
			&m2,
			otherMapPose,
			0.25f /*Max dist. for corr.*/,
			DEG2RAD(5.0f),
			dumm,
			corrs,
			corrsRatio,
			&ecm,
			&cov,
			true);
	}

	printf("In %fms:\nCorrs ratio=%f\necm=%f\n",tictac.Tac()*10,corrsRatio,sqrt(ecm/ (corrs.size() ? corrs.size():1) ));

}


int main()
{
	try
	{
		TestMatchingCovariance();

		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception catched: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}

