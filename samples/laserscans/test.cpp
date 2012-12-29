/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

