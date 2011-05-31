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

#include <mrpt/utils.h>
#include <mrpt/slam.h>
#include <mrpt/bayes.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::bayes;
using namespace mrpt::slam;
using namespace mrpt::random;

double SIGMA = 0.05;

// The custom class:
class CMyRejectionSampling: public CRejectionSamplingCapable<CPose2D>
{
protected:
	void RS_drawFromProposal( CPose2D &outSample )
	{
		double ang = randomGenerator.drawUniform(-M_PI,M_PI);
		double R   = randomGenerator.drawGaussian1D(1.0,SIGMA);
		outSample.x( 1.0f - cos(ang) * R );
		outSample.y( sin(ang) * R );
		outSample.phi( randomGenerator.drawUniform(-M_PI,M_PI) );
	}

	/** Returns the NORMALIZED observation likelihood at a given point of the state space (values in the range [0,1]).
	  */
	double RS_observationLikelihood( const CPose2D &x)
	{
		return exp( -0.5*square((x.distanceTo(CPoint2D(0,0))-1.0f)/SIGMA) );
	}
};

// ------------------------------------------------------
//				TestRS
// ------------------------------------------------------
void TestRS()
{
	CMyRejectionSampling							RS;
	std::vector<CMyRejectionSampling::TParticle>	samples;
	CTicTac		tictac;

	tictac.Tic();
	printf("Computing...");

	RS.rejectionSampling( 1000,samples );

	printf("Ok! %fms\n",1000*tictac.Tac());

	FILE	*f = os::fopen( "_out_samples.txt","wt");
	std::vector<CMyRejectionSampling::TParticle>::iterator	it;
	for (it=samples.begin();it!=samples.end();it++)
		os::fprintf(f,"%f %f %f %e\n",it->d->x(),it->d->y(),it->d->phi(),it->log_w );


	os::fclose(f);
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		 TestRS();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "EXCEPCTION: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped excepcion!!");
		return -1;
	}
}
