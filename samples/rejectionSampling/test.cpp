/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CTicTac.h>
#include <mrpt/bayes.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/random.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::system;

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
