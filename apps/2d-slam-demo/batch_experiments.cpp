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

#include "slamdemoApp.h"

#include "slamdemoMain.h"
#include <wx/image.h>
#include <wx/msgdlg.h>


#include <mrpt/base.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::system;

// Do batch experiments as indicated by the command line arguments
//  and the command file "cfgFil".
void slamdemoApp::DoBatchExperiments(const std::string &cfgFil)
{
	CConfigFile  cf(cfgFil);

	win->options.loadFromConfigFile(cf,"");
	win->m_SLAM.options.loadFromConfigFile(cf,"");
	win->m_SLAM.KF_options.loadFromConfigFile(cf,"");

	win->resetSimulator(win->options.map_generator);

	const size_t nSteps = round(6*win->options.path_square_len/double(win->options.robot_step_length));

	CTicTac   tim;
	tim.Tic();

	if (!m_option_norun)
	for (size_t i=0;i<nSteps;i++)
		win->executeOneStep();

	double mT = tim.Tac();

	// Stats:
	const size_t N = win->m_historicData.size();
	double totalFP=0;
	double totalFN=0;
	double totalJCBBiters=0;
	double err_xy=0;
	double err_phi=0;
	double err_D2=0;


	for (size_t i=0;i<N;i++)
	{
		totalFP+=win->m_historicData[i].da_false_pos;
		totalFN+=win->m_historicData[i].da_false_neg;
		totalJCBBiters+=win->m_historicData[i].jcbb_iters;
		err_xy+=win->m_historicData[i].GT_robot_pose.distanceTo(win->m_historicData[i].estimate_robot_pose.getMeanVal());

		CPosePDFGaussian p( win->m_historicData[i].GT_robot_pose, CMatrixDouble33()/* cov= zero */ );
		double dm2=1000;
		try
		{
			dm2=win->m_historicData[i].estimate_robot_pose.mahalanobisDistanceTo(p);
		}
		catch(...) { }
		err_D2+=dm2;
		err_phi+=fabs( wrapToPi( win->m_historicData[i].GT_robot_pose.phi() - win->m_historicData[i].estimate_robot_pose.getMeanVal().phi() ) );
	}


	if (N) {
		totalFP/=N;
		totalFN/=N;
		totalJCBBiters/=N;
		err_xy/=N;
		err_phi/=N;
		err_D2/=N;
		mT/=N;
	}

	cout << "FP: " << totalFP <<
			" FN: " << totalFN <<
			" JCBB iters: " << totalJCBBiters << " Run iters: " << N << endl;
	cout << "Mean err_xy: " << err_xy
		 << " mean tim: " << mT << endl;

	ofstream  f("slam_out.txt", ios::out | ios::app );

	f 	<< mT << " "
		<< err_xy << " "
		<< err_phi <<  " "
		<< err_D2 <<  " "
		<< totalJCBBiters <<  " "
		<< totalFP <<  " "
		<< totalFN <<  " "
		<< endl;

	win->updateAllGraphs();
}
