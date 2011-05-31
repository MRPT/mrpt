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
