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

#include <mrpt/hmtslam.h> // Precomp header

#ifdef HTMSLAM_HAS_FABMAP
#  include <FabMapLibInterface.h>
#  define THE_FABMAP   static_cast<fabmap::FabMapInstance*>(m_fabmap)
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::hmtslam;
using namespace std;

CTopLCDetector_FabMap::CTopLCDetector_FabMap(CHMTSLAM *hmtslam) : 
	CTopLCDetectorBase(hmtslam), 
	m_fabmap(NULL)
{
#ifdef HTMSLAM_HAS_FABMAP
	// Use already loaded options:
	const CTopLCDetector_FabMap::TOptions *o = &m_hmtslam->m_options.TLC_fabmap_options;

    unsigned int nVocabSize;
	if (!fabmap::ParseOXV_PeekDimensions(o->vocab_path + o->vocabName + ".oxv",nVocabSize))
		THROW_EXCEPTION_CUSTOM_MSG1("Error parsing vocabulary file: %s", std::string(o->vocab_path + o->vocabName + ".oxv").c_str() )

	m_fabmap = new fabmap::FabMapInstance( o->vocab_path,o->vocabName, o->p_obs_given_exists,0.1, o->p_at_new_place, nVocabSize, o->df_lik_smooth );

	// Restart msg:
	cout << "[CTopLCDetector_FabMap::constructor] Resetting FabMap" << endl;
	THE_FABMAP->hmtslam_restart();
#else
	THROW_EXCEPTION("Please, recompile MRPT with FabMap to use this class.")
#endif
}

CTopLCDetector_FabMap::~CTopLCDetector_FabMap()
{
#ifdef HTMSLAM_HAS_FABMAP
	delete THE_FABMAP;
	m_fabmap = NULL;
#endif
}

/** This method must compute the topological observation model.
  * \param out_log_lik The output, a log-likelihood.
  * \return NULL, or a PDF of the estimated translation between the two areas (can be a multi-modal PDF).
  */
CPose3DPDFPtr CTopLCDetector_FabMap::computeTopologicalObservationModel(
	const THypothesisID		&hypID,
	const CHMHMapNodePtr	&currentArea,
	const CHMHMapNodePtr	&refArea,
	double					&out_log_lik
	)
{
	return CPose3DPDFPtr();
}

/** Hook method for being warned about the insertion of a new poses into the maps.
  *  This should be independent of hypothesis IDs.
  */
void CTopLCDetector_FabMap::OnNewPose(
	const TPoseID 			&poseID,
	const CSensoryFrame		*SF )
{
#ifdef HTMSLAM_HAS_FABMAP

	vector<string> lstObsImages;

	size_t n = 0;
	CObservationImagePtr obsIm;
	while ( (obsIm = SF->getObservationByClass<CObservationImage>(n)).present() )
	{
		string path;
		obsIm->image.getExternalStorageFileAbsolutePath(path);
		lstObsImages.push_back(path);
		n++;
	};

	if (lstObsImages.empty())  return; // Not all poses must have images.

	cout << "[OnNewPose] Adding new pose: " << poseID << " # of images: " << lstObsImages.size() <<  endl;
	THE_FABMAP->hmtslam_addNewPose(poseID,lstObsImages);

#endif
}




// Initialization
CTopLCDetector_FabMap::TOptions::TOptions() :
	vocab_path("./vocab"),
	vocabName("vocab_name"),
	p_obs_given_exists(0.39), 
	p_at_new_place(0.99), 
	df_lik_smooth(0.99)
{
}

//  Load parameters from configuration source
void  CTopLCDetector_FabMap::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&iniFile,
	const std::string		&section)
{
	MRPT_LOAD_CONFIG_VAR(vocab_path,string,  			iniFile, section );
	MRPT_LOAD_CONFIG_VAR(vocabName,string,  			iniFile, section );
	MRPT_LOAD_CONFIG_VAR(p_obs_given_exists,double,  			iniFile, section );
	MRPT_LOAD_CONFIG_VAR(p_at_new_place,double,  			iniFile, section );
	MRPT_LOAD_CONFIG_VAR(df_lik_smooth,double,  			iniFile, section );
}

//  This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
void CTopLCDetector_FabMap::TOptions::dumpToTextStream(CStream &out) const	{
	out.printf("\n----------- [CTopLCDetector_FabMap::TOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(vocab_path, string)
	LOADABLEOPTS_DUMP_VAR(vocabName, string)
	LOADABLEOPTS_DUMP_VAR(p_obs_given_exists, double)
	LOADABLEOPTS_DUMP_VAR(p_at_new_place, double)
	LOADABLEOPTS_DUMP_VAR(df_lik_smooth, double)
}
