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
#ifndef _CTopLCDetector_FabMap_H
#define _CTopLCDetector_FabMap_H

#include <mrpt/hmtslam/CTopLCDetectorBase.h>

namespace mrpt
{
	namespace hmtslam
	{
		/** \ingroup mrpt_hmtslam_grp */
		class CTopLCDetector_FabMap : public CTopLCDetectorBase
		{
		protected:
			CTopLCDetector_FabMap( CHMTSLAM *hmtslam );

			void *m_fabmap;		// FabMapInstance*

		public:
			/** A class factory, to be implemented in derived classes.
			  */
			static CTopLCDetectorBase* createNewInstance( CHMTSLAM *hmtslam )
			{
				return static_cast<CTopLCDetectorBase*>(new CTopLCDetector_FabMap(hmtslam));
			}

			/** Destructor */
			virtual ~CTopLCDetector_FabMap();

			/** This method must compute the topological observation model.
			  * \param out_log_lik The output, a log-likelihood.
			  * \return NULL (empty smart pointer), or a PDF of the estimated translation between the two areas (can be a multi-modal PDF).
			  */
			CPose3DPDFPtr computeTopologicalObservationModel(
				const THypothesisID		&hypID,
				const CHMHMapNodePtr	&currentArea,
				const CHMHMapNodePtr	&refArea,
				double					&out_log_lik
				 );

			/** Hook method for being warned about the insertion of a new poses into the maps.
			  *  This should be independent of hypothesis IDs.
			  */
			void OnNewPose(
				const TPoseID 			&poseID,
				const CSensoryFrame		*SF );


			/** Options for a TLC-detector of type FabMap, used from CHMTSLAM 
			  */
			struct TOptions : public utils::CLoadableOptions
			{
				/** Initialization of default params
				  */
				TOptions();

				/** Load parameters from configuration source
				  */
				void  loadFromConfigFile(
					const mrpt::utils::CConfigFileBase	&source,
					const std::string		&section);

				/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
				  */
				void  dumpToTextStream(CStream	&out) const;

				std::string		vocab_path,vocabName;
				double			p_obs_given_exists, p_at_new_place, df_lik_smooth;
			};


		}; // end class
	} // end namespace
} // end namespace


#endif
