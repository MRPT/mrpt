/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
