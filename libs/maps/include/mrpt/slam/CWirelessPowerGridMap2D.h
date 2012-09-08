/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#ifndef CWirelessPowerGridMap2D_H
#define CWirelessPowerGridMap2D_H

#include <mrpt/slam/CRandomFieldGridMap2D.h>
#include <mrpt/slam/CObservationWirelessPower.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CWirelessPowerGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )


	/** CWirelessPowerGridMap2D represents a PDF of wifi concentrations over a 2D area.
	  *
	  *  There are a number of methods available to build the wifi grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor.
	  *
	  *  The following papers describe the mapping alternatives implemented here:
	  *		- mrKernelDM: A kernel-based method:
	  *		"Building gas concentration gridmaps with a mobile robot", Lilienthal, A. and Duckett, T., Robotics and Autonomous Systems, v.48, 2004.
	  *
	  *		- mrKernelDMV: A kernel-based method:
	  *		"A Statistical Approach to Gas Distribution Modelling with Mobile Robots--The Kernel DM+ V Algorithm"
	  * 	  , Lilienthal, A.J. and Reggente, M. and Trincavelli, M. and Blanco, J.L. and Gonzalez, J., IROS 2009.
	  *
	  * \sa mrpt::slam::CRandomFieldGridMap2D, mrpt::slam::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::slam::CMultiMetricMap
	  * \ingroup mrpt_maps_grp
	  */
	class MAPS_IMPEXP CWirelessPowerGridMap2D : public CRandomFieldGridMap2D
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CWirelessPowerGridMap2D )
	public:
		/** Constructor
		  */
		CWirelessPowerGridMap2D(
			TMapRepresentation	mapType = mrAchim,
            float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1
			);

		/** Destructor */
		virtual ~CWirelessPowerGridMap2D();


		/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
		 */
		 double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );


		/** Parameters related with inserting observations into the map:
		  */
		struct MAPS_IMPEXP TInsertionOptions :
			public utils::CLoadableOptions,
			public TInsertionOptionsCommon
		{
			TInsertionOptions();	//!< Default values loader

			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			void  dumpToTextStream(CStream	&out) const; //!< See utils::CLoadableOptions

		} insertionOptions;

		/** Returns an image just as described in \a saveAsBitmapFile */
		virtual void  getAsBitmapFile(mrpt::utils::CImage &out_img) const;

		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  */
		void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Returns a 3D object representing the map.
		  */
		void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

	protected:
		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() {
			return &insertionOptions;
		}

		 /** Erase all the contents of the map
		  */
		 virtual void  internal_clear();

		 /** Insert the observation information into this map. This method must be implemented
		  *    in derived classes.
		  * \param obs The observation
		  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
		  *
		  * \sa CObservation::insertObservationInto
		  */
		 virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );


	};


	} // End of namespace

} // End of namespace

#endif
