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
#ifndef CMetricMapBuilder_H
#define CMetricMapBuilder_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CListOfClasses.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/synch.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/slam/CActionCollection.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	/** @defgroup metric_slam_grp Metric SLAM algorithms
	  * \ingroup mrpt_slam_grp */

	/** This virtual class is the base for SLAM implementations. See derived classes for more information.
	 *
	 * \sa CMetricMap  \ingroup metric_slam_grp
	 */
	class SLAM_IMPEXP CMetricMapBuilder : public mrpt::utils::CDebugOutputCapable
	{
	protected:
		/** Critical zones
		  */
		synch::CCriticalSection	critZoneChangingMap;

		/** Enter critical section for map updating:
		  */
		inline void  enterCriticalSection() { critZoneChangingMap.enter(); }

		/** Leave critical section for map updating:
		  */
		inline void  leaveCriticalSection() { critZoneChangingMap.leave(); }

	public:
		CMetricMapBuilder();           //!< Constructor
		virtual ~CMetricMapBuilder( ); //!< Destructor.

		// ---------------------------------------------------------------------
		/** @name Pure virtual methods to implement in any particular SLAM algorithm
		    @{ */

		/** Initialize the method, starting with a known location PDF "x0"(if supplied, set to NULL to left unmodified) and a given fixed, past map. */
		virtual void  initialize(
				const CSimpleMap &initialMap = CSimpleMap(),
				CPosePDF					*x0 = NULL
				) = 0;

		/** Returns a copy of the current best pose estimation as a pose PDF. */
		virtual CPose3DPDFPtr  getCurrentPoseEstimation() const = 0;

		/** Process a new action and observations pair to update this map: See the description of the class at the top of this page to see a more complete description.
		 *  \param action The estimation of the incremental pose change in the robot pose.
		 *	\param observations The set of observations that robot senses at the new pose.
		 */
		virtual void  processActionObservation( CActionCollection &action,CSensoryFrame	&observations ) = 0;

		/** Fills "out_map" with the set of "poses"-"sensory-frames", thus the so far built map. */
		virtual void  getCurrentlyBuiltMap(CSimpleMap &out_map) const = 0;

		/** Returns just how many sensory-frames are stored in the currently build map. */
		virtual unsigned int  getCurrentlyBuiltMapSize() = 0;

		/** Returns the map built so far. NOTE that for efficiency a pointer to the internal object is passed, DO NOT delete nor modify the object in any way, if desired, make a copy of ir with "duplicate()". */
		virtual CMultiMetricMap*   getCurrentlyBuiltMetricMap() = 0;

		/** A useful method for debugging: the current map (and/or poses) estimation is dumped to an image file.
		  * \param file The output file name
		  * \param formatEMF_BMP Output format = true:EMF, false:BMP
		  */
		virtual void  saveCurrentEstimationToImage(const std::string &file, bool formatEMF_BMP = true) = 0;

		/** @} */ 
		// ---------------------------------------------------------------------

		/** Clear all elements of the maps, and reset localization to (0,0,0deg). */
		void  clear();

		/** Enables or disables the map updating (default state is enabled) */
		void  enableMapUpdating( bool enable )
		{
			options.enableMapUpdating = enable;
		}

		/** Load map (mrpt::slam::CSimpleMap) from a ".simplemap" file */
		void  loadCurrentMapFromFile(const std::string &fileName);

		/** Save map (mrpt::slam::CSimpleMap) to a ".simplemap" file. */
		void  saveCurrentMapToFile(const std::string &fileName, bool compressGZ=true) const;

		/** Options for the algorithm */
		struct SLAM_IMPEXP TOptions
		{
			TOptions() : verbose(true),
						 enableMapUpdating(true),
						 debugForceInsertion(false),
						 alwaysInsertByClass()
			{
			}

			bool	verbose;             //!< If true shows debug information in the console, default is true.
			bool	enableMapUpdating;   //!< Enable map updating, default is true.
			bool	debugForceInsertion; //!< Always insert into map. Default is false: detect if necesary.

			/** A list of observation classes (derived from mrpt::slam::CObservation) which will be always inserted in the map, disregarding the minimum insertion distances).
			  *  Default: Empty. How to insert classes:
			  *   \code
			  *     alwaysInserByClass.insert(CLASS_ID(CObservationImage));
			  *   \endcode
			  * \sa mrpt::utils::CListOfClasses
			  */
			mrpt::utils::CListOfClasses		alwaysInsertByClass;

		};

		TOptions options;

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
