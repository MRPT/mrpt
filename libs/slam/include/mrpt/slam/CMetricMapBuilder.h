/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMetricMapBuilder_H
#define CMetricMapBuilder_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CListOfClasses.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/obs/CActionCollection.h>

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
	class SLAM_IMPEXP CMetricMapBuilder : public mrpt::utils::COutputLogger
	{
	protected:
		mrpt::synch::CCriticalSection   critZoneChangingMap; //!< Critical zones
		/** Enter critical section for map updating */
		inline void  enterCriticalSection() { critZoneChangingMap.enter(); }
		/** Leave critical section for map updating */
		inline void  leaveCriticalSection() { critZoneChangingMap.leave(); }
	public:
		CMetricMapBuilder();           //!< Constructor
		virtual ~CMetricMapBuilder( ); //!< Destructor.

		// ---------------------------------------------------------------------
		/** @name Pure virtual methods to implement in any particular SLAM algorithm
		    @{ */

		/** Initialize the method, starting with a known location PDF "x0"(if supplied, set to NULL to left unmodified) and a given fixed, past map. */
		virtual void  initialize(
			const mrpt::maps::CSimpleMap &initialMap = mrpt::maps::CSimpleMap(),
			mrpt::poses::CPosePDF *x0 = NULL
			) = 0;

		/** Returns a copy of the current best pose estimation as a pose PDF. */
		virtual mrpt::poses::CPose3DPDFPtr  getCurrentPoseEstimation() const = 0;

		/** Process a new action and observations pair to update this map: See the description of the class at the top of this page to see a more complete description.
		 *  \param action The estimation of the incremental pose change in the robot pose.
		 *	\param observations The set of observations that robot senses at the new pose.
		 */
		virtual void  processActionObservation( mrpt::obs::CActionCollection &action,mrpt::obs::CSensoryFrame	&observations ) = 0;

		/** Fills "out_map" with the set of "poses"-"sensory-frames", thus the so far built map. */
		virtual void  getCurrentlyBuiltMap(mrpt::maps::CSimpleMap &out_map) const = 0;

		/** Returns just how many sensory-frames are stored in the currently build map. */
		virtual unsigned int  getCurrentlyBuiltMapSize() = 0;

		/** Returns the map built so far. NOTE that for efficiency a pointer to the internal object is passed, DO NOT delete nor modify the object in any way, if desired, make a copy of ir with "duplicate()". */
		virtual const mrpt::maps::CMultiMetricMap*   getCurrentlyBuiltMetricMap() const = 0;

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

		/** Load map (mrpt::maps::CSimpleMap) from a ".simplemap" file */
		void  loadCurrentMapFromFile(const std::string &fileName);

		/** Save map (mrpt::maps::CSimpleMap) to a ".simplemap" file. */
		void  saveCurrentMapToFile(const std::string &fileName, bool compressGZ=true) const;

		/** Options for the algorithm */
		struct SLAM_IMPEXP TOptions
		{
			TOptions(mrpt::utils::VerbosityLevel &verb_level_ref ) :
				verbosity_level (verb_level_ref),
				enableMapUpdating(true),
				debugForceInsertion(false),
				alwaysInsertByClass()
			{
			}

			mrpt::utils::VerbosityLevel & verbosity_level;
			bool	enableMapUpdating;   //!< Enable map updating, default is true.
			bool	debugForceInsertion; //!< Always insert into map. Default is false: detect if necesary.

			/** A list of observation classes (derived from mrpt::obs::CObservation) which will be always inserted in the map, disregarding the minimum insertion distances).
			  *  Default: Empty. How to insert classes:
			  *   \code
			  *     alwaysInserByClass.insert(CLASS_ID(CObservationImage));
			  *   \endcode
			  * \sa mrpt::utils::CListOfClasses
			  */
			mrpt::utils::CListOfClasses		alwaysInsertByClass;
		};

		TOptions options;

		public: 
			MRPT_MAKE_ALIGNED_OPERATOR_NEW 
	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
