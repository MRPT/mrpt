/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CSENSORYFRAME_H
#define CSENSORYFRAME_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/link_pragmas.h>

namespace mrpt
{
	namespace obs
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSensoryFrame, mrpt::utils::CSerializable, OBS_IMPEXP )

		/** Declares a class for storing a "sensory frame", a set of "observations" taken by the robot approximately at the same time as one "snapshot" of the environment.
		  * It can contain "observations" of many different kinds.
		  *
		  *  New observations can be added using:
		  *
		  * \code
		  * CObservationXXXPtr	o = CObservationXXX::Create();  // Create a smart pointer containing an object of class "CObservationXXX"
		  * o->(...)
		  *
		  * CSensoryFrame	 sf;
		  * sf.insert(o);
		  * \endcode
		  *
		  * The following methods are equivalent for adding new observations to a "sensory frame":
		  * - CSensoryFrame::operator +=
		  * - CSensoryFrame::push_back
		  * - CSensoryFrame::insert
		  *
		  * To examine the objects within a sensory frame, the following methods exist:
		  * - CSensoryFrame::getObservationByClass : Looks for some specific observation class.
		  * - CSensoryFrame::begin : To iterate over all observations.
		  * - CSensoryFrame::getObservationByIndex : To query by index.
		  *
		  * Notice that contained observations objects are automatically deleted on
		  *  this object's destruction or clear.
		  * \sa CObservation
	 	  * \ingroup mrpt_obs_grp
		  */
		class OBS_IMPEXP CSensoryFrame : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CSensoryFrame )

		public:
			 /** Default constructor
			  */
			 CSensoryFrame();

			 /** Copy constructor
			  */
			 CSensoryFrame( const CSensoryFrame &);

			/** @name Cached points map
				@{  */
		protected:
			/** A points map, build only under demand by the methods getAuxPointsMap() and buildAuxPointsMap().
			  *  It's a generic smart pointer to avoid depending here in the library mrpt-obs on classes on other libraries.
			  */
			mutable mrpt::maps::CMetricMapPtr  m_cachedMap;

			void internal_buildAuxPointsMap( const void *options = NULL ) const;  //!< Internal method, used from buildAuxPointsMap()

		public:

			/** Returns the cached points map representation of the scan, if already build with buildAuxPointsMap(), or NULL otherwise.
			  * Usage:
			  *  \code
			  *    mrpt::maps::CPointsMap *map = obs->getAuxPointsMap<mrpt::maps::CPointsMap>();
			  *  \endcode
			  * \sa buildAuxPointsMap
			  */
			template <class POINTSMAP>
			inline const POINTSMAP* getAuxPointsMap() const {
				return static_cast<POINTSMAP*>(m_cachedMap.pointer());
			}

			/** Returns a cached points map representing this laser scan, building it upon the first call.
			  * \param options Can be NULL to use default point maps' insertion options, or a pointer to a "CPointsMap::TInsertionOptions" structure to override some params.
			  * Usage:
			  *  \code
			  *    mrpt::maps::CPointsMap *map = sf->buildAuxPointsMap<mrpt::maps::CPointsMap>(&options or NULL);
			  *  \endcode
			  * \sa getAuxPointsMap
			  */
			template <class POINTSMAP>
			inline const POINTSMAP	*buildAuxPointsMap( const void *options = NULL ) const {
				internal_buildAuxPointsMap(options);
				return static_cast<POINTSMAP*>(m_cachedMap.pointer());
			}

			/** @} */


			 /** Copy
			   */
			 CSensoryFrame&  operator =( const CSensoryFrame &o);

			 /** Destructor.
			  */
			 virtual ~CSensoryFrame();

			 /** Clear all current observations.
			  */
			 void		clear();

			 /** Insert all the observations in this SF into a metric map or any kind (see mrpt::maps::CMetricMap).
			  *  It calls CObservation::insertObservationInto for all stored observation.
			  * \param theMap The map where this observation is to be inserted: the map will be updated.
			  * \param robotPose The pose of the robot base for this observation, relative to the target metric map. Set to NULL (default) to use (0,0,0deg)
			  *
			  * \return Returns true if the map has been updated, or false if this observations
			  *			has nothing to do with a metric map (for example, a sound observation).
			  *
			  * \sa mrpt::maps::CMetricMap, CObservation::insertObservationInto, CMetricMap::insertObservation
			  */
			 bool  insertObservationsInto( mrpt::maps::CMetricMap *theMap, const mrpt::poses::CPose3D *robotPose = NULL ) const;

			 /** Insert all the observations in this SF into a metric map or any kind (see mrpt::maps::CMetricMap).
			  *  It calls CObservation::insertObservationInto for all stored observation.
			  * \param theMap The map where this observation is to be inserted: the map will be updated.
			  * \param robotPose The pose of the robot base for this observation, relative to the target metric map. Set to NULL (default) to use (0,0,0deg)
			  *
			  * \return Returns true if the map has been updated, or false if this observations
			  *			has nothing to do with a metric map (for example, a sound observation).
			  *
			  * \sa mrpt::maps::CMetricMap, CObservation::insertObservationInto, CMetricMap::insertObservation
			  */
			 inline bool  insertObservationsInto( mrpt::maps::CMetricMapPtr &theMap, const mrpt::poses::CPose3D *robotPose = NULL ) const
			 {
				 return insertObservationsInto(theMap.pointer(), robotPose);
			 }


			 /** You can use "sf1+=sf2;" to add observations in sf2 to sf1. Objects are copied, not referenced, thus the source can be safely deleted next.
			   * \sa moveFrom
			  */
			 void operator += (const CSensoryFrame &sf);

			 /** You can use "sf+=obs;" to add the observation "obs" to the "sf1". Objects are copied, using the smart pointer, thus the original pointer can be safely deleted next.
			   * \sa moveFrom
			  */
			 void operator += (const CObservationPtr &obs);

			 /** Copies all the observation from another object, then erase them from the origin object (this method is fast since only pointers are copied); Previous objects in this objects are not deleted.
			   * \sa operator +=
			   */
			 void moveFrom(CSensoryFrame &sf);

			 /** Inserts a new observation to the list: The pointer to the objects is copied, thus DO NOT delete the passed object, this class will do at destructor or when appropriate.
			   */
			 void push_back(const CObservationPtr &obs);

			 /** Inserts a new observation to the list: The pointer to the objects is copied, thus DO NOT delete the passed object, this class will do at destructor or when appropriate.
			   */
			 void insert(const CObservationPtr &obs);

			 /** Returns the i'th observation of a given class (or of a descendant class), or NULL if there is no such observation in the array.
			   *  Example:
			   * \code
					CObservationImagePtr obs = m_SF->getObservationByClass<CObservationImage>();
			   * \endcode
			   * By default (ith=0), the first observation is returned.
			   */
			 template <typename T>
			 typename T::SmartPtr getObservationByClass( const size_t &ith = 0 ) const
			 {
				MRPT_START
				size_t  foundCount = 0;
				const mrpt::utils::TRuntimeClassId*	class_ID = T::classinfo;
				for (const_iterator it = begin();it!=end();++it)
					if ( (*it)->GetRuntimeClass()->derivedFrom( class_ID ) )
						if (foundCount++ == ith)
							return typename T::SmartPtr(*it);
				return typename T::SmartPtr();	// Not found: return empty smart pointer
				MRPT_END
			 }

			/** You can use CSensoryFrame::begin to get a iterator to the first element.
			  */
			typedef std::deque<CObservationPtr>::iterator		iterator;

			/** You can use CSensoryFrame::begin to get a iterator to the first element.
			  */
			typedef std::deque<CObservationPtr>::const_iterator	const_iterator;

			/** Returns a constant iterator to the first observation: this is an example of usage:
			  * \code
			  *   CSensoryFrame  sf;
			  *   ...
			  *   for (CSensoryFrame::const_iterator it=sf.begin();it!=sf.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CObservation*"
			  *   }
			  *
			  * \endcode
			  */
			const_iterator begin() const { return m_observations.begin(); }

			/** Returns a constant iterator to the end of the list of observations: this is an example of usage:
			  * \code
			  *   CSensoryFrame  sf;
			  *   ...
			  *   for (CSensoryFrame::const_iterator it=sf.begin();it!=sf.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CObservation*"
			  *   }
			  *
			  * \endcode
			  */
			const_iterator end() const { return m_observations.end(); }

			/** Returns a iterator to the first observation: this is an example of usage:
			  * \code
			  *   CSensoryFrame  sf;
			  *   ...
			  *   for (CSensoryFrame::iterator it=sf.begin();it!=sf.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CObservation*"
			  *   }
			  *
			  * \endcode
			  */
			iterator begin() { return m_observations.begin(); }

			/** Returns a iterator to the end of the list of observations: this is an example of usage:
			  * \code
			  *   CSensoryFrame  sf;
			  *   ...
			  *   for (CSensoryFrame::iterator it=sf.begin();it!=sf.end();++it)
			  *	  {
			  *      (*it)->... // (*it) is a "CObservation*"
			  *   }
			  *
			  * \endcode
			  */
			inline iterator end() { return m_observations.end(); }


			/** Returns the number of observations in the list. */
			inline size_t size() const { return m_observations.size(); }

			/** Returns true if there are no observations in the list. */
			inline bool empty() const { return  m_observations.empty(); }

			/** Removes the i'th observation in the list (0=first). */
			void   eraseByIndex(const size_t &idx);

			/** Removes the given observation in the list, and return an iterator to the next element (or this->end() if it was the last one).
			  */
			iterator erase( const iterator &it);

			/** Removes all the observations that match a given sensorLabel.
			  */
			void   eraseByLabel(const std::string &label);

			/** Returns the i'th observation in the list (0=first).
			  * \sa begin, size
			  */
			CObservationPtr getObservationByIndex( const size_t &idx ) const;

			/** Returns the i'th observation in the list (0=first), and as a different smart pointer type:
			  * \code
			  *   sf.getObservationByIndexAs<CObservationStereoImagesPtr>(i);
			  * \endcode
			  * \sa begin, size
			  */
			template <typename T>
			T getObservationByIndexAs( const size_t &idx ) const
			{
				return static_cast<T>(getObservationByIndex(idx));
			}

			/** Returns the i'th observation in the list with the given "sensorLabel" (0=first).
			  * \return The observation, or NULL if not found.
			  * \sa begin, size
			  */
			CObservationPtr getObservationBySensorLabel( const std::string &label, const size_t &idx = 0) const;

			/** Returns the i'th observation in the list with the given "sensorLabel" (0=first), and as a different smart pointer type:
			  * \code
			  *   sf.getObservationBySensorLabelAs<CObservationStereoImagesPtr>(i);
			  * \endcode
			  * \sa begin, size
			  */
			template <typename T>
			T getObservationBySensorLabelAs( const std::string &label, const size_t &idx = 0) const
			{
				return T(getObservationBySensorLabel(label,idx));
			}

			/** Efficiently swaps the contents of two objects.
			  */
			void swap( CSensoryFrame &sf );

		protected:
			 /** The set of observations taken at the same time instant. See the top of this page for instructions on accessing this.
			  */
			 //std::deque<CObservation*>	m_observations;
			std::deque<CObservationPtr>	m_observations;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CSensoryFrame, mrpt::utils::CSerializable, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
