/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRawlog_H
#define CRawlog_H

#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/utils/CConfigFileMemory.h>


namespace mrpt
{
	namespace obs
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRawlog, mrpt::utils::CSerializable, OBS_IMPEXP )

		typedef std::pair<mrpt::system::TTimeStamp, CObservationPtr>		TTimeObservationPair;		//!< For usage with CRawlog classes.
		typedef std::multimap<mrpt::system::TTimeStamp, CObservationPtr>	TListTimeAndObservations;	//!< For usage with CRawlog classes.


		/** This class stores a rawlog (robotic datasets) in one of two possible formats:
		 *		- Format #1: A sequence of actions and observations. There is a sequence of objects, where each one can be of one type:
		 *			- An action:	Implemented as a CActionCollection object, the actuation of the robot (i.e. odometry increment).
		 *			- Observations: Implemented as a CSensoryFrame, refering to a set of robot observations from the same pose.
		 *		- Format #2: A sequence of actions and observations. There is a sequence of objects, where each one can be of one type:
		 *
		 *	Refer to the wiki page about <a href="http://www.mrpt.org/Rawlog_Format" >rawlog files</a>.
		 *
		 *  See also the application <a href="http://www.mrpt.org/Application:RawLogViewer" >RawLogViewer</a > for the GUI program that visualizes and manages rawlogs.
		 *
		 *  This class also publishes a static helper method for loading rawlog files in format #1: see CRawlog::readActionObservationPair
		 *
		 *  There is a field for comments and blocks of parameters (in ini-like format) accessible through getCommentText and setCommentText
		 *   (introduced in MRPT 0.6.4). When serialized to a rawlog file, the comments are saved as an additional observation of the
		 *   type CObservationComments at the beginning of the file, but this observation does not appear after loading for clarity.
		 *
		 * \note Since MRPT version 0.5.5, this class also provides a STL container-like interface (see CRawlog::begin, CRawlog::iterator, ...).
		 * \note The format #2 is supported since MRPT version 0.6.0.
		 * \note There is a static helper method "detectImagesDirectory" for localizing the external images directory of a rawlog.
		 *
		 * \sa CSensoryFrame, CPose2D, <a href="http://www.mrpt.org/Rawlog_Format"> RawLog file format</a>.
	 	 * \ingroup mrpt_obs_grp
		 */
		class OBS_IMPEXP CRawlog : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CRawlog )

		private:
			typedef	std::vector<mrpt::utils::CSerializablePtr> TListObjects;
			TListObjects	m_seqOfActObs;	//!< The list where the objects really are in.

			CObservationComment		m_commentTexts;	//!< Comments of the rawlog.

		public:
			void getCommentText( std::string &t) const;	//!< Returns the block of comment text for the rawlog
			std::string getCommentText() const;			//!< Returns the block of comment text for the rawlog
			void setCommentText( const std::string &t);	//!< Changes the block of comment text for the rawlog
			void getCommentTextAsConfigFile( mrpt::utils::CConfigFileMemory &memCfg ) const; //!< Saves the block of comment text for the rawlog into the passed config file object.

			/** The type of each entry in a rawlog.
			  * \sa CRawlog::getType
			  */
			enum TEntryType
			{
				etSensoryFrame = 0,
				etActionCollection,
				etObservation,
				etOther
			};

			/** Default constructor */
			CRawlog();


			/** Destructor: */
			virtual ~CRawlog();

			/** Clear the sequence of actions/observations. Smart pointers to objects previously in the rawlog will remain being valid. */
			void  clear();

			/** Add an action to the sequence: a collection of just one element is created.
			  *   The object is duplicated, so the original one can be freed if desired.
			  */
			void  addAction( CAction &action );

			/** Add a set of actions to the sequence; the object is duplicated, so the original one can be freed if desired.
			  * \sa addObservations, addActionsMemoryReference
			  */
			void  addActions( CActionCollection &action );

			/** Add a set of observations to the sequence; the object is duplicated, so the original one can be free if desired.
			  * \sa addActions, addObservationsMemoryReference
			  */
			void  addObservations( CSensoryFrame &observations );

			/** Add a set of actions to the sequence, using a smart pointer to the object to add.
			  * \sa addActions, addObservationsMemoryReference, addObservationMemoryReference
			  */
			void  addActionsMemoryReference( const CActionCollectionPtr &action );

			/** Add a set of observations to the sequence, using a smart pointer to the object to add.
			  * \sa addObservations, addActionsMemoryReference, addObservationMemoryReference
			  */
			void  addObservationsMemoryReference( const CSensoryFramePtr &observations );

			/** Add a single observation to the sequence, using a smart pointer to the object to add.
			  * \sa addObservations, addActionsMemoryReference
			  */
			void  addObservationMemoryReference( const CObservationPtr &observation );

			/** Generic add for a smart pointer to a CSerializable object:
			  * \sa addObservations, addActionsMemoryReference, addObservationMemoryReference
			  */
			void  addGenericObject( const mrpt::utils::CSerializablePtr &obj );

			/** Load the contents from a file containing one of these possibilities:
			  *  - A "CRawlog" object.
			  *  - Directly the sequence of objects (pairs `CSensoryFrame`/`CActionCollection` or `CObservation*` objects). In this case the method stops reading on EOF of an unrecogniced class name.
			  *  - Only if `non_obs_objects_are_legal` is true, any `CSerializable` object is allowed in the log file. Otherwise, the read stops on classes different from the ones listed in the item above.
			  * \returns It returns false upon error reading or accessing the file.
			  */
			bool  loadFromRawLogFile( const std::string &fileName, bool non_obs_objects_are_legal = false );

			/** Saves the contents to a rawlog-file, compatible with RawlogViewer (As the sequence of internal objects).
			  *  The file is saved with gz-commpressed if MRPT has gz-streams.
			  * \returns It returns false if any error is found while writing/creating the target file.
			  */
			bool saveToRawLogFile( const std::string &fileName ) const;

			/** Returns the number of actions / observations object in the sequence. */
			size_t  size() const;

			/** Returns the type of a given element.
			  * \sa isAction, isObservation
			  */
			TEntryType getType( size_t index ) const;

			/** Delete the action or observation stored in the given index.
			  * \exception std::exception If index is out of bounds
			  */
			void  remove( size_t index );

			/** Delete the elements stored in the given range of indices (including both the first and last one).
			  * \exception std::exception If any index is out of bounds
			  */
			void  remove( size_t first_index, size_t last_index );

			/** Returns the i'th element in the sequence, as being actions, where index=0 is the first object.
			  *  If it is not a CActionCollection, it throws an exception. Do neighter modify nor delete the returned pointer.
			  * \sa size, isAction, getAsObservations, getAsObservation
			  * \exception std::exception If index is out of bounds
			  */
			CActionCollectionPtr  getAsAction( size_t index ) const;

			/** Returns the i'th element in the sequence, as being an action, where index=0 is the first object.
			  *  If it is not an CSensoryFrame, it throws an exception. Do neighter modify nor delete the returned pointer.
			  * \sa size, isAction, getAsAction, getAsObservation
			  * \exception std::exception If index is out of bounds
			  */
			CSensoryFramePtr  getAsObservations( size_t index ) const;

			/** Returns the i'th element in the sequence, being its class whatever.
			  * \sa size, isAction, getAsAction, getAsObservations
			  * \exception std::exception If index is out of bounds
			  */
			mrpt::utils::CSerializablePtr    getAsGeneric( size_t index ) const;

			/** Returns the i'th element in the sequence, as being an observation, where index=0 is the first object.
			  *  If it is not an CObservation, it throws an exception. Do neighter modify nor delete the returned pointer.
			  *  This is the proper method to obtain the objects stored in a "only observations"-rawlog file (named "format #2" above.
			  * \sa size, isAction, getAsAction
			  * \exception std::exception If index is out of bounds
			  */
			CObservationPtr  getAsObservation( size_t index ) const;


			/** A normal iterator, plus the extra method "getType" to determine the type of each entry in the sequence. */
			class iterator
			{
			protected:
				TListObjects::iterator	m_it;

			public:
				iterator() : m_it() {  }
				iterator(const TListObjects::iterator& it) : m_it(it)  {  }
				virtual ~iterator() { }

				iterator & operator = (const iterator& o) {  m_it = o.m_it; return *this; }

				bool operator == (const iterator& o) {  return m_it == o.m_it; }
				bool operator != (const iterator& o) {  return m_it != o.m_it; }

				mrpt::utils::CSerializablePtr operator *() { return *m_it; }

				inline iterator  operator ++(int) { iterator aux =*this; m_it++; return aux; }  // Post
				inline iterator& operator ++()    { m_it++; return *this; }  // Pre
				inline iterator  operator --(int) { iterator aux = *this; m_it--; return aux; }  // Post
				inline iterator& operator --()    { m_it--; return *this; }  // Pre

				TEntryType getType() const
				{
					if ( (*m_it)->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
						return etObservation;
					else if ( (*m_it)->GetRuntimeClass()->derivedFrom( CLASS_ID(CSensoryFrame) ) )
						return etSensoryFrame;
					else
						return etActionCollection;
				}

				static iterator erase( TListObjects& lst, const iterator &it) { return lst.erase(it.m_it); }
			};

			/** A normal iterator, plus the extra method "getType" to determine the type of each entry in the sequence. */
			class const_iterator
			{
			protected:
				TListObjects::const_iterator	m_it;

			public:
				const_iterator() : m_it() {  }
				const_iterator(const TListObjects::const_iterator& it) : m_it(it)  {  }
				virtual ~const_iterator() { }

				bool operator == (const const_iterator& o) {  return m_it == o.m_it; }
				bool operator != (const const_iterator& o) {  return m_it != o.m_it; }

				const mrpt::utils::CSerializablePtr operator *() const { return *m_it; }

				inline const_iterator  operator ++(int) { const_iterator aux =*this; m_it++; return aux; }  // Post
				inline const_iterator& operator ++()    { m_it++; return *this; }  // Pre
				inline const_iterator  operator --(int) { const_iterator aux = *this; m_it--; return aux; }  // Post
				inline const_iterator& operator --()    { m_it--; return *this; }  // Pre

				TEntryType getType() const
				{
					if ( (*m_it)->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
						return etObservation;
					else if ( (*m_it)->GetRuntimeClass()->derivedFrom( CLASS_ID(CSensoryFrame) ) )
						return etSensoryFrame;
					else
						return etActionCollection;
				}

			};


			const_iterator begin() const { return m_seqOfActObs.begin(); }
			iterator begin() { return m_seqOfActObs.begin(); }
			const_iterator end() const { return m_seqOfActObs.end(); }
			iterator end() { return m_seqOfActObs.end(); }

			iterator erase(const iterator &it) { return iterator::erase(m_seqOfActObs, it); }

			/** Returns the sub-set of observations of a given class whose time-stamp t fulfills  time_start <= t < time_end.
			  *  This method requires the timestamps of the sensors to be in strict ascending order (which should be the normal situation).
			  *   Otherwise, the output is undeterminate.
			  * \sa findClosestObservationsByClass
			  */
			void findObservationsByClassInRange(
				mrpt::system::TTimeStamp		time_start,
				mrpt::system::TTimeStamp		time_end,
				const mrpt::utils::TRuntimeClassId	*class_type,
				TListTimeAndObservations		&out_found,
				size_t							guess_start_position = 0
				) const;

			/** Efficiently copy the contents from other existing object, and remove the data from the origin (after calling this, the original object will have no actions/observations).
			  */
			void moveFrom( CRawlog &obj);

			/** Efficiently swap the contents of two existing objects.
			  */
			void swap( CRawlog &obj);

			/** Reads a consecutive pair action / observation from the rawlog opened at some input stream.
			  *   Previous contents of action and observations are discarded (using stlplus::smart_ptr::clear_unique), and
			  *    at exit they contain the new objects read from the rawlog file.
			  *  The input/output variable "rawlogEntry" is just a counter of the last rawlog entry read, for logging or monitoring purposes.
			  * \return false if there was some error, true otherwise.
			  * \sa getActionObservationPair, getActionObservationPairOrObservation
			  */
			static bool readActionObservationPair(
				mrpt::utils::CStream					&inStream,
				CActionCollectionPtr	&action,
				CSensoryFramePtr		&observations,
				size_t			& rawlogEntry );

			/** Reads a consecutive pair action/sensory_frame OR an observation, depending of the rawlog format, from the rawlog opened at some input stream.
			  *   Previous contents of action and observations are discarded (using stlplus::smart_ptr::clear_unique), and
			  *    at exit they contain the new objects read from the rawlog file.
			  *
			  *  At return, one of this will happen:
			  *		- action/observations contain objects (i.e. action.present() evaluates as true).
			  *		- observation contains an object (i.e. observation.present() evaluates as true).
			  *
			  *  The input/output variable "rawlogEntry" is just a counter of the last rawlog entry read, for logging or monitoring purposes.
			  * \return false if there was some error, true otherwise.
			  * \sa getActionObservationPair
			  */
			static bool getActionObservationPairOrObservation(
				mrpt::utils::CStream					&inStream,
				CActionCollectionPtr	&action,
				CSensoryFramePtr		&observations,
				CObservationPtr			&observation,
				size_t			& rawlogEntry );

			/** Gets the next consecutive pair action / observation from the rawlog loaded into this object.
			  *   Previous contents of action and observations are discarded (using stlplus::smart_ptr::clear_unique), and
			  *    at exit they contain the new objects read from the rawlog file.
			  *  The input/output variable "rawlogEntry" is just a counter of the last rawlog entry read, for logging or monitoring purposes.
			  * \return false if there was some error, true otherwise.
			  * \sa readActionObservationPair
			  */
			bool getActionObservationPair(
				CActionCollectionPtr  &action,
				CSensoryFramePtr      &observations,
				size_t	              &rawlogEntry ) const;

			/** Tries to auto-detect the external-images directory of the given rawlog file.
			  *  This searches for the existence of the directories:
			  *		- "<rawlog_file_path>/<rawlog_filename>_Images"
			  *		- "<rawlog_file_path>/<rawlog_filename>_images"
			  *		- "<rawlog_file_path>/<rawlog_filename>_IMAGES"
			  *		- "<rawlog_file_path>/Images"  (This one is returned if none of the choices actually exists).
			  *
			  *  The results from this function should be written into mrpt::utils::CImage::IMAGES_PATH_BASE to enable automatic
			  *  loading of extenrnally-stored images in rawlogs.
			  */
			static std::string detectImagesDirectory(const std::string &rawlogFilename);

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CRawlog, mrpt::utils::CSerializable, OBS_IMPEXP )

	} // End of namespace
} // End of namespace

#endif
