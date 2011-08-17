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
#ifndef CSimpleMap_H
#define CSimpleMap_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>

namespace mrpt
{
namespace slam
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CSimpleMap, mrpt::utils::CSerializable, OBS_IMPEXP )

	/** This class stores a sequence of <Probabilistic Pose,SensoryFrame> pairs, thus a "metric map" can be totally determined with this information.
	 *     The pose of the sensory frame is not deterministic, but described by some PDF. Full 6D poses are used.
	 *
	 *  \note Objects of this class are serialized into (possibly GZ-compressed) files with the extension ".simplemap".
	 *
	 * \note Before MRPT 0.9.0 the name of this class was "CSensFrameProbSequence", that's why there is a typedef with that name to allow backward compatibility.
	 * \sa CSensoryFrame, CPosePDF
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CSimpleMap : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CSimpleMap )

	public:
		/** Constructor
		  */
		CSimpleMap();

		/** Copy constructor
		  */
		CSimpleMap( const CSimpleMap &o );

		/** Copy constructor
		  */
		CSimpleMap & operator = ( const CSimpleMap& o);

		/**  Destructor:
		  */
		virtual ~CSimpleMap();

		/** Save this object to a .simplemap binary file (compressed with gzip)
		  * \sa loadFromFile
		  * \return false on any error.
		  */
		bool saveToFile(const std::string &filName) const;

		/** Load the contents of this object from a .simplemap binary file (possibly compressed with gzip)
		  * \sa saveToFile
		  * \return false on any error.
		  */
		bool loadFromFile(const std::string &filName);


		/** Returns the pairs count.
		  */
		size_t size() const;

		/** Access to the i'th pair, first one is index '0'. NOTE: This method
		  *  returns pointers to the objects inside the list, nor a copy of them,
		  *  so <b>do neither modify them nor delete them</b>.
		  * NOTE: You can pass a NULL pointer if you dont need one of the two variables to be returned.
		  * \exception std::exception On index out of bounds.
		  */
		void  get(size_t index, CPose3DPDFPtr &out_posePDF, CSensoryFramePtr &out_SF ) const ;

		/** Changes the i'th pair, first one is index '0'.
		  *  The referenced object is COPIED, so you can freely destroy the object passed as parameter after calling this.
		  *  If one of the pointers is NULL, the corresponding contents of the current i'th pair is not modified (i.e. if you want just to modify one of the values).
		  * \exception std::exception On index out of bounds.
		  * \sa insert, get, remove
		  */
		void  set(size_t index, const CPose3DPDFPtr &in_posePDF, const CSensoryFramePtr &in_SF );

		/** Changes the i'th pair, first one is index '0'.
		  *  The referenced object is COPIED, so you can freely destroy the object passed as parameter after calling this.
		  *  If one of the pointers is NULL, the corresponding contents of the current i'th pair is not modified (i.e. if you want just to modify one of the values).
		  * This version for 2D PDFs just converts the 2D PDF into 3D before calling the 3D version.
		  * \exception std::exception On index out of bounds.
		  * \sa insert, get, remove
		  */
		void  set(size_t index, const CPosePDFPtr &in_posePDF, const CSensoryFramePtr &in_SF );

		/** Deletes the i'th pair, first one is index '0'.
		  * \exception std::exception On index out of bounds.
		  * \sa insert, get, set
		  */
		void  remove(size_t index);

		/** Add a new pair to the sequence. The objects are copied, so original ones can be free if desired after insertion. */
		void  insert( const CPose3DPDF *in_posePDF, const CSensoryFrame &in_SF );

		/** Add a new pair to the sequence, making a copy of the smart pointer (it's not made unique). */
		void  insert( const CPose3DPDF *in_posePDF, const CSensoryFramePtr &in_SF );

		/** Add a new pair to the sequence, making a copy of the smart pointer (it's not made unique). */
		void  insert( const CPose3DPDFPtr &in_posePDF, const CSensoryFramePtr &in_SF );

		/** Add a new pair to the sequence. The objects are copied, so original ones can be free if desired
		  *  after insertion.
		  * This version for 2D PDFs just converts the 2D PDF into 3D before calling the 3D version.
		  */
		void  insert( const CPosePDFPtr &in_posePDF, const CSensoryFramePtr &in_SF );

		/** Add a new pair to the sequence. The objects are copied, so original ones can be free if desired
		  *  after insertion.
		  * This version for 2D PDFs just converts the 2D PDF into 3D before calling the 3D version.
		  */
		void  insert( const CPosePDF *in_posePDF, const CSensoryFrame &in_SF );

		/** Add a new pair to the sequence. The objects are copied, so original ones can be free if desired
		  *  after insertion.
		  * This version for 2D PDFs just converts the 2D PDF into 3D before calling the 3D version.
		  */
		void  insert( const CPosePDF *in_posePDF, const CSensoryFramePtr &in_SF );

		/** Remove all stored pairs.
		  * \sa remove
		  */
		void  clear();

		/** Change the coordinate origin of all stored poses, for consistency with future new poses to enter in the system. */
		void changeCoordinatesOrigin( const CPose3D  &newOrigin );


		typedef std::pair<CPose3DPDFPtr,CSensoryFramePtr> TPosePDFSensFramePair;
		typedef std::deque<TPosePDFSensFramePair> TPosePDFSensFramePairList;

		typedef TPosePDFSensFramePairList::const_iterator 	const_iterator;
		typedef TPosePDFSensFramePairList::iterator 		iterator;
		typedef TPosePDFSensFramePairList::reverse_iterator reverse_iterator;
		typedef TPosePDFSensFramePairList::const_reverse_iterator const_reverse_iterator;


		inline const_iterator begin() const 	{ return m_posesObsPairs.begin(); }
		inline const_iterator end() const 		{ return m_posesObsPairs.end(); }
		inline iterator begin() 				{ return m_posesObsPairs.begin(); }
		inline iterator end() 					{ return m_posesObsPairs.end(); }

		inline const_reverse_iterator rbegin() const 	{ return m_posesObsPairs.rbegin(); }
		inline const_reverse_iterator rend() const 		{ return m_posesObsPairs.rend(); }
		inline reverse_iterator rbegin() 				{ return m_posesObsPairs.rbegin(); }
		inline reverse_iterator rend() 					{ return m_posesObsPairs.rend(); }

	private:
		/** The stored data */
		TPosePDFSensFramePairList		m_posesObsPairs;

	}; // End of class def.

	// For compatibility with code < 0.9.0
#if MRPT_BACKCOMPATIB_08X
	typedef CSimpleMap		CSensFrameProbSequence;
	typedef CSimpleMapPtr	CSensFrameProbSequencePtr;
#endif


	} // End of namespace
} // End of namespace

#endif
