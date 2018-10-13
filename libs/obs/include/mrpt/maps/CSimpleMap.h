/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt::maps
{
/** This class stores a sequence of <Probabilistic Pose,SensoryFrame> pairs,
 * thus a "metric map" can be totally determined with this information.
 *     The pose of the sensory frame is not deterministic, but described by some
 * PDF. Full 6D poses are used.
 *
 *  \note Objects of this class are serialized into (possibly GZ-compressed)
 * files with the extension ".simplemap".
 *
 * \note Before MRPT 0.9.0 the name of this class was "CSensFrameProbSequence",
 * that's why there is a type with that name to allow backward compatibility.
 * \sa CSensoryFrame, CPosePDF
 * \ingroup mrpt_obs_grp
 */
class CSimpleMap : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CSimpleMap)
   public:
	/** Default constructor (empty map) */
	CSimpleMap() = default;
	/** Copy constructor */
	CSimpleMap(const CSimpleMap& o);
	/** Copy */
	CSimpleMap& operator=(const CSimpleMap& o);

	/** \name Map access and modification
	 * @{ */

	/** Save this object to a .simplemap binary file (compressed with gzip)
	 * \sa loadFromFile
	 * \return false on any error. */
	bool saveToFile(const std::string& filName) const;

	/** Load the contents of this object from a .simplemap binary file (possibly
	 * compressed with gzip)
	 * \sa saveToFile
	 * \return false on any error. */
	bool loadFromFile(const std::string& filName);

	/** Returns the count of pairs (pose,sensory data) */
	size_t size() const;
	/** Returns size()!=0 */
	bool empty() const;

	/** Access to the i'th pair, first one is index '0'. NOTE: This method
	 *  returns pointers to the objects inside the list, nor a copy of them,
	 *  so <b>do neither modify them nor delete them</b>.
	 * NOTE: You can pass a nullptr pointer if you dont need one of the two
	 * variables to be returned.
	 * \exception std::exception On index out of bounds.
	 */
	void get(
		size_t index, mrpt::poses::CPose3DPDF::Ptr& out_posePDF,
		mrpt::obs::CSensoryFrame::Ptr& out_SF) const;

	/** Changes the i'th pair, first one is index '0'.
	 *  The referenced object is COPIED, so you can freely destroy the object
	 * passed as parameter after calling this.
	 *  If one of the pointers is nullptr, the corresponding contents of the
	 * current i'th pair is not modified (i.e. if you want just to modify one of
	 * the values).
	 * \exception std::exception On index out of bounds.
	 * \sa insert, get, remove
	 */
	void set(
		size_t index, const mrpt::poses::CPose3DPDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Changes the i'th pair, first one is index '0'.
	 *  The referenced object is COPIED, so you can freely destroy the object
	 * passed as parameter after calling this.
	 *  If one of the pointers is nullptr, the corresponding contents of the
	 * current i'th pair is not modified (i.e. if you want just to modify one of
	 * the values).
	 * This version for 2D PDFs just converts the 2D PDF into 3D before calling
	 * the 3D version.
	 * \exception std::exception On index out of bounds.
	 * \sa insert, get, remove
	 */
	void set(
		size_t index, const mrpt::poses::CPosePDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Deletes the i'th pair, first one is index '0'.
	 * \exception std::exception On index out of bounds.
	 * \sa insert, get, set
	 */
	void remove(size_t index);

	/** Add a new pair to the sequence. The objects are copied, so original ones
	 * can be free if desired after insertion. */
	void insert(
		const mrpt::poses::CPose3DPDF* in_posePDF,
		const mrpt::obs::CSensoryFrame& in_SF);

	/** Add a new pair to the sequence, making a copy of the smart pointer (it's
	 * not made unique). */
	void insert(
		const mrpt::poses::CPose3DPDF* in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Add a new pair to the sequence, making a copy of the smart pointer (it's
	 * not made unique). */
	void insert(
		const mrpt::poses::CPose3DPDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Insert a new pair to the sequence, making a copy of the smart pointer
	(it's
	 * not made unique) to
	\param index Position in the simplemap where new element will be inserted to
	*/
	void insertToPos(
		size_t index, const mrpt::poses::CPose3DPDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Add a new pair to the sequence. The objects are copied, so original ones
	 * can be free if desired
	 *  after insertion.
	 * This version for 2D PDFs just converts the 2D PDF into 3D before calling
	 * the 3D version.
	 */
	void insert(
		const mrpt::poses::CPosePDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Add a new pair to the sequence. The objects are copied, so original ones
	 * can be free if desired
	 *  after insertion.
	 * This version for 2D PDFs just converts the 2D PDF into 3D before calling
	 * the 3D version.
	 */
	void insert(
		const mrpt::poses::CPosePDF* in_posePDF,
		const mrpt::obs::CSensoryFrame& in_SF);

	/** Add a new pair to the sequence. The objects are copied, so original ones
	 * can be free if desired
	 *  after insertion.
	 * This version for 2D PDFs just converts the 2D PDF into 3D before calling
	 * the 3D version.
	 */
	void insert(
		const mrpt::poses::CPosePDF* in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Remove all stored pairs.  \sa remove */
	void clear();

	/** Change the coordinate origin of all stored poses, for consistency with
	 * future new poses to enter in the system. */
	void changeCoordinatesOrigin(const mrpt::poses::CPose3D& newOrigin);

	/** @} */

	/** \name Iterators API
	 * @{ */
	using TPosePDFSensFramePair =
		std::pair<mrpt::poses::CPose3DPDF::Ptr, mrpt::obs::CSensoryFrame::Ptr>;
	using TPosePDFSensFramePairList = std::deque<TPosePDFSensFramePair>;

	using const_iterator = TPosePDFSensFramePairList::const_iterator;
	using iterator = TPosePDFSensFramePairList::iterator;
	using reverse_iterator = TPosePDFSensFramePairList::reverse_iterator;
	using const_reverse_iterator =
		TPosePDFSensFramePairList::const_reverse_iterator;

	inline const_iterator begin() const { return m_posesObsPairs.begin(); }
	inline const_iterator end() const { return m_posesObsPairs.end(); }
	inline iterator begin() { return m_posesObsPairs.begin(); }
	inline iterator end() { return m_posesObsPairs.end(); }
	inline const_reverse_iterator rbegin() const
	{
		return m_posesObsPairs.rbegin();
	}
	inline const_reverse_iterator rend() const
	{
		return m_posesObsPairs.rend();
	}
	inline reverse_iterator rbegin() { return m_posesObsPairs.rbegin(); }
	inline reverse_iterator rend() { return m_posesObsPairs.rend(); }
	/** @} */

   private:
	/** The stored data */
	TPosePDFSensFramePairList m_posesObsPairs;

};  // End of class def.

}  // namespace mrpt::maps
