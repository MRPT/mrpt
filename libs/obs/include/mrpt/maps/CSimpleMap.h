/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CSerializable.h>

#include <tuple>

namespace mrpt::maps
{
/** A view-based representation of a metric map.
 *
 *  This comprises a list of `<ProbabilisticPose,SensoryFrame>` pairs, that is,
 *  the **poses** (keyframes) from which a set of **observations** where
 * gathered:
 *  - Poses, in the global `map` frame of reference, are stored as probabilistic
 * PDFs over SE(3) as instances of mrpt::poses::CPose3DPDF
 *  - Observations are stored as mrpt::obs::CSensoryFrame.
 *
 * Note that in order to generate an actual metric map (occupancy grid, point
 * cloud, octomap, etc.) from a "simple map", you must instantiate the desired
 * metric map class and invoke its virtual method
 * mrpt::maps::CMetricMap::loadFromProbabilisticPosesAndObservations().
 *
 * \note Objects of this class are serialized into GZ-compressed
 *       files with the extension `.simplemap`.
 *       See [Robotics file formats](robotics_file_formats.html).
 *
 * \sa mrpt::obs::CSensoryFrame, mrpt::poses::CPose3DPDF, mrpt::maps::CMetricMap
 *
 * \ingroup mrpt_obs_grp
 */
class CSimpleMap : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CSimpleMap, mrpt::maps)
   public:
	CSimpleMap() = default;	 //!< Default ctor: empty map
	~CSimpleMap() = default;

	/** Copy constructor, makes a deep copy of all data. */
	CSimpleMap(const CSimpleMap& o);

	/** Copy, making a deep copy of all data. */
	CSimpleMap& operator=(const CSimpleMap& o);

	struct Pair
	{
		Pair() = default;
		~Pair() = default;

		mrpt::poses::CPose3DPDF::Ptr pose;
		mrpt::obs::CSensoryFrame::Ptr sf;
	};

	struct ConstPair
	{
		ConstPair() = default;
		~ConstPair() = default;

		ConstPair(const Pair& p) : pose(p.pose), sf(p.sf) {}

		mrpt::poses::CPose3DPDF::ConstPtr pose;
		mrpt::obs::CSensoryFrame::ConstPtr sf;
	};

	/** \name Map access and modification
	 * @{ */

	/** Save this object to a .simplemap binary file (compressed with gzip)
	 * See [Robotics file formats](robotics_file_formats.html).
	 * \sa loadFromFile()
	 * \return false on any error. */
	bool saveToFile(const std::string& filName) const;

	/** Load the contents of this object from a .simplemap binary file (possibly
	 * compressed with gzip)
	 * See [Robotics file formats](robotics_file_formats.html).
	 * \sa saveToFile()
	 * \return false on any error. */
	bool loadFromFile(const std::string& filName);

	/** Returns the count of (pose,sensoryFrame) pairs */
	size_t size() const { return m_posesObsPairs.size(); }

	/** Returns size()!=0 */
	bool empty() const { return m_posesObsPairs.empty(); }

	/** Access to the 0-based index i'th pair.
	 * \exception std::exception On index out of bounds.
	 */
	void get(
		size_t index, mrpt::poses::CPose3DPDF::ConstPtr& out_posePDF,
		mrpt::obs::CSensoryFrame::ConstPtr& out_SF) const
	{
		ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
		out_posePDF = m_posesObsPairs[index].pose;
		out_SF = m_posesObsPairs[index].sf;
	}
	/// \overload
	std::tuple<
		mrpt::poses::CPose3DPDF::ConstPtr, mrpt::obs::CSensoryFrame::ConstPtr>
		get(size_t index) const
	{
		ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
		return {m_posesObsPairs[index].pose, m_posesObsPairs[index].sf};
	}

	ConstPair getAsPair(size_t index) const
	{
		ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
		return m_posesObsPairs.at(index);
	}
	Pair& getAsPair(size_t index)
	{
		ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
		return m_posesObsPairs.at(index);
	}

	/// \overload
	void get(
		size_t index, mrpt::poses::CPose3DPDF::Ptr& out_posePDF,
		mrpt::obs::CSensoryFrame::Ptr& out_SF)
	{
		ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
		out_posePDF = m_posesObsPairs[index].pose;
		out_SF = m_posesObsPairs[index].sf;
	}

	/// \overload
	std::tuple<mrpt::poses::CPose3DPDF::Ptr, mrpt::obs::CSensoryFrame::Ptr> get(
		size_t index)
	{
		ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
		return {m_posesObsPairs[index].pose, m_posesObsPairs[index].sf};
	}

	/** Changes the 0-based index i'th pair.
	 *  If one of either `in_posePDF` or `in_SF` are empty `shared_ptr`s, the
	 * corresponding field in the map is not modified.
	 *
	 * \exception std::exception On index out of bounds.
	 * \sa insert, get, remove
	 */
	void set(
		size_t index, const mrpt::poses::CPose3DPDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/// \overload
	void set(size_t index, const Pair& poseSF)
	{
		set(index, poseSF.pose, poseSF.sf);
	}

	/// \overload For SE(2) pose PDF, internally converted to SE(3).
	void set(
		size_t index, const mrpt::poses::CPosePDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Deletes the 0-based index i'th pair.
	 * \exception std::exception On index out of bounds.
	 * \sa insert, get, set
	 */
	void remove(size_t index);

	/** Adds a new keyframe (SE(3) pose) to the view-based map, making a deep
	 * copy of the pose PDF (observations within the SF are always copied as
	 * `shared_ptr`s).
	 */
	void insert(
		const mrpt::poses::CPose3DPDF& in_posePDF,
		const mrpt::obs::CSensoryFrame& in_SF);

	/** Adds a new keyframe (SE(3) pose) to the view-based map.
	 *  Both shared pointers are copied (shallow object copies).
	 */
	void insert(
		const mrpt::poses::CPose3DPDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/// \overload
	void insert(const Pair& poseSF) { insert(poseSF.pose, poseSF.sf); }

	/** Adds a new keyframe (SE(2) pose) to the view-based map, making a deep
	 * copy of the pose PDF (observations within the SF are always copied as
	 * `shared_ptr`s).
	 */
	void insert(
		const mrpt::poses::CPosePDF& in_posePDF,
		const mrpt::obs::CSensoryFrame& in_SF);

	/** Adds a new keyframe (SE(2) pose) to the view-based map.
	 *  Both shared pointers are copied (shallow object copies).
	 */
	void insert(
		const mrpt::poses::CPosePDF::Ptr& in_posePDF,
		const mrpt::obs::CSensoryFrame::Ptr& in_SF);

	/** Remove all stored pairs.  \sa remove */
	void clear() { m_posesObsPairs.clear(); }

	/** Change the coordinate origin of all stored poses, that is, translates
	 * and rotates the map such that the old SE(3) origin (identity
	 * transformation) becomes the new provided one.
	 */
	void changeCoordinatesOrigin(const mrpt::poses::CPose3D& newOrigin);

	/** @} */

	/** \name Iterators API
	 * @{ */
	using TPosePDFSensFramePairList = std::deque<Pair>;

	using const_iterator = TPosePDFSensFramePairList::const_iterator;
	using iterator = TPosePDFSensFramePairList::iterator;
	using reverse_iterator = TPosePDFSensFramePairList::reverse_iterator;
	using const_reverse_iterator =
		TPosePDFSensFramePairList::const_reverse_iterator;

	const_iterator begin() const { return m_posesObsPairs.begin(); }
	const_iterator end() const { return m_posesObsPairs.end(); }
	const_iterator cbegin() const { return m_posesObsPairs.cbegin(); }
	const_iterator cend() const { return m_posesObsPairs.cend(); }
	iterator begin() { return m_posesObsPairs.begin(); }
	iterator end() { return m_posesObsPairs.end(); }
	const_reverse_iterator rbegin() const { return m_posesObsPairs.rbegin(); }
	const_reverse_iterator rend() const { return m_posesObsPairs.rend(); }
	const_reverse_iterator crbegin() const { return m_posesObsPairs.crbegin(); }
	const_reverse_iterator crend() const { return m_posesObsPairs.crend(); }
	reverse_iterator rbegin() { return m_posesObsPairs.rbegin(); }
	reverse_iterator rend() { return m_posesObsPairs.rend(); }
	/** @} */

   private:
	/** The stored data */
	TPosePDFSensFramePairList m_posesObsPairs;

};	// End of class def.

}  // namespace mrpt::maps
