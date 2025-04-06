/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TTwist3D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CSerializable.h>

#include <tuple>

namespace mrpt::maps
{
/** A view-based map: a set of poses and what the robot saw from those poses.
 *
 * A simplemap comprises a sequence of tuples, each containing:
 * - The **keyframe SE(3) pose** of the robot, including (optionally) its
 *   uncertainty, as instances of mrpt::poses::CPose3DPDF
 * - The **raw observations** from that keyframe, in a mrpt::obs::CSensoryFrame
 * - Optionally, the **twist** (linear and angular velocity) of the robot in the
 *   local frame of reference, at that moment. It can be used to undistort data
 *   from a rotatory lidar, for example.
 *
 * To generate an actual metric map (occupancy grid, point cloud, octomap, etc.)
 * from a "simple map", the user must instantiate the desired metric map
 * class(es) and invoke its virtual method
 * mrpt::maps::CMetricMap::loadFromSimpleMap().
 *
 * Users can also use the new top-level [library
 * mp2p_icp_filters](https://github.com/MOLAorg/mp2p_icp/) and its CLI
 * application
 * [sm2mm](https://github.com/MOLAorg/mp2p_icp/tree/master/apps/sm2mm)
 * (simple-map to metric-map)
 * to generate metric maps including pre-processing of raw data in a flexible
 * way.
 *
 * To programatically change an existing simplemap, use the non-const get()
 * method and modify the returned reference.
 *
 * Copy constructor and copy operator makes shallow copies of all data.
 * A makeDeepCopy() method is also provided which duplicates all internal data,
 * if really needed.
 *
 * \note Objects of this class are serialized into GZ-compressed files with
 *       the extension `.simplemap`.
 *       See [Robotics file formats](robotics_file_formats.html).
 *
 * \sa mrpt::obs::CSensoryFrame, mrpt::poses::CPose3DPDF,
 *     mrpt::maps::CMetricMap, https://github.com/MOLAorg/mp2p_icp/
 *
 * \ingroup mrpt_obs_grp
 */
class CSimpleMap : public mrpt::serialization::CSerializable
{
  DEFINE_SERIALIZABLE(CSimpleMap, mrpt::maps)
 public:
  CSimpleMap() = default;  //!< Default ctor: empty map
  ~CSimpleMap() = default;

  /** makes a deep copy of all data  */
  [[nodiscard]] CSimpleMap makeDeepCopy();

  struct Keyframe
  {
    Keyframe() = default;
    ~Keyframe() = default;

    Keyframe(
        const mrpt::poses::CPose3DPDF::Ptr& kfPose,
        const mrpt::obs::CSensoryFrame::Ptr& kfSf,
        const std::optional<mrpt::math::TTwist3D>& kflocalTwist = std::nullopt) :
        pose(kfPose), sf(kfSf), localTwist(kflocalTwist)
    {
    }

    mrpt::poses::CPose3DPDF::Ptr pose;
    mrpt::obs::CSensoryFrame::Ptr sf;  //!< raw observations
    std::optional<mrpt::math::TTwist3D> localTwist;
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

  /** Returns the number of keyframes in the map */
  size_t size() const { return m_keyframes.size(); }

  /** Returns size()!=0 */
  bool empty() const { return m_keyframes.empty(); }

  /// const accessor
  const Keyframe& get(size_t index) const
  {
    ASSERT_LT_(index, m_keyframes.size());
    return m_keyframes[index];
  }

  /// non-const accessor, returning a reference suitable for modification
  Keyframe& get(size_t index)
  {
    ASSERT_LT_(index, m_keyframes.size());
    return m_keyframes[index];
  }

  /** Deletes the 0-based index i'th keyframe.
   * \exception std::exception On index out of bounds.
   * \sa insert, get, set
   */
  void remove(size_t index);

  /** Adds a new keyframe (SE(3) pose) to the view-based map.
   *  Both shared pointers are copied (shallow object copies).
   */
  void insert(const Keyframe& kf);

  /** Adds a new keyframe (SE(3) pose) to the view-based map.
   *  Both shared pointers are copied (shallow object copies).
   */
  void insert(
      const mrpt::poses::CPose3DPDF::Ptr& in_posePDF,
      const mrpt::obs::CSensoryFrame::Ptr& in_SF,
      const std::optional<mrpt::math::TTwist3D>& twist = std::nullopt)
  {
    Keyframe kf;
    kf.pose = in_posePDF;
    kf.sf = in_SF;
    kf.localTwist = twist;
    insert(kf);
  }

  /** Remove all stored keyframes.  \sa remove */
  void clear() { m_keyframes.clear(); }

  /** Change the coordinate origin of all stored poses, that is, translates
   * and rotates the map such that the old SE(3) origin (identity
   * transformation) becomes the new provided one.
   */
  void changeCoordinatesOrigin(const mrpt::poses::CPose3D& newOrigin);

  /** @} */

  /** \name Iterators API
   * @{ */
  using KeyframeList = std::deque<Keyframe>;

  using const_iterator = KeyframeList::const_iterator;
  using iterator = KeyframeList::iterator;
  using reverse_iterator = KeyframeList::reverse_iterator;
  using const_reverse_iterator = KeyframeList::const_reverse_iterator;

  const_iterator begin() const { return m_keyframes.begin(); }
  const_iterator end() const { return m_keyframes.end(); }
  const_iterator cbegin() const { return m_keyframes.cbegin(); }
  const_iterator cend() const { return m_keyframes.cend(); }
  iterator begin() { return m_keyframes.begin(); }
  iterator end() { return m_keyframes.end(); }
  const_reverse_iterator rbegin() const { return m_keyframes.rbegin(); }
  const_reverse_iterator rend() const { return m_keyframes.rend(); }
  const_reverse_iterator crbegin() const { return m_keyframes.crbegin(); }
  const_reverse_iterator crend() const { return m_keyframes.crend(); }
  reverse_iterator rbegin() { return m_keyframes.rbegin(); }
  reverse_iterator rend() { return m_keyframes.rend(); }
  /** @} */

 private:
  /** The stored data */
  KeyframeList m_keyframes;

};  // End of class def.

}  // namespace mrpt::maps
