/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** A "sensory frame" is a set of observations taken by the robot
 *  approximately at the same time, so they can be considered as a multi-sensor
 *  "snapshot" of the environment.
 * It can contain "observations" of many different kinds.
 *
 *  New observations can be added using:
 *
 * \code
 * // Create a smart pointer containing an object of class "CObservationXXX"
 * CObservationXXX::Ptr	o = std::make_shared<CObservationXXX>();
 * // o->... // fill it...
 * CSensoryFrame sf;
 * sf.insert(o);
 * \endcode
 *
 * The following methods are equivalent for adding new observations to a
 * "sensory frame":
 * - CSensoryFrame::operator +=
 * - CSensoryFrame::push_back
 * - CSensoryFrame::insert
 *
 * To examine the objects within a sensory frame, the following methods exist:
 * - CSensoryFrame::getObservationByClass : Looks for some specific observation
 * class.
 * - CSensoryFrame::begin : To iterate over all observations.
 * - CSensoryFrame::getObservationByIndex : To query by index.
 *
 * Note that `shared_ptr<>`s to the observations are stored, so
 * a copy of a CSensoryFrame will contain references to the **same** objects,
 * i.e. copies are shallows copies, not deep copies.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CSensoryFrame : public mrpt::serialization::CSerializable
{
  DEFINE_SERIALIZABLE(CSensoryFrame, mrpt::obs)

 public:
  CSensoryFrame() = default;  //!< Default ctor

  /** @name Cached points map
    @{  */
  /** Returns the cached points map representation of the scan, if already
   * build with buildAuxPointsMap(), or nullptr otherwise.
   * Usage:
   *  \code
   *    mrpt::maps::CPointsMap *map =
   * obs->getAuxPointsMap<mrpt::maps::CPointsMap>();
   *  \endcode
   * \sa buildAuxPointsMap
   */
  template <class POINTSMAP>
  inline const POINTSMAP* getAuxPointsMap() const
  {
    return static_cast<POINTSMAP*>(m_cachedMap.get());
  }

  /** Returns a cached points map representing this laser scan, building it
   * upon the first call.
   * \param options Can be nullptr to use default point maps' insertion
   * options, or a pointer to a "CPointsMap::TInsertionOptions" structure to
   * override some params.
   * Usage:
   *  \code
   *    mrpt::maps::CPointsMap *map =
   * sf->buildAuxPointsMap<mrpt::maps::CPointsMap>(&options or nullptr);
   *  \endcode
   * \sa getAuxPointsMap
   */
  template <class POINTSMAP>
  inline const POINTSMAP* buildAuxPointsMap(const void* options = nullptr) const
  {
    internal_buildAuxPointsMap(options);
    return static_cast<POINTSMAP*>(m_cachedMap.get());
  }

  /** @} */

  /** Clear the container, so it holds no observations. */
  void clear();

  /** Insert all the observations in this SF into a metric map or any kind
   *(see mrpt::maps::CMetricMap).
   *  It calls CObservation::insertObservationInto for all stored observation.
   * \param theMap The map where this observation is to be inserted: the map
   *will be updated.
   * \param robotPose The pose of the robot base for this observation,
   *relative to the target metric map. Set to nullptr (default) to use SE(3)
   *identity, i.e. the origin.
   *
   * \return Returns true if the map has been updated, or false if this
   *observations have nothing to do with the metric map (e.g. trying to insert
   *an image into a gridmap).
   *
   * \sa mrpt::maps::CMetricMap, CObservation::insertObservationInto,
   *CMetricMap::insertObservation
   */
  bool insertObservationsInto(
      mrpt::maps::CMetricMap& theMap,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) const;

  /// \overload
  inline bool insertObservationsInto(
      mrpt::maps::CMetricMap::Ptr& theMap,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) const
  {
    return insertObservationsInto(*theMap, robotPose);
  }

  /** You can use "sf1+=sf2;" to add all observations in sf2 to sf1.
   */
  void operator+=(const CSensoryFrame& sf);

  /** You can use "sf+=obs;" to add the observation "obs" to the "sf1".
   */
  void operator+=(const CObservation::Ptr& obs);

  /** Insert a new observation to the sensory frame. */
  void push_back(const CObservation::Ptr& obs);

  /// Synonym with push_back()
  inline void insert(const CObservation::Ptr& obs) { push_back(obs); }

  /** Returns the i'th observation of a given class (or of a descendant
  class), or nullptr if there is no such observation in the array.
  *  Example:
  * \code
     CObservationImage::Ptr obs =
  m_SF->getObservationByClass<CObservationImage>();
  * \endcode
  * By default (ith=0), the first observation is returned.
  */
  template <typename T>
  typename T::Ptr getObservationByClass(size_t ith = 0) const
  {
    MRPT_START
    size_t foundCount = 0;
    const mrpt::rtti::TRuntimeClassId* class_ID = &T::GetRuntimeClassIdStatic();
    for (const auto& it : *this)
      if (it->GetRuntimeClass()->derivedFrom(class_ID))
        if (foundCount++ == ith) return std::dynamic_pointer_cast<T>(it);
    return typename T::Ptr();  // Not found: return empty smart pointer
    MRPT_END
  }

  /** You can use CSensoryFrame::begin to get a iterator to the first element.
   */
  using iterator = std::deque<CObservation::Ptr>::iterator;

  /** You can use CSensoryFrame::begin to get a iterator to the first element.
   */
  using const_iterator = std::deque<CObservation::Ptr>::const_iterator;

  /** Returns a constant iterator to the first observation: this is an example
   *of usage:
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
  /** Returns a constant iterator to the end of the list of observations: this
   *is an example of usage:
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
  /** Returns a iterator to the first observation: this is an example of
   *usage:
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
  /** Returns a iterator to the end of the list of observations: this is an
   *example of usage:
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
  inline bool empty() const { return m_observations.empty(); }
  /** Removes the i'th observation in the list (0=first). */
  void eraseByIndex(size_t idx);

  /** Removes the given observation in the list, and return an iterator to the
   * next element (or this->end() if it was the last one).
   */
  iterator erase(const iterator& it);

  /** Removes all the observations that match a given sensorLabel.
   */
  void eraseByLabel(const std::string& label);

  /** Returns the i'th observation in the list (0=first).
   *  \throw std::exception If out of range.
   * \sa begin, size
   */
  const CObservation::Ptr& getObservationByIndex(size_t idx) const;
  /// \overload
  CObservation::Ptr& getObservationByIndex(size_t idx);

  /** Returns the i'th observation in the list (0=first), and as a different
   * smart pointer type:
   * \code
   *   sf.getObservationByIndexAs<CObservationStereoImages::Ptr>(i);
   * \endcode
   * \sa begin, size
   */
  template <typename T>
  T getObservationByIndexAs(size_t idx) const
  {
    return std::dynamic_pointer_cast<typename T::element_type>(getObservationByIndex(idx));
  }

  /** Returns the i'th observation in the list with the given "sensorLabel"
   * (0=first).
   * \return The observation, or nullptr if not found.
   * \sa begin, size
   */
  CObservation::Ptr getObservationBySensorLabel(const std::string& label, size_t idx = 0) const;

  /** Returns the i'th observation in the list with the given "sensorLabel"
   * (0=first), and as a different smart pointer type:
   * \code
   *   sf.getObservationBySensorLabelAs<CObservationStereoImages::Ptr>(i);
   * \endcode
   * \sa begin, size
   */
  template <typename T>
  T getObservationBySensorLabelAs(const std::string& label, size_t idx = 0) const
  {
    return std::dynamic_pointer_cast<typename T::element_type>(
        getObservationBySensorLabel(label, idx));
  }

  /** Efficiently swaps the contents of two objects.
   */
  void swap(CSensoryFrame& sf);

 protected:
  /** The set of observations taken at the same time instant. See the top of
   * this page for instructions on accessing this.
   */
  std::deque<CObservation::Ptr> m_observations;

  /** A point cloud map, build only under demand by the methods
   * getAuxPointsMap() and buildAuxPointsMap(). It's a generic smart pointer
   * to avoid depending here in the library mrpt-obs on classes on other
   * libraries.
   */
  mutable mrpt::maps::CMetricMap::Ptr m_cachedMap;

  /** Internal method, used from buildAuxPointsMap() */
  void internal_buildAuxPointsMap(const void* options = nullptr) const;

};  // End of class def.

}  // namespace mrpt::obs
