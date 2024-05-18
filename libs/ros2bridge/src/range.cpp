/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
  APPLICATION: mrpt_ros bridge
  FILE: range.cpp
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/range.h>

#include <cmath>

template <class T, class = void>
struct has_variance : std::false_type
{
};

template <class T>
struct has_variance<T, std::void_t<decltype(T::variance)>> : std::true_type
{
};

// for "if constexpr" to work (avoid build errors if field does not exist)
// it must be inside a template:
template <class MSG_T>
void fromROS_variance(const MSG_T& msg, mrpt::obs::CObservationRange& obj)
{
  if constexpr (has_variance<MSG_T>::value)
  {
    obj.sensedData.at(0).sensorNoiseStdDeviation = std::sqrt(msg.variance);
  }
}
template <class MSG_T>
void toROS_variance(MSG_T& msg, const mrpt::obs::CObservationRange::TMeasurement& m)
{
  if constexpr (has_variance<MSG_T>::value)
  {
    msg.variance = mrpt::square(m.sensorNoiseStdDeviation);
  }
}

bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::Range& msg, mrpt::obs::CObservationRange& obj)
{
  obj.minSensorDistance = msg.min_range;
  obj.maxSensorDistance = msg.max_range;
  obj.sensorConeAperture = msg.field_of_view;

  obj.sensedData.resize(1);
  obj.sensedData.at(0).sensedDistance = msg.range;

  // See: https://github.com/MRPT/mrpt/issues/1270
  fromROS_variance(msg, obj);

  return true;
}

bool mrpt::ros2bridge::toROS(
    const mrpt::obs::CObservationRange& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::Range* msg)
{
  const auto num_range = obj.sensedData.size();

  // 1) sensor_msgs::msg::Range:: header
  for (size_t i = 0; i < num_range; i++) msg[i].header = msg_header;

  // 2) sensor_msg::Range parameters
  for (size_t i = 0; i < num_range; i++)
  {
    msg[i].max_range = obj.maxSensorDistance;
    msg[i].min_range = obj.minSensorDistance;
    msg[i].field_of_view = obj.sensorConeAperture;

    // See: https://github.com/MRPT/mrpt/issues/1270
    toROS_variance(msg[i], obj.sensedData[i]);
  }

  /// following part needs to be double checked, it looks incorrect
  /// ROS has single number float for range, MRPT has a list of
  /// sensedDistances
  for (size_t i = 0; i < num_range; i++) msg[i].range = obj.sensedData.at(i).sensedDistance;

  /// currently the following are not available in MRPT for corresponding
  /// range ROS message NO corresponding value for MRPT radiation_type at
  /// http://mrpt.ual.es/reference/devel/_c_observation_range_8h_source.html
  // msg.radiation_type
  return true;
}

/// Range ROS message
/*
uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
*/
