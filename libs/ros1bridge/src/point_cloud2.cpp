/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "mrpt/ros1bridge/point_cloud2.h"

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/version.h>
using namespace mrpt::maps;

namespace mrpt::ros1bridge
{
inline bool check_field(
	const sensor_msgs::PointField& input_field, std::string check_name,
	const sensor_msgs::PointField** output)
{
	bool coherence_error = false;
	if (input_field.name == check_name)
	{
		if (input_field.datatype != sensor_msgs::PointField::FLOAT32 &&
			input_field.datatype != sensor_msgs::PointField::FLOAT64)
		{
			*output = NULL;
			coherence_error = true;
		}
		else
		{
			*output = &input_field;
		}
	}
	return coherence_error;
}

inline void get_float_from_field(
	const sensor_msgs::PointField* field, const unsigned char* data,
	float& output)
{
	if (field != NULL)
	{
		if (field->datatype == sensor_msgs::PointField::FLOAT32)
			output = *(reinterpret_cast<const float*>(&data[field->offset]));
		else
			output = (float)(*(
				reinterpret_cast<const double*>(&data[field->offset])));
	}
	else
		output = 0.0;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *
 * \return true on sucessful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::PointCloud2& msg, CSimplePointsMap& obj)
{
	// Copy point data
	unsigned int num_points = msg.width * msg.height;
	obj.clear();
	obj.reserve(num_points);

	bool incompatible_clouds = false;
	const sensor_msgs::PointField *x_field = NULL, *y_field = NULL,
								  *z_field = NULL;

	for (unsigned int i = 0; i < msg.fields.size() && !incompatible_clouds; i++)
	{
		incompatible_clouds |= check_field(msg.fields[i], "x", &x_field);
		incompatible_clouds |= check_field(msg.fields[i], "y", &y_field);
		incompatible_clouds |= check_field(msg.fields[i], "z", &z_field);
	}

	if (incompatible_clouds ||
		(x_field == NULL && y_field == NULL && z_field == NULL))
		return false;

	// If not, memcpy each group of contiguous fields separately
	for (unsigned int row = 0; row < msg.height; ++row)
	{
		const unsigned char* row_data = &msg.data[row * msg.row_step];
		for (uint32_t col = 0; col < msg.width; ++col)
		{
			const unsigned char* msg_data = row_data + col * msg.point_step;

			float x, y, z;
			get_float_from_field(x_field, msg_data, x);
			get_float_from_field(y_field, msg_data, y);
			get_float_from_field(z_field, msg_data, z);
			obj.insertPoint(x, y, z);
		}
	}

	return true;
}

/** Convert mrpt::slam::CSimplePointsMap -> sensor_msgs/PointCloud2
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 *  Since CSimplePointsMap only contains (x,y,z) data,
 * sensor_msgs::PointCloud2::channels will be empty.
 * \return true on sucessful conversion, false on any error.
 */
bool toROS(
	const CSimplePointsMap& obj, const std_msgs::Header& msg_header,
	sensor_msgs::PointCloud2& msg)
{
	throw ros::Exception("not implemented yet.");
}

}  // namespace mrpt::ros1bridge
