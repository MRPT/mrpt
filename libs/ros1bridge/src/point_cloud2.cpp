/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/version.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

using namespace mrpt::maps;

static bool check_field(
	const sensor_msgs::PointField& input_field, std::string check_name,
	const sensor_msgs::PointField** output)
{
	bool coherence_error = false;
	if (input_field.name == check_name)
	{
		if (input_field.datatype != sensor_msgs::PointField::FLOAT32 &&
			input_field.datatype != sensor_msgs::PointField::FLOAT64 &&
			input_field.datatype != sensor_msgs::PointField::UINT16 &&
			input_field.datatype != sensor_msgs::PointField::UINT8)
		{
			*output = nullptr;
			coherence_error = true;
		}
		else
		{
			*output = &input_field;
		}
	}
	return coherence_error;
}

static void get_float_from_field(
	const sensor_msgs::PointField* field, const unsigned char* data,
	float& output)
{
	if (field != nullptr)
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
static void get_uint16_from_field(
	const sensor_msgs::PointField* field, const unsigned char* data,
	uint16_t& output)
{
	if (field != nullptr)
	{
		if (field->datatype == sensor_msgs::PointField::UINT16)
			output = *(reinterpret_cast<const uint16_t*>(&data[field->offset]));
		else if (field->datatype == sensor_msgs::PointField::UINT8)
			output = *(reinterpret_cast<const uint8_t*>(&data[field->offset]));
	}
	else
		output = 0;
}

std::set<std::string> mrpt::ros1bridge::extractFields(
	const sensor_msgs::PointCloud2& msg)
{
	std::set<std::string> lst;
	for (const auto& f : msg.fields)
		lst.insert(f.name);
	return lst;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *
 * \return true on sucessful conversion, false on any error.
 */
bool mrpt::ros1bridge::fromROS(
	const sensor_msgs::PointCloud2& msg, CSimplePointsMap& obj)
{
	// Copy point data
	unsigned int num_points = msg.width * msg.height;
	obj.clear();
	obj.reserve(num_points);

	bool incompatible = false;
	const sensor_msgs::PointField *x_field = nullptr, *y_field = nullptr,
								  *z_field = nullptr;

	for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
	{
		incompatible |= check_field(msg.fields[i], "x", &x_field);
		incompatible |= check_field(msg.fields[i], "y", &y_field);
		incompatible |= check_field(msg.fields[i], "z", &z_field);
	}

	if (incompatible || (!x_field || !y_field || !z_field)) return false;

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

bool mrpt::ros1bridge::fromROS(
	const sensor_msgs::PointCloud2& msg, CPointsMapXYZI& obj)
{
	// Copy point data
	unsigned int num_points = msg.width * msg.height;
	obj.clear();
	obj.reserve(num_points);

	bool incompatible = false;
	const sensor_msgs::PointField *x_field = nullptr, *y_field = nullptr,
								  *z_field = nullptr, *i_field = nullptr;

	for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
	{
		incompatible |= check_field(msg.fields[i], "x", &x_field);
		incompatible |= check_field(msg.fields[i], "y", &y_field);
		incompatible |= check_field(msg.fields[i], "z", &z_field);
		incompatible |= check_field(msg.fields[i], "intensity", &i_field);
	}

	if (incompatible || (!x_field || !y_field || !z_field || !i_field))
		return false;

	// If not, memcpy each group of contiguous fields separately
	for (unsigned int row = 0; row < msg.height; ++row)
	{
		const unsigned char* row_data = &msg.data[row * msg.row_step];
		for (uint32_t col = 0; col < msg.width; ++col)
		{
			const unsigned char* msg_data = row_data + col * msg.point_step;

			float x, y, z, i;
			get_float_from_field(x_field, msg_data, x);
			get_float_from_field(y_field, msg_data, y);
			get_float_from_field(z_field, msg_data, z);
			get_float_from_field(i_field, msg_data, i);
			obj.insertPoint(x, y, z);

			obj.setPointIntensity(obj.size() - 1, i / 255.0);
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
bool mrpt::ros1bridge::toROS(
	const CSimplePointsMap& obj, const std_msgs::Header& msg_header,
	sensor_msgs::PointCloud2& msg)
{
	msg.header = msg_header;

	// 2D structure of the point cloud. If the cloud is unordered, height is
	//  1 and width is the length of the point cloud.
	msg.height = 1;
	msg.width = obj.size();

	std::array<std::string, 3> names = {"x", "y", "z"};
	std::array<size_t, 3> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};

	msg.fields.resize(3);
	for (size_t i = 0; i < 3; i++)
	{
		auto& f = msg.fields.at(i);

		f.count = 1;
		f.offset = offsets[i];
		f.datatype = sensor_msgs::PointField::FLOAT32;
		f.name = names[i];
	}

#if MRPT_IS_BIG_ENDIAN
	msg.is_bigendian = true;
#else
	msg.is_bigendian = false;
#endif

	msg.point_step = sizeof(float) * 3;
	msg.row_step = msg.width * msg.point_step;

	// data:
	msg.data.resize(msg.row_step * msg.height);

	const auto& xs = obj.getPointsBufferRef_x();
	const auto& ys = obj.getPointsBufferRef_y();
	const auto& zs = obj.getPointsBufferRef_z();

	float* pointDest = reinterpret_cast<float*>(msg.data.data());
	for (size_t i = 0; i < xs.size(); i++)
	{
		*pointDest++ = xs[i];
		*pointDest++ = ys[i];
		*pointDest++ = zs[i];
	}

	return true;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::obs::CObservationRotatingScan */
bool mrpt::ros1bridge::fromROS(
	const sensor_msgs::PointCloud2& msg,
	mrpt::obs::CObservationRotatingScan& obj,
	const mrpt::poses::CPose3D& sensorPoseOnRobot,
	unsigned int num_azimuth_divisions)
{
	// Copy point data
	obj.timestamp = mrpt::ros1bridge::fromROS(msg.header.stamp);

	bool incompatible = false;
	const sensor_msgs::PointField *x_field = nullptr, *y_field = nullptr,
								  *z_field = nullptr, *i_field = nullptr,
								  *ring_field = nullptr;

	for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
	{
		incompatible |= check_field(msg.fields[i], "x", &x_field);
		incompatible |= check_field(msg.fields[i], "y", &y_field);
		incompatible |= check_field(msg.fields[i], "z", &z_field);
		incompatible |= check_field(msg.fields[i], "ring", &ring_field);
		check_field(msg.fields[i], "intensity", &i_field);
	}

	if (incompatible || (!x_field || !y_field || !z_field || !ring_field))
		return false;

	// 1st: go through the scan and find ring count:
	uint16_t ring_min = 0, ring_max = 0;

	for (unsigned int row = 0; row < msg.height; ++row)
	{
		const unsigned char* row_data = &msg.data[row * msg.row_step];
		for (uint32_t col = 0; col < msg.width; ++col)
		{
			const unsigned char* msg_data = row_data + col * msg.point_step;
			uint16_t ring_id = 0;
			get_uint16_from_field(ring_field, msg_data, ring_id);

			mrpt::keep_min(ring_min, ring_id);
			mrpt::keep_max(ring_max, ring_id);
		}
	}
	ASSERT_NOT_EQUAL_(ring_min, ring_max);

	obj.rowCount = ring_max - ring_min + 1;
	obj.columnCount = num_azimuth_divisions;

	obj.rangeImage.resize(obj.rowCount, obj.columnCount);
	obj.rangeImage.fill(0);

	// Default unit: 1cm
	if (obj.rangeResolution == 0) obj.rangeResolution = 1e-2;

	if (i_field)
	{
		obj.intensityImage.resize(obj.rowCount, obj.columnCount);
		obj.intensityImage.fill(0);
	}
	else
		obj.intensityImage.resize(0, 0);

	// If not, memcpy each group of contiguous fields separately
	for (unsigned int row = 0; row < msg.height; ++row)
	{
		const unsigned char* row_data = &msg.data[row * msg.row_step];
		for (uint32_t col = 0; col < msg.width; ++col)
		{
			const unsigned char* msg_data = row_data + col * msg.point_step;

			float x, y, z;
			uint16_t ring_id = 0;
			get_float_from_field(x_field, msg_data, x);
			get_float_from_field(y_field, msg_data, y);
			get_float_from_field(z_field, msg_data, z);
			get_uint16_from_field(ring_field, msg_data, ring_id);

			const mrpt::math::TPoint3D localPt =
				sensorPoseOnRobot.inverseComposePoint(
					mrpt::math::TPoint3D(x, y, z));
			const double azimuth = std::atan2(localPt.y, localPt.x);

			const auto az_idx = lround(
				(num_azimuth_divisions - 1) * (azimuth + M_PI) / (2 * M_PI));
			ASSERT_GE_(az_idx, 0);
			ASSERT_LE_(az_idx, num_azimuth_divisions - 1);

			// Store in matrix form:
			obj.rangeImage(ring_id, az_idx) =
				lround(localPt.norm() / obj.rangeResolution);

			if (i_field)
			{
				float intensity;
				get_float_from_field(i_field, msg_data, intensity);
				ASSERT_LE_(intensity, 255.0f);
				obj.intensityImage(ring_id, az_idx) = lround(intensity);
			}
		}
	}
	return true;
}
