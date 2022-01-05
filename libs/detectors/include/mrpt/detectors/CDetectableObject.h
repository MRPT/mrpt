/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CSerializable.h>

#include <cmath>

namespace mrpt::detectors
{
/** Base class that contains common atributes and functions of detectable
 * objects.
 * It was initially thought for detected objects in images from cams, but it's
 * easily
 * expandable to other source types (f.i. scanners).
 * \ingroup mrpt_detectors_grp
 */
class CDetectableObject : public mrpt::serialization::CSerializable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CDetectableObject)

   public:
	/** Must be an unique id for each detectable object */
	std::string m_id;

	/** Observation wich contain the deteted object */
	mrpt::obs::CObservation::Ptr obs;

	inline void setObservation(mrpt::obs::CObservation::Ptr newObs)
	{
		obs = newObs;
	};

};	// End of class

class CDetectable2D : public CDetectableObject
{
	DEFINE_SERIALIZABLE(CDetectable2D, mrpt::detectors)

   public:
	/** 2D Coordinates of detected object */
	float m_x, m_y;
	/** Size of detected object */
	float m_height, m_width;

	/** Extra constructor */
	CDetectable2D(int x = 0, int y = 0, int height = 0, int width = 0)
		: m_x(x), m_y(y), m_height(height), m_width(width){};

	/** Copy pointer content constructor */
	CDetectable2D(const CDetectable2D* d) { *this = *d; };
	/** Compute distance between centers of two detectable 2D objects.
	 * \return calculated distance.
	 */
	inline double distanceTo(const CDetectable2D& d2)
	{
		// Calculate objects centers
		double c_x1 = (m_x + m_width / 2);
		double c_x2 = (d2.m_x + d2.m_width / 2);
		double c_y1 = (m_y + m_height / 2);
		double c_y2 = (d2.m_y + d2.m_height / 2);

		return std::sqrt(std::pow(c_x1 - c_x2, 2) + pow(c_y1 - c_y2, 2));
	};
};

class CDetectable3D : public CDetectable2D
{
	DEFINE_SERIALIZABLE(CDetectable3D, mrpt::detectors)

   public:
	CDetectable3D() = default;

	CDetectable3D(const CDetectable2D::Ptr& object2d);

	/** Copy pointer content constructor */
	CDetectable3D(const CDetectable3D* d) { *this = *d; };
	/** Z coordinate of detected object */
	float m_z;

};	// End of class
}  // namespace mrpt::detectors
