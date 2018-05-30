/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CPOINT3D_H
#define CPOINT3D_H

#include <mrpt/poses/CPoint.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::poses
{
/** A class used to store a 3D point.
 *
 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,
 * or refer
 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry" >2D/3D Geometry
 * tutorial</a> in the wiki.
 *
 *  <div align=center>
 *   <img src="CPoint3D.gif">
 *  </div>
 *
 * \ingroup poses_grp
 * \sa CPoseOrPoint,CPose, CPoint
 */
class CPoint3D : public CPoint<CPoint3D>,
				 public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CPoint3D)

   public:
	/** [x,y,z] */
	mrpt::math::CArrayDouble<3> m_coords;

   public:
	/** Constructor for initializing point coordinates. */
	inline CPoint3D(const double x = 0, const double y = 0, const double z = 0)
	{
		m_coords[0] = x;
		m_coords[1] = y;
		m_coords[2] = z;
	}

	/** Constructor from a XYZ 3-vector */
	explicit inline CPoint3D(const mrpt::math::CArrayDouble<3>& xyz)
		: m_coords(xyz)
	{
	}

	/** Constructor from an CPoint2D object. */
	explicit CPoint3D(const CPoint2D& p);

	/** Constructor from an CPose3D object. */
	explicit CPoint3D(const CPose3D& p);

	/** Constructor from an CPose2D object. */
	explicit CPoint3D(const CPose2D& p);

	/** Constructor from lightweight object. */
	inline explicit CPoint3D(const mrpt::math::TPoint3D& p)
	{
		m_coords[0] = p.x;
		m_coords[1] = p.y;
		m_coords[2] = p.z;
	}
	mrpt::math::TPoint3D asTPoint() const;

	/** Returns this point as seen from "b", i.e. result = this - b */
	CPoint3D operator-(const CPose3D& b) const;

	/** Returns this point minus point "b", i.e. result = this - b */
	CPoint3D operator-(const CPoint3D& b) const;

	/** Returns this point plus point "b", i.e. result = this + b */
	CPoint3D operator+(const CPoint3D& b) const;

	/** Returns this point plus pose "b", i.e. result = this + b  */
	CPose3D operator+(const CPose3D& b) const;

	enum
	{
		is_3D_val = 1
	};
	static constexpr bool is_3D() { return is_3D_val != 0; }
	enum
	{
		is_PDF_val = 0
	};
	static constexpr bool is_PDF() { return is_PDF_val != 0; }
	/** @name STL-like methods and typedefs
	   @{   */
	/** The type of the elements */
	using value_type = double;
	using reference = double&;
	using const_reference = const double&;
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;

	// size is constant
	enum
	{
		static_size = 3
	};
	static constexpr size_type size() { return static_size; }
	static constexpr bool empty() { return false; }
	static constexpr size_type max_size() { return static_size; }
	static inline void resize(const size_t n)
	{
		if (n != static_size)
			throw std::logic_error(format(
				"Try to change the size of CPoint3D to %u.",
				static_cast<unsigned>(n)));
	}
	/** Templatized serializeTo function */
	template <typename SCHEMA_CAPABLE>
	SCHEMA_CAPABLE serializeTo() const
	{
		SCHEMA_CAPABLE out;
		out["datatype"] = this->GetRuntimeClass()->className;
		out["version"] = 1;
		out["x"] = m_coords[0];
		out["y"] = m_coords[1];
		out["z"] = m_coords[2];
		return out;	
	}

	/** Templatized serializeFrom function 
	 * Serializes only if the datatype matched to className 
	*/
	template <typename SCHEMA_CAPABLE>
	void serializeFrom(SCHEMA_CAPABLE& in)
	{
		uint8_t version = in.get("version",0);
		if(in["datatype"] == this->GetRuntimeClass()->className)
		{
			switch(version)
			{
				case 1:
				{
					m_coords[0] = in["x"];
					m_coords[1] = in["y"];
					m_coords[2] = in["z"];
				}
				break;
				default:
					MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
			}
		}
	}
	/** @} */

	void setToNaN() override;

};  // End of class def.

}  // namespace mrpt::poses
#endif
