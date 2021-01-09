/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPoseOrPoint.h>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/**
 * Standard object for storing any 3D lightweight object. Do not inherit from
 * this class.
 * \sa TPoint3D,TSegment3D,TLine3D,TPlane,TPolygon3D
 */
struct TObject3D
{
   private:
	/**
	 * Object type identifier.
	 */
	unsigned char type{GEOMETRIC_TYPE_UNDEFINED};
	/**
	 * Union containing pointer to actual data.
	 */
	struct tobject3d_data_t
	{
		TPoint3D point;
		TSegment3D segment;
		TLine3D line;
		TPolygon3D* polygon{nullptr};
		TPlane plane;

		tobject3d_data_t() = default;
	} data;
	/**
	 * Destroys the object and releases the pointer, if any.
	 */
	void destroy()
	{
		if (type == GEOMETRIC_TYPE_POLYGON) delete data.polygon;
		type = GEOMETRIC_TYPE_UNDEFINED;
	}

   public:
	/**
	 * Constructor from point.
	 */
	TObject3D(const TPoint3D& p) : type(GEOMETRIC_TYPE_POINT)
	{
		data.point = p;
	}
	/**
	 * Constructor from segment.
	 */
	TObject3D(const TSegment3D& s) : type(GEOMETRIC_TYPE_SEGMENT)
	{
		data.segment = s;
	}
	/**
	 * Constructor from line.
	 */
	TObject3D(const TLine3D& r) : type(GEOMETRIC_TYPE_LINE) { data.line = r; }
	/**
	 * Constructor from polygon.
	 */
	TObject3D(const TPolygon3D& p) : type(GEOMETRIC_TYPE_POLYGON)
	{
		data.polygon = new TPolygon3D(p);
	}
	/**
	 * Constructor from plane.
	 */
	TObject3D(const TPlane& p) : type(GEOMETRIC_TYPE_PLANE) { data.plane = p; }
	/**
	 * Empty constructor.
	 */
	TObject3D() : type(GEOMETRIC_TYPE_UNDEFINED) {}
	/**
	 * Destructor.
	 */
	~TObject3D() { destroy(); }
	/**
	 * Checks whether content is a point.
	 */
	bool isPoint() const { return type == GEOMETRIC_TYPE_POINT; }
	/**
	 * Checks whether content is a segment.
	 */
	bool isSegment() const { return type == GEOMETRIC_TYPE_SEGMENT; }
	/**
	 * Checks whether content is a line.
	 */
	bool isLine() const { return type == GEOMETRIC_TYPE_LINE; }
	/**
	 * Checks whether content is a polygon.
	 */
	bool isPolygon() const { return type == GEOMETRIC_TYPE_POLYGON; }
	/**
	 * Checks whether content is a plane.
	 */
	bool isPlane() const { return type == GEOMETRIC_TYPE_PLANE; }
	/**
	 * Gets object type.
	 */
	unsigned char getType() const { return type; }
	/**
	 * Gets the content as a point, returning false if the type is not
	 * adequate.
	 */
	bool getPoint(TPoint3D& p) const
	{
		if (isPoint())
		{
			p = data.point;
			return true;
		}
		else
			return false;
	}
	/**
	 * Gets the content as a segment, returning false if the type is not
	 * adequate.
	 */
	bool getSegment(TSegment3D& s) const
	{
		if (isSegment())
		{
			s = data.segment;
			return true;
		}
		else
			return false;
	}
	/**
	 * Gets the content as a line, returning false if the type is not adequate.
	 */
	bool getLine(TLine3D& r) const
	{
		if (isLine())
		{
			r = data.line;
			return true;
		}
		else
			return false;
	}
	/**
	 * Gets the content as a polygon, returning false if the type is not
	 * adequate.
	 */
	bool getPolygon(TPolygon3D& p) const
	{
		if (isPolygon())
		{
			p = *(data.polygon);
			return true;
		}
		else
			return false;
	}
	/**
	 * Gets the content as a plane, returning false if the type is not
	 * adequate.
	 */
	bool getPlane(TPlane& p) const
	{
		if (isPlane())
		{
			p = data.plane;
			return true;
		}
		else
			return false;
	}
	/**
	 * Assigns another object, creating a new pointer if needed.
	 */
	TObject3D& operator=(const TObject3D& obj)
	{
		if (this == &obj) return *this;
		destroy();
		switch (type = obj.type)
		{
			case GEOMETRIC_TYPE_POINT:
				data.point = obj.data.point;
				break;
			case GEOMETRIC_TYPE_SEGMENT:
				data.segment = obj.data.segment;
				break;
			case GEOMETRIC_TYPE_LINE:
				data.line = obj.data.line;
				break;
			case GEOMETRIC_TYPE_POLYGON:
				data.polygon = new TPolygon3D(*(obj.data.polygon));
				break;
			case GEOMETRIC_TYPE_PLANE:
				data.plane = obj.data.plane;
				break;
			case GEOMETRIC_TYPE_UNDEFINED:
				break;
			default:
				THROW_EXCEPTION("Invalid TObject3D object");
		}
		return *this;
	}
	/**
	 * Assigns a point to this object.
	 */
	void operator=(const TPoint3D& p)
	{
		destroy();
		type = GEOMETRIC_TYPE_POINT;
		data.point = p;
	}
	/**
	 * Assigns a segment to this object.
	 */
	void operator=(const TSegment3D& s)
	{
		destroy();
		type = GEOMETRIC_TYPE_SEGMENT;
		data.segment = s;
	}
	/**
	 * Assigns a line to this object.
	 */
	void operator=(const TLine3D& l)
	{
		destroy();
		type = GEOMETRIC_TYPE_LINE;
		data.line = l;
	}
	/**
	 * Assigns a polygon to this object.
	 */
	void operator=(const TPolygon3D& p)
	{
		destroy();
		type = GEOMETRIC_TYPE_POLYGON;
		data.polygon = new TPolygon3D(p);
	}
	/**
	 * Assigns a plane to this object.
	 */
	void operator=(const TPlane& p)
	{
		destroy();
		type = GEOMETRIC_TYPE_PLANE;
		data.plane = p;
	}
	/**
	 * Projects into 2D space.
	 * \throw std::logic_error if the 3D object loses its properties when
	 * projecting into 2D space (for example, it's a plane or a vertical line).
	 */
	void generate2DObject(TObject2D& obj) const;
	/**
	 * Constructs from another object.
	 */
	TObject3D(const TObject3D& obj) : type(GEOMETRIC_TYPE_UNDEFINED)
	{
		operator=(obj);
	}
	/**
	 * Static method to retrieve every point included in a vector of objects.
	 */
	static void getPoints(
		const std::vector<TObject3D>& objs, std::vector<TPoint3D>& pnts);
	/**
	 * Static method to retrieve every segment included in a vector of objects.
	 */
	static void getSegments(
		const std::vector<TObject3D>& objs, std::vector<TSegment3D>& sgms);
	/**
	 * Static method to retrieve every line included in a vector of objects.
	 */
	static void getLines(
		const std::vector<TObject3D>& objs, std::vector<TLine3D>& lins);
	/**
	 * Static method to retrieve every plane included in a vector of objects.
	 */
	static void getPlanes(
		const std::vector<TObject3D>& objs, std::vector<TPlane>& plns);
	/**
	 * Static method to retrieve every polygon included in a vector of objects.
	 */
	static void getPolygons(
		const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys);
	/**
	 * Static method to retrieve every point included in a vector of objects,
	 * returning the remaining objects in another argument.
	 */
	static void getPoints(
		const std::vector<TObject3D>& objs, std::vector<TPoint3D>& pnts,
		std::vector<TObject3D>& remainder);
	/**
	 * Static method to retrieve every segment included in a vector of objects,
	 * returning the remaining objects in another argument.
	 */
	static void getSegments(
		const std::vector<TObject3D>& objs, std::vector<TSegment3D>& sgms,
		std::vector<TObject3D>& remainder);
	/**
	 * Static method to retrieve every line included in a vector of objects,
	 * returning the remaining objects in another argument.
	 */
	static void getLines(
		const std::vector<TObject3D>& objs, std::vector<TLine3D>& lins,
		std::vector<TObject3D>& remainder);
	/**
	 * Static method to retrieve every plane included in a vector of objects,
	 * returning the remaining objects in another argument.
	 */
	static void getPlanes(
		const std::vector<TObject3D>& objs, std::vector<TPlane>& plns,
		std::vector<TObject3D>& remainder);
	/**
	 * Static method to retrieve every polygon included in a vector of objects,
	 * returning the remaining objects in another argument.
	 */
	static void getPolygons(
		const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys,
		std::vector<TObject3D>& remainder);
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject3D& o);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject3D& o);

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TObject3D, mrpt::math)

}  // namespace mrpt::typemeta
