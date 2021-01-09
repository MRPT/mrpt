/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/TSegment2D.h>

namespace mrpt::math
{
/** \addtogroup  geometry_grp
 * @{ */

/**
 * Standard type for storing any lightweight 2D type. Do not inherit from this
 * class.
 * \sa TPoint2D,TSegment2D,TLine2D,TPolygon2D
 */
struct TObject2D
{
   private:
	/**
	 * Object type identifier.
	 */
	unsigned char type{GEOMETRIC_TYPE_UNDEFINED};
	/**
	 * Union type storing pointers to every allowed type.
	 */
	struct tobject2d_data_t
	{
		TPoint2D point;
		TSegment2D segment;
		TLine2D line;
		TPolygon2D* polygon{nullptr};

		tobject2d_data_t() = default;
	} data;
	/**
	 * Destroys the object, releasing the pointer to the content (if any).
	 */
	void destroy()
	{
		if (type == GEOMETRIC_TYPE_POLYGON) delete data.polygon;
		type = GEOMETRIC_TYPE_UNDEFINED;
	}

   public:
	/**
	 * Implicit constructor from point.
	 */
	TObject2D(const TPoint2D& p) : type(GEOMETRIC_TYPE_POINT)
	{
		data.point = p;
	}
	/**
	 * Implicit constructor from segment.
	 */
	TObject2D(const TSegment2D& s) : type(GEOMETRIC_TYPE_SEGMENT)
	{
		data.segment = s;
	}
	/**
	 * Implicit constructor from line.
	 */
	TObject2D(const TLine2D& r) : type(GEOMETRIC_TYPE_LINE) { data.line = r; }
	/**
	 * Implicit constructor from polygon.
	 */
	TObject2D(const TPolygon2D& p) : type(GEOMETRIC_TYPE_POLYGON)
	{
		data.polygon = new TPolygon2D(p);
	}
	/**
	 * Implicit constructor from polygon.
	 */
	TObject2D() : type(GEOMETRIC_TYPE_UNDEFINED) {}
	/**
	 * Object destruction.
	 */
	~TObject2D() { destroy(); }
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
	 * Gets content type.
	 */
	unsigned char getType() const { return type; }
	/**
	 * Gets the content as a point, returning false if the type is inadequate.
	 */
	bool getPoint(TPoint2D& p) const
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
	 * Gets the content as a segment, returning false if the type is
	 * inadequate.
	 */
	bool getSegment(TSegment2D& s) const
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
	 * Gets the content as a line, returning false if the type is inadequate.
	 */
	bool getLine(TLine2D& r) const
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
	 * Gets the content as a polygon, returning false if the type is
	 * inadequate.
	 */
	bool getPolygon(TPolygon2D& p) const
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
	 * Assign another TObject2D. Pointers are not shared.
	 */
	TObject2D& operator=(const TObject2D& obj)
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
				data.polygon = new TPolygon2D(*(obj.data.polygon));
				break;
		}
		return *this;
	}
	/**
	 * Assign a point to this object.
	 */
	void operator=(const TPoint2D& p)
	{
		destroy();
		type = GEOMETRIC_TYPE_POINT;
		data.point = p;
	}
	/**
	 * Assign a segment to this object.
	 */
	void operator=(const TSegment2D& s)
	{
		destroy();
		type = GEOMETRIC_TYPE_SEGMENT;
		data.segment = s;
	}
	/**
	 * Assign a line to this object.
	 */
	void operator=(const TLine2D& l)
	{
		destroy();
		type = GEOMETRIC_TYPE_LINE;
		data.line = l;
	}
	/**
	 * Assign a polygon to this object.
	 */
	void operator=(const TPolygon2D& p)
	{
		destroy();
		type = GEOMETRIC_TYPE_POLYGON;
		data.polygon = new TPolygon2D(p);
	}
	/**
	 * Project into 3D space.
	 */
	void generate3DObject(TObject3D& obj) const;
	/**
	 * Constructor from another TObject2D.
	 */
	TObject2D(const TObject2D& obj) : type(GEOMETRIC_TYPE_UNDEFINED)
	{
		operator=(obj);
	}
	/**
	 * Static method to retrieve all the points in a vector of TObject2D.
	 */
	static void getPoints(
		const std::vector<TObject2D>& objs, std::vector<TPoint2D>& pnts);
	/**
	 * Static method to retrieve all the segments in a vector of TObject2D.
	 */
	static void getSegments(
		const std::vector<TObject2D>& objs, std::vector<TSegment2D>& sgms);
	/**
	 * Static method to retrieve all the lines in a vector of TObject2D.
	 */
	static void getLines(
		const std::vector<TObject2D>& objs, std::vector<TLine2D>& lins);
	/**
	 * Static method to retrieve all the polygons in a vector of TObject2D.
	 */
	static void getPolygons(
		const std::vector<TObject2D>& objs, std::vector<TPolygon2D>& polys);
	/**
	 * Static method to retrieve all the points in a vector of TObject2D,
	 * returning the remainder objects in another parameter.
	 */
	static void getPoints(
		const std::vector<TObject2D>& objs, std::vector<TPoint2D>& pnts,
		std::vector<TObject2D>& remainder);
	/**
	 * Static method to retrieve all the segments in a vector of TObject2D,
	 * returning the remainder objects in another parameter.
	 */
	static void getSegments(
		const std::vector<TObject2D>& objs, std::vector<TSegment2D>& sgms,
		std::vector<TObject2D>& remainder);
	/**
	 * Static method to retrieve all the lines in a vector of TObject2D,
	 * returning the remainder objects in another parameter.
	 */
	static void getLines(
		const std::vector<TObject2D>& objs, std::vector<TLine2D>& lins,
		std::vector<TObject2D>& remainder);
	/**
	 * Static method to retrieve all the polygons in a vector of TObject2D,
	 * returning the remainder objects in another parameter.
	 */
	static void getPolygons(
		const std::vector<TObject2D>& objs, std::vector<TPolygon2D>& polys,
		std::vector<TObject2D>& remainder);
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TObject2D& o);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TObject2D& o);

/** @} */

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TObject2D, mrpt::math)
}  // namespace mrpt::typemeta
