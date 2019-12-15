/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <deque>

namespace mrpt::opengl
{
// Frwd decls:
class COpenGLViewport;
class CSetOfObjects;
namespace gl_utils
{
void checkOpenGLErr_impl(const char* filename, int lineno);
}

/** Checks glGetError and throws an exception if an error situation is found
 */
#define CHECK_OPENGL_ERROR() \
	mrpt::opengl::gl_utils::checkOpenGLErr_impl(__FILE__, __LINE__);

/** The base class of 3D objects that can be directly rendered through OpenGL.
 *  In this class there are a set of common properties to all 3D objects,
 *mainly:
 *		- A name (m_name): A name that can be optionally asigned to objects for
 *easing its reference.
 *		- 6D coordinates (x,y,z,yaw,pitch,roll), relative to the "current"
 *reference framework. By default, any object is referenced to global scene
 *coordinates.
 *		- A RGB color: This field will be used in simple elements (points,
 *lines,
 *text,...) but is ignored in more complex objects that carry their own color
 *information (triangle sets,...)
 *  See the main class opengl::COpenGLScene
 *  \sa opengl::COpenGLScene, mrpt::opengl
 * \ingroup mrpt_opengl_grp
 */
class CRenderizable : public mrpt::serialization::CSerializable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizable)

	friend class mrpt::opengl::COpenGLViewport;
	friend class mrpt::opengl::CSetOfObjects;

   protected:
	std::string m_name;
	bool m_show_name{false};
	/** Color components in the range [0,255] */
	mrpt::img::TColor m_color;
	/** 6D pose wrt the parent coordinate reference. This class automatically
	 * holds the cached 3x3 rotation matrix for quick load into opengl stack. */
	mrpt::poses::CPose3D m_pose;
	/** Scale components to apply to the object (default=1) */
	float m_scale_x{1}, m_scale_y{1}, m_scale_z{1};
	/** Is the object visible? (default=true) */
	bool m_visible{true};

   public:
	/** @name Changes the appearance of the object to render
		@{ */

	/** Changes the name of the object */
	void setName(const std::string& n) { m_name = n; }
	/** Returns the name of the object */
	const std::string& getName() const { return m_name; }
	inline bool isVisible()
		const /** Is the object visible? \sa setVisibility */
	{
		return m_visible;
	}
	inline void setVisibility(
		bool visible =
			true) /** Set object visibility (default=true) \sa isVisible */
	{
		m_visible = visible;
	}

	/** Enables or disables showing the name of the object as a label when
	 * rendering */
	inline void enableShowName(bool showName = true) { m_show_name = showName; }
	/** \sa enableShowName */
	inline bool isShowNameEnabled() const { return m_show_name; }
	/** Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to
	 * this) */
	CRenderizable& setPose(const mrpt::poses::CPose3D& o);
	/** Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to
	 * this) */
	CRenderizable& setPose(const mrpt::poses::CPose2D& o);
	/** Set the 3D pose from a  mrpt::math::TPose3D object (return a ref to
	 * this) */
	CRenderizable& setPose(const mrpt::math::TPose3D& o);
	/** Set the 3D pose from a  mrpt::math::TPose3D object (return a ref to
	 * this) */
	CRenderizable& setPose(const mrpt::math::TPose2D& o);
	/** Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to
	 * this) */
	CRenderizable& setPose(const mrpt::poses::CPoint3D& o);
	/** Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to
	 * this) */
	CRenderizable& setPose(const mrpt::poses::CPoint2D& o);

	/** Returns the 3D pose of the object as TPose3D */
	mrpt::math::TPose3D getPose() const;
	/** Returns a const ref to the 3D pose of the object as mrpt::poses::CPose3D
	 * (which explicitly contains the 3x3 rotation matrix) */
	inline const mrpt::poses::CPose3D& getPoseRef() const { return m_pose; }
	/** Changes the location of the object, keeping untouched the orientation
	 * \return a ref to this */
	inline CRenderizable& setLocation(double x, double y, double z)
	{
		m_pose.x(x);
		m_pose.y(y);
		m_pose.z(z);
		return *this;
	}

	/** Changes the location of the object, keeping untouched the orientation
	 * \return a ref to this  */
	inline CRenderizable& setLocation(const mrpt::math::TPoint3D& p)
	{
		m_pose.x(p.x);
		m_pose.y(p.y);
		m_pose.z(p.z);
		return *this;
	}

	/** Translation relative to parent coordinate origin. */
	inline double getPoseX() const { return m_pose.x(); }
	/** Translation relative to parent coordinate origin. */
	inline double getPoseY() const { return m_pose.y(); }
	/** Translation relative to parent coordinate origin. */
	inline double getPoseZ() const { return m_pose.z(); }
	/** Rotation relative to parent coordinate origin, in **DEGREES**. */
	inline double getPoseYaw() const { return mrpt::RAD2DEG(m_pose.yaw()); }
	/** Rotation relative to parent coordinate origin, in **DEGREES**. */
	inline double getPosePitch() const { return mrpt::RAD2DEG(m_pose.pitch()); }
	/** Rotation relative to parent coordinate origin, in **DEGREES**. */
	inline double getPoseRoll() const { return mrpt::RAD2DEG(m_pose.roll()); }
	/** Rotation relative to parent coordinate origin, in radians. */
	inline double getPoseYawRad() const { return m_pose.yaw(); }
	/** Rotation relative to parent coordinate origin, in radians. */
	inline double getPosePitchRad() const { return m_pose.pitch(); }
	/** Rotation relative to parent coordinate origin, in radians. */
	inline double getPoseRollRad() const { return m_pose.roll(); }
	/** Color components in the range [0,1] */
	inline double getColorR() const { return m_color.R / 255.; }
	/** Color components in the range [0,1] */
	inline double getColorG() const { return m_color.G / 255.; }
	/** Color components in the range [0,1] */
	inline double getColorB() const { return m_color.B / 255.; }
	/** Color components in the range [0,1] */
	inline double getColorA() const { return m_color.A / 255.; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorR_u8() const { return m_color.R; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorG_u8() const { return m_color.G; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorB_u8() const { return m_color.B; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorA_u8() const { return m_color.A; }
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorR(const double r)
	{
		return setColorR_u8(static_cast<uint8_t>(255 * r));
	}
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorG(const double g)
	{
		return setColorG_u8(static_cast<uint8_t>(255 * g));
	}
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorB(const double b)
	{
		return setColorB_u8(static_cast<uint8_t>(255 * b));
	}
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorA(const double a)
	{
		return setColorA_u8(static_cast<uint8_t>(255 * a));
	}
	/**Color components in the range [0,255] \return a ref to this */
	virtual CRenderizable& setColorR_u8(const uint8_t r)
	{
		m_color.R = r;
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	virtual CRenderizable& setColorG_u8(const uint8_t g)
	{
		m_color.G = g;
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	virtual CRenderizable& setColorB_u8(const uint8_t b)
	{
		m_color.B = b;
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	virtual CRenderizable& setColorA_u8(const uint8_t a)
	{
		m_color.A = a;
		return *this;
	}

	/** Scale to apply to the object, in all three axes (default=1)  \return a
	 * ref to this */
	inline CRenderizable& setScale(float s)
	{
		m_scale_x = m_scale_y = m_scale_z = s;
		return *this;
	}
	/** Scale to apply to the object in each axis (default=1)  \return a ref to
	 * this */
	inline CRenderizable& setScale(float sx, float sy, float sz)
	{
		m_scale_x = sx;
		m_scale_y = sy;
		m_scale_z = sz;
		return *this;
	}
	/** Get the current scaling factor in one axis */
	inline float getScaleX() const { return m_scale_x; }
	/** Get the current scaling factor in one axis */
	inline float getScaleY() const { return m_scale_y; }
	/** Get the current scaling factor in one axis */
	inline float getScaleZ() const { return m_scale_z; }
	/** Returns the object color property as a TColorf */
	inline mrpt::img::TColorf getColor() const
	{
		return mrpt::img::TColorf(m_color);
	}
	/** Changes the default object color \return a ref to this */
	CRenderizable& setColor(const mrpt::img::TColorf& c)
	{
		return setColor_u8(mrpt::img::TColor(
			c.R * 255.f, c.G * 255.f, c.B * 255.f, c.A * 255.f));
	}

	/** Set the color components of this object (R,G,B,Alpha, in the range 0-1)
	 * \return a ref to this */
	inline CRenderizable& setColor(double R, double G, double B, double A = 1)
	{
		return setColor_u8(R * 255, G * 255, B * 255, A * 255);
	}

	/** Returns the object color property as a TColor */
	inline const mrpt::img::TColor& getColor_u8() const { return m_color; }
	/*** Changes the default object color \return a ref to this */
	virtual CRenderizable& setColor_u8(const mrpt::img::TColor& c);

	/** Set the color components of this object (R,G,B,Alpha, in the range
	 * 0-255)  \return a ref to this */
	CRenderizable& setColor_u8(uint8_t R, uint8_t G, uint8_t B, uint8_t A = 255)
	{
		return setColor_u8(mrpt::img::TColor(R, G, B, A));
	}

	/** @} */

	/** Default constructor:  */
	CRenderizable();
	~CRenderizable() override;

	/** Implements the rendering of 3D objects in each class derived from
	 * CRenderizable.
	 */
	virtual void render() const = 0;

	/** Simulation of ray-trace, given a pose. Returns true if the ray
	 * effectively collisions with the object (returning the distance to the
	 * origin of the ray in "dist"), or false in other case. "dist" variable
	 * yields undefined behaviour when false is returned
	 */
	virtual bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const;

	/** This method is safe for calling from within ::render() methods \sa
	 * renderTextBitmap, mrpt::opengl::gl_utils */
	static void renderTextBitmap(const char* str, void* fontStyle);

	/** Return the exact width in pixels for a given string, as will be rendered
	 * by renderTextBitmap().
	 * \sa renderTextBitmap, mrpt::opengl::gl_utils
	 */
	static int textBitmapWidth(
		const std::string& str,
		mrpt::opengl::TOpenGLFont font =
			mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24);

	/** Render a text message in the current rendering context, creating a
	 * glViewport in the way (do not call within ::render() methods)
	 *   - Coordinates (x,y) are 2D pixels, starting at bottom-left of the
	 * viewport. Negative numbers will wrap to the opposite side of the viewport
	 * (e.g. x=-10 means 10px fromt the right).
	 *   - The text color is defined by (color_r,color_g,color_b), each float
	 * numbers in the range [0,1].
	 *  \sa renderTextBitmap, textBitmapWidth, mrpt::opengl::gl_utils
	 */
	static void renderTextBitmap(
		int screen_x, int screen_y, const std::string& str, float color_r = 1,
		float color_g = 1, float color_b = 1,
		mrpt::opengl::TOpenGLFont font =
			mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24);

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	virtual void getBoundingBox(
		mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const = 0;

   protected:
	void writeToStreamRender(mrpt::serialization::CArchive& out) const;
	void readFromStreamRender(mrpt::serialization::CArchive& in);

	/** Returns the lowest next free texture name (avoid using OpenGL's own
	 * function since we may call them from different threads and seem it's not
	 * cool).  */
	static unsigned int getNewTextureNumber();
	static void releaseTextureName(unsigned int i);
};
/** A list of objects pointers, automatically managing memory free at
 * destructor, and managing copies correctly. */
using CListOpenGLObjects = std::deque<CRenderizable::Ptr>;

/** Applies a mrpt::poses::CPose3D transformation to the object. Note that this
 * method doesn't <i>set</i> the pose to the given value, but <i>combines</i> it
 * with the existing one.
 * \sa setPose */
CRenderizable::Ptr& operator<<(
	CRenderizable::Ptr& r, const mrpt::poses::CPose3D& p);

}  // namespace mrpt::opengl
