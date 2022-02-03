/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>

#include <deque>

namespace mrpt::opengl
{
/** The base class of 3D objects that can be directly rendered through OpenGL.
 *  In this class there are a set of common properties to all 3D objects,
 *mainly:
 * - A name (m_name): A name that can be optionally asigned to objects for
 *easing its reference.
 * - 6D coordinates (x,y,z,yaw,pitch,roll), relative to the "current"
 *reference framework. By default, any object is referenced to global scene
 *coordinates.
 * - A RGB color: This field will be used in simple elements (points,
 *lines, text,...) but is ignored in more complex objects that carry their own
 *color information (triangle sets,...)
 *
 * See the main class opengl::COpenGLScene
 *
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
	inline float getColorR() const { return u8tof(m_color.R); }
	/** Color components in the range [0,1] */
	inline float getColorG() const { return u8tof(m_color.G); }
	/** Color components in the range [0,1] */
	inline float getColorB() const { return u8tof(m_color.B); }
	/** Color components in the range [0,1] */
	inline float getColorA() const { return u8tof(m_color.A); }
	/** Color components in the range [0,255] */
	inline uint8_t getColorR_u8() const { return m_color.R; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorG_u8() const { return m_color.G; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorB_u8() const { return m_color.B; }
	/** Color components in the range [0,255] */
	inline uint8_t getColorA_u8() const { return m_color.A; }
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorR(const float r) { return setColorR_u8(f2u8(r)); }
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorG(const float g) { return setColorG_u8(f2u8(g)); }
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorB(const float b) { return setColorB_u8(f2u8(b)); }
	/**Color components in the range [0,1] \return a ref to this */
	CRenderizable& setColorA(const float a) { return setColorA_u8(f2u8(a)); }
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
		return setColor_u8(
			mrpt::img::TColor(f2u8(c.R), f2u8(c.G), f2u8(c.B), f2u8(c.A)));
	}

	/** Set the color components of this object (R,G,B,Alpha, in the range 0-1)
	 * \return a ref to this */
	inline CRenderizable& setColor(float R, float G, float B, float A = 1)
	{
		return setColor_u8(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
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

	/** Used from COpenGLScene::asYAML().
	 * \note (New in MRPT 2.4.2) */
	virtual void toYAMLMap(mrpt::containers::yaml& propertiesMap) const;

	/** Default constructor:  */
	CRenderizable();
	~CRenderizable() override;

	/** Context for calls to render() */
	struct RenderContext
	{
		RenderContext() = default;

		const mrpt::opengl::TRenderMatrices* state = nullptr;
		const mrpt::opengl::Program* shader = nullptr;
		mrpt::opengl::shader_id_t shader_id;
		const mrpt::opengl::TLightParameters* lights = nullptr;
	};

	/** Implements the rendering of 3D objects in each class derived from
	 * CRenderizable. This can be called more than once (one per required shader
	 * program) if the object registered several shaders. \sa
	 * renderUpdateBuffers
	 */
	virtual void render(const RenderContext& rc) const = 0;

	/** Process all children objects recursively, if the object is a container
	 */
	virtual void enqueForRenderRecursive(
		[[maybe_unused]] const mrpt::opengl::TRenderMatrices& state,
		[[maybe_unused]] RenderQueue& rq) const
	{
		// do thing
	}

	/** Called whenever m_outdatedBuffers is true: used to re-generate
	 * OpenGL vertex buffers, etc. before they are sent for rendering in
	 * render() */
	virtual void renderUpdateBuffers() const = 0;

	/** Returns the ID of the OpenGL shader program required to render this
	 * class. \sa DefaultShaderID
	 */
	virtual shader_list_t requiredShaders() const
	{
		THROW_EXCEPTION("Not implemented in derived class!");
	}

	/** Calls renderUpdateBuffers() and clear the flag that is set with
	 * notifyChange() */
	void updateBuffers() const
	{
		renderUpdateBuffers();
		const_cast<CRenderizable&>(*this).m_outdatedBuffers = false;
	}

	/** Call to enable calling renderUpdateBuffers() before the next
	 * render() rendering iteration. */
	void notifyChange() const
	{
		const_cast<CRenderizable&>(*this).m_outdatedBuffers = true;
	}

	/** Returns whether notifyChange() has been invoked since the last call
	 * to renderUpdateBuffers(), meaning the latter needs to be called again
	 * before rendering.
	 */
	bool hasToUpdateBuffers() const { return m_outdatedBuffers; }

	/** Simulation of ray-trace, given a pose. Returns true if the ray
	 * effectively collisions with the object (returning the distance to the
	 * origin of the ray in "dist"), or false in other case. "dist" variable
	 * yields undefined behaviour when false is returned
	 */
	virtual bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const;

	/** Evaluates the bounding box of this object (including possible
	 * children) in the coordinate frame of the object parent. */
	virtual auto getBoundingBox() const -> mrpt::math::TBoundingBox = 0;

	[[deprecated(
		"Use getBoundingBox() const -> mrpt::math::TBoundingBox instead.")]]  //
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
	{
		const auto bb = getBoundingBox();
		bb_min = bb.min;
		bb_max = bb.max;
	}

	/** Provide a representative point (in object local coordinates), used to
	 * sort objects by eye-distance while rendering with transparencies
	 * (Default=[0,0,0]) */
	virtual mrpt::math::TPoint3Df getLocalRepresentativePoint() const
	{
		return m_representativePoint;
	}

	/** See getLocalRepresentativePoint() */
	void setLocalRepresentativePoint(const mrpt::math::TPoint3Df& p)
	{
		m_representativePoint = p;
	}

	/** Returns or constructs (in its first invokation) the associated
	 * mrpt::opengl::CText object representing the label of the object.
	 * \sa enableShowName()
	 */
	mrpt::opengl::CText& labelObject() const;

	/** Free opengl buffers */
	virtual void freeOpenGLResources() = 0;

	/** Initializes all textures (loads them into opengl memory). */
	virtual void initializeTextures() const {}

   protected:
	void writeToStreamRender(mrpt::serialization::CArchive& out) const;
	void readFromStreamRender(mrpt::serialization::CArchive& in);

	bool m_outdatedBuffers = true;
	mrpt::math::TPoint3Df m_representativePoint{0, 0, 0};

	/** Optional pointer to a mrpt::opengl::CText */
	mutable std::shared_ptr<mrpt::opengl::CText> m_label_obj;
};

/** A list of smart pointers to renderizable objects */
using CListOpenGLObjects = std::deque<CRenderizable::Ptr>;

/** Enum for cull face modes in triangle-based shaders.
 *  \sa CRenderizableShaderTriangles, CRenderizableShaderTexturedTriangles
 *  \ingroup mrpt_opengl_grp
 */
enum class TCullFace : uint8_t
{
	/** The default: culls none, so all front and back faces are visible. */
	NONE = 0,
	/** Skip back faces (those that are NOT seen in the CCW direction) */
	BACK,
	/** Skip front faces (those that ARE seen in the CCW direction) */
	FRONT
};

/** @name Miscellaneous rendering methods
@{ */

/** Processes, recursively, all objects in the list, classifying them by shader
 * programs into a list suitable to be used within processPendingRendering()
 *
 * For each object in the list:
 *   - checks visibility of each object
 *   - update the MODELVIEW matrix according to its coordinates
 *   - call its ::render()
 *   - shows its name (if enabled).
 *
 * \note Used by CSetOfObjects and COpenGLViewport
 *
 * \sa processPendingRendering
 */
void enqueForRendering(
	const mrpt::opengl::CListOpenGLObjects& objs,
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq);

/** After enqueForRendering(), actually executes the rendering tasks, grouped
 * shader by shader.
 *
 *  \note Used by COpenGLViewport
 */
void processRenderQueue(
	const RenderQueue& rq,
	std::map<shader_id_t, mrpt::opengl::Program::Ptr>& shaders,
	const mrpt::opengl::TLightParameters& lights);

/** @} */

}  // namespace mrpt::opengl

MRPT_ENUM_TYPE_BEGIN(mrpt::opengl::TCullFace)
using namespace mrpt::opengl;
MRPT_FILL_ENUM_MEMBER(TCullFace, NONE);
MRPT_FILL_ENUM_MEMBER(TCullFace, BACK);
MRPT_FILL_ENUM_MEMBER(TCullFace, FRONT);
MRPT_ENUM_TYPE_END()
