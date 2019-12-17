/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
using _GLENUM = uint32_t;

/**
 * Objects of this class represent a generic openGL object without specific
 * geometric properties.
 * \ingroup mrpt_opengl_grp
 */
class COpenGLStandardObject : public CRenderizable
{
	DEFINE_SERIALIZABLE(COpenGLStandardObject, mrpt::opengl)
   protected:
	/**
	 * OpenGL identifier of the object type.
	 */
	_GLENUM type{0};
	/**
	 * Set of points in which consists this object.
	 */
	std::vector<mrpt::math::TPoint3D> vertices;
	/**
	 * Granularity of the openGL elements. 3 for GL_TRIANGLES, 4 for GL_QUADS,
	 * and so on. Setting it to 0 will generate a single openGL object.
	 */
	uint32_t chunkSize{0};
	/**
	 * Set of openGL properties enabled in the rendering of this object.
	 */
	std::vector<_GLENUM> enabled;
	float normal[3];

   public:
	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;
	/**
	 * Ray Tracing. Will always return false, since objects of this class are
	 * not intended to have geometric properties.
	 * \sa mrpt::opengl::CRenderizable
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/**
	 * Enable some openGL flag.
	 */
	inline void enable(_GLENUM flag)
	{
		if (find(enabled.begin(), enabled.end(), flag) == enabled.end())
			enabled.push_back(flag);
		CRenderizable::notifyChange();
	}
	/**
	 * Disable some openGL flag.
	 */
	void disable(_GLENUM flag);
	/**
	 * Check whether an openGL will be enabled during the rendering of this
	 * object.
	 */
	inline bool isEnabled(_GLENUM flag) const
	{
		return find(enabled.begin(), enabled.end(), flag) != enabled.end();
	}
	/**
	 * Get a list of all currently enabled openGL flags.
	 */
	inline void getEnabledFlags(std::vector<_GLENUM>& v) const { v = enabled; }
	/**
	 * Set the list of all openGL flags.
	 */
	inline void setFlags(const std::vector<_GLENUM>& v)
	{
		enabled = v;
		CRenderizable::notifyChange();
	}
	/**
	 * Set the normal vector to this object.
	 */
	inline void setNormal(const float (&n)[3])
	{
		for (size_t i = 0; i < 3; i++) normal[i] = n[i];
		CRenderizable::notifyChange();
	}
	/**
	 * Gets the normal vector to this object.
	 */
	inline void getNormal(float (&n)[3]) const
	{
		for (size_t i = 0; i < 3; i++) n[i] = normal[i];
	}
	/**
	 * Creation of object from type, vertices, chunk size and a list of enabled
	 * openGL flags.
	 * \throw std::logic_error if the number of vertices is not an exact
	 * multiple of the chunk size.
	 */
	COpenGLStandardObject(
		_GLENUM t, const std::vector<mrpt::math::TPoint3D>& v, uint32_t cs,
		const std::vector<_GLENUM>& en)
		: type(t), vertices(v), chunkSize(cs), enabled(en)
	{
		for (float& i : normal) i = 0.0;
	}
	/**
	 * Baic empty constructor, initializes to default.
	 */
	COpenGLStandardObject()
		: vertices(std::vector<mrpt::math::TPoint3D>(0)),

		  enabled(std::vector<_GLENUM>())
	{
		for (float& i : normal) i = 0.0;
	}
	/**
	 * Destructor.
	 */
	~COpenGLStandardObject() override = default;
};
}  // namespace mrpt::opengl
