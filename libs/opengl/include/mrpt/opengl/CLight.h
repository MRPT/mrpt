/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/serialization/serialization_frwds.h>
namespace mrpt::opengl
{
/** Each of the possible lights of a 3D scene \sa COpenGLViewport
 * *IMPORTANT NOTE*: It's the user responsibility to define unique light IDs
 * for each light.
 *  The OpenGL standard only assures that valid IDs are 0,1,..7
 * Refer to standard OpenGL literature and tutorials for the meaning of each
 * field.
 */
struct CLight
{
	/** Default constructor, sets default values */
	CLight();

	void setPosition(float x, float y, float z, float w);
	void setDirection(float dx, float dy, float dz);

	/** OpenGL ID (typical range: 0-7) */
	uint8_t light_ID{0};

	float color_ambient[4];
	float color_diffuse[4];
	float color_specular[4];

	/** [x,y,z,w]: w=0 means directional light, w=1 means a light at a real 3D
	 * position. */
	float position[4];
	/** [x,y,z] */
	float direction[3];
	float constant_attenuation{1.f};
	float linear_attenuation{0.f};
	float quadratic_attenuation{0.f};
	float spot_exponent{0.f};
	float spot_cutoff{180.f};

	void writeToStream(mrpt::serialization::CArchive& out) const;
	void readFromStream(mrpt::serialization::CArchive& in);

	/** Define the light in the current OpenGL rendering context (users normally
	 * don't need to call this explicitly, it's done from within a \sa
	 * COpenGLViewport) */
	void sendToOpenGL() const;

	DECLARE_TTYPENAME_CLASSNAME(mrpt::opengl::CLight)
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::opengl::CLight& o);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::opengl::CLight& o);

}  // namespace mrpt::opengl
