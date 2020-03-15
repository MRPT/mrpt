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
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>

namespace mrpt::opengl
{
/** Lighting parameters, mostly for triangle shaders.
 * Refer to standard OpenGL literature and tutorials for the meaning of each
 * field, and to the shader GLSL code itself.
 */
struct TLightParameters
{
	TLightParameters() = default;
	~TLightParameters() = default;

	mrpt::img::TColorf diffuse = {0.8f, 0.8f, 0.8f, 0.0f};
	mrpt::img::TColorf ambient = {0.2f, 0.2f, 0.2f, 1.0f};
	mrpt::img::TColorf specular = {1.0f, 1.0f, 1.0f, 1.0f};

	/** Light direction (must be normalized) */
	mrpt::math::TVector3Df direction = {-0.40825f, -0.40825f, -0.81650f};

	void writeToStream(mrpt::serialization::CArchive& out) const;
	void readFromStream(mrpt::serialization::CArchive& in);

	DECLARE_TTYPENAME_CLASSNAME(mrpt::opengl::TLightParameters)
};

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::opengl::TLightParameters& o);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out,
	const mrpt::opengl::TLightParameters& o);

}  // namespace mrpt::opengl
