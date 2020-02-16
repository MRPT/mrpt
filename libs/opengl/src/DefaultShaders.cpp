/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/DefaultShaders.h>
#include "opengl_internals.h"

using namespace mrpt::opengl;

// TODO: May be, allow users to register custom shaders?

Program::Ptr mrpt::opengl::LoadDefaultShader(const shader_id_t id)
{
#if MRPT_HAS_OPENGL_GLUT

	// Vertex shader:
	const char* vertex_shader = nullptr;
	const char* fragment_shader = nullptr;
	std::vector<std::string> attribs, uniforms;

	switch (id)
	{
		case DefaultShaderID::POINTS:
			vertex_shader =
#include "../shaders/points.v.glsl"
				;
			fragment_shader =
#include "../shaders/points.f.glsl"
				;
			uniforms = {"p_matrix", "mv_matrix", "vertexPointSize"};
			attribs = {"position", "vertexColor"};
			break;

			// ==============================
		case DefaultShaderID::WIREFRAME:
			vertex_shader =
#include "../shaders/wireframe.v.glsl"
				;
			fragment_shader =
#include "../shaders/wireframe.f.glsl"
				;
			uniforms = {"p_matrix", "mv_matrix"};
			attribs = {"position", "vertexColor"};
			break;

			// ==============================
		case DefaultShaderID::TRIANGLES:
			vertex_shader =
#include "../shaders/triangles.v.glsl"
				;
			fragment_shader =
#include "../shaders/triangles.f.glsl"
				;
			uniforms = {"p_matrix", "mv_matrix"};
			attribs = {"position", "vertexColor", "vertexNormal"};
			break;
			// ==============================
		case DefaultShaderID::TEXTURED_TRIANGLES:
			vertex_shader =
#include "../shaders/textured-triangles.v.glsl"
				;
			fragment_shader =
#include "../shaders/textured-triangles.f.glsl"
				;
			uniforms = {"p_matrix", "mv_matrix"};
			attribs = {"position", "vertexColor", "vertexNormal"};
			break;
			// ==============================
		case DefaultShaderID::TEXT:
			vertex_shader =
#include "../shaders/text.v.glsl"
				;
			fragment_shader =
#include "../shaders/text.f.glsl"
				;
			uniforms = {"p_matrix", "mv_matrix"};
			attribs = {"position", "vertexColor", "vertexNormal"};
			break;

		default:
			THROW_EXCEPTION_FMT(
				"Unknown shader_id_t=%u", static_cast<unsigned>(id));
	};

	auto shader = std::make_shared<Program>();

	std::string errMsgs;
	std::vector<Shader> lstShaders;
	lstShaders.resize(2);
	if (!lstShaders[0].compile(GL_VERTEX_SHADER, vertex_shader, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error compiling GL_VERTEX_SHADER:\n%s", errMsgs.c_str());
	}
	if (!lstShaders[1].compile(GL_FRAGMENT_SHADER, fragment_shader, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error compiling GL_FRAGMENT_SHADER:\n%s", errMsgs.c_str());
	}
	if (!shader->linkProgram(lstShaders, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error linking Opengl Shader programs:\n%s", errMsgs.c_str());
	}

	// Uniforms:
	for (const auto& name : uniforms) shader->declareUniform(name);

	// Attributes:
	for (const auto& name : attribs) shader->declareAttribute(name);

	return shader;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}
