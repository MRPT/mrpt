/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/opengl_api.h>

#include <regex>

using namespace mrpt::opengl;

// TODO: May be, allow users to register custom shaders?

Program::Ptr mrpt::opengl::LoadDefaultShader(const shader_id_t id)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

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
			uniforms = {
				"p_matrix",
				"mv_matrix",
				"vertexPointSize",
				"enableVariablePointSize",
				"variablePointSize_K",
				"variablePointSize_DepthScale"};
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
		case DefaultShaderID::TRIANGLES_LIGHT:
			vertex_shader =
#include "../shaders/triangles-light.v.glsl"
				;
			fragment_shader =
#include "../shaders/triangles-light.f.glsl"
				;
			uniforms = {"p_matrix",		 "v_matrix",	  "m_matrix",
						"light_diffuse", "light_ambient", "light_direction"};
			attribs = {"position", "vertexColor", "vertexNormal"};
			break;
			// ==============================
		case DefaultShaderID::TRIANGLES_NO_LIGHT:
			vertex_shader =
#include "../shaders/triangles-no-light.v.glsl"
				;
			fragment_shader =
#include "../shaders/triangles-no-light.f.glsl"
				;
			uniforms = {"pmv_matrix"};
			attribs = {"position", "vertexColor"};
			break;
			// ==============================
		case DefaultShaderID::TEXTURED_TRIANGLES_LIGHT:
			vertex_shader =
#include "../shaders/textured-triangles-light.v.glsl"
				;
			fragment_shader =
#include "../shaders/textured-triangles-light.f.glsl"
				;
			uniforms = {"p_matrix",		 "v_matrix",	  "m_matrix",
						"light_diffuse", "light_ambient", "light_direction",
						"textureSampler"};
			attribs = {"position", "vertexUV", "vertexNormal"};
			break;
			// ==============================
		case DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT:
			vertex_shader =
#include "../shaders/textured-triangles-no-light.v.glsl"
				;
			fragment_shader =
#include "../shaders/textured-triangles-no-light.f.glsl"
				;
			uniforms = {"pmv_matrix", "textureSampler"};
			attribs = {"position", "vertexUV"};
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
			attribs = {"position", "vertexColor"};
			break;

		default:
			THROW_EXCEPTION_FMT(
				"Unknown shader_id_t=%u", static_cast<unsigned>(id));
	};

			// Init GLEW if not already done:
#ifdef _WIN32
	glewInit();
#endif

#if defined(__EMSCRIPTEN__)
	// in emscripten + GLES3 we need to insert this line after version:
	std::string sOrgV = vertex_shader;
	sOrgV = std::regex_replace(
		sOrgV, std::regex("#version 300 es"),
		"#version 300 es\r\nprecision mediump float;\r\n");

	vertex_shader = sOrgV.c_str();

	std::string sOrgF = fragment_shader;
	sOrgF = std::regex_replace(
		sOrgF, std::regex("#version 300 es"),
		"#version 300 es\r\nprecision mediump float;\r\n");

	fragment_shader = sOrgF.c_str();

#endif

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

#if 0
	// Debug:
	std::cout << "Built Shader program #" << int(id) << "\n";
	shader->dumpProgramDescription(std::cout);
	std::cout << "\n";
#endif

	// Uniforms:
	for (const auto& name : uniforms)
		shader->declareUniform(name);

	// Attributes:
	for (const auto& name : attribs)
		shader->declareAttribute(name);

	return shader;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}
