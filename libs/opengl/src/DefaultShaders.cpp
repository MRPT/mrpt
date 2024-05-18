/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/opengl_api.h>

#include <regex>

using namespace mrpt::opengl;

Program::Ptr mrpt::opengl::LoadDefaultShader(const shader_id_t id)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  // Vertex shader:
  const char* vertex_shader = nullptr;
  const char* fragment_shader = nullptr;
  const char* fragShaderIncludes = nullptr;
  std::vector<std::string> attribs, uniforms;

  switch (id)
  {
    // ==============================
    // Regular geometric elements
    // ==============================
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

    case DefaultShaderID::TRIANGLES_LIGHT:
      vertex_shader =
#include "../shaders/triangles-light.v.glsl"
          ;
      fragment_shader =
#include "../shaders/triangles-light.f.glsl"
          ;
      uniforms = {"p_matrix",      "v_matrix",        "m_matrix",    "light_diffuse",
                  "light_ambient", "light_specular",  "light_color", "light_direction",
                  "cam_position",  "materialSpecular"};
      attribs = {"position", "vertexColor", "vertexNormal"};
      break;

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

    case DefaultShaderID::TEXTURED_TRIANGLES_LIGHT:
      vertex_shader =
#include "../shaders/textured-triangles-light.v.glsl"
          ;
      fragment_shader =
#include "../shaders/textured-triangles-light.f.glsl"
          ;
      uniforms = {"p_matrix",        "v_matrix",       "m_matrix",        "light_diffuse",
                  "light_ambient",   "light_specular", "light_color",     "cam_position",
                  "light_direction", "textureSampler", "materialSpecular"};
      attribs = {"position", "vertexUV", "vertexNormal"};
      break;

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

    // ===========================================
    // Shadow generation 1st/2nd pass shaders
    // ===========================================
    case DefaultShaderID::TRIANGLES_SHADOW_1ST:
      vertex_shader =
#include "../shaders/triangles-shadow-1st.v.glsl"
          ;
      fragment_shader =
#include "../shaders/triangles-shadow-1st.f.glsl"
          ;
      uniforms = {"m_matrix", "light_pv_matrix"};
      attribs = {"position"};
      break;

    case DefaultShaderID::TRIANGLES_SHADOW_2ND:
      vertex_shader =
#include "../shaders/triangles-shadow-2nd.v.glsl"
          ;
      fragment_shader =
#include "../shaders/triangles-shadow-2nd.f.glsl"
          ;
      fragShaderIncludes =
#include "../shaders/shadow-calculation.f.glsl"
          ;
      uniforms = {
          "p_matrix",
          "v_matrix",
          "m_matrix",
          "light_pv_matrix",
          "shadowMap",
          "light_diffuse",
          "light_ambient",
          "light_specular",
          "light_direction",
          "shadow_bias",
          "shadow_bias_cam2frag",
          "shadow_bias_normal",
          "light_color",
          "cam_position",
          "materialSpecular"};
      attribs = {"position", "vertexColor", "vertexNormal"};
      break;

      // 1st stage is the same for textured and non-textured triangles:
    case DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_1ST:
      vertex_shader =
#include "../shaders/triangles-shadow-1st.v.glsl"
          ;
      fragment_shader =
#include "../shaders/triangles-shadow-1st.f.glsl"
          ;
      uniforms = {"m_matrix", "light_pv_matrix"};
      attribs = {"position"};
      break;

    case DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND:
      vertex_shader =
#include "../shaders/textured-triangles-shadow-2nd.v.glsl"
          ;
      fragment_shader =
#include "../shaders/textured-triangles-shadow-2nd.f.glsl"
          ;
      fragShaderIncludes =
#include "../shaders/shadow-calculation.f.glsl"
          ;
      uniforms = {
          "p_matrix",
          "v_matrix",
          "m_matrix",
          "light_pv_matrix",
          "shadowMap",
          "light_diffuse",
          "light_ambient",
          "light_specular",
          "light_direction",
          "shadow_bias",
          "shadow_bias_cam2frag",
          "shadow_bias_normal",
          "light_color",
          "cam_position",
          "materialSpecular",
          "textureSampler"};
      attribs = {"position", "vertexNormal", "vertexUV"};
      break;

    // ===========================================
    // Special effects
    // ===========================================
    case DefaultShaderID::SKYBOX:
      vertex_shader =
#include "../shaders/skybox.v.glsl"
          ;
      fragment_shader =
#include "../shaders/skybox.f.glsl"
          ;
      uniforms = {"p_matrix", "v_matrix_no_translation", "skybox"};
      attribs = {"position"};
      break;

    case DefaultShaderID::DEBUG_TEXTURE_TO_SCREEN:
      vertex_shader =
#include "../shaders/debug_show_texture.v.glsl"
          ;
      fragment_shader =
#include "../shaders/debug_show_texture.f.glsl"
          ;
      uniforms = {"textureId"};
      attribs = {"position", "vertexUV"};
      break;

    default:
      THROW_EXCEPTION_FMT("Unknown shader_id_t=%u", static_cast<unsigned>(id));
  };

      // Init GLEW if not already done:
#ifdef _WIN32
  glewInit();
#endif

#if defined(__EMSCRIPTEN__)
  // in emscripten + GLES3 we need to insert this line after version:
  std::string sOrgV = vertex_shader;
  sOrgV = std::regex_replace(
      sOrgV, std::regex("#version 300 es"), "#version 300 es\r\nprecision mediump float;\r\n");

  vertex_shader = sOrgV.c_str();

  std::string sOrgF = fragment_shader;
  sOrgF = std::regex_replace(
      sOrgF, std::regex("#version 300 es"), "#version 300 es\r\nprecision mediump float;\r\n");

  fragment_shader = sOrgF.c_str();

#endif

  auto shader = std::make_shared<Program>();

  std::string errMsgs;
  std::vector<Shader> lstShaders;
  lstShaders.resize(2);
  if (!lstShaders[0].compile(GL_VERTEX_SHADER, {vertex_shader}, errMsgs))
  {
    THROW_EXCEPTION_FMT(
        "Error compiling GL_VERTEX_SHADER (%s):\n%s", vertex_shader, errMsgs.c_str());
  }
  if (!lstShaders[1].compile(
          GL_FRAGMENT_SHADER, {fragShaderIncludes ? fragShaderIncludes : "", fragment_shader},
          errMsgs))
  {
    THROW_EXCEPTION_FMT(
        "Error compiling GL_FRAGMENT_SHADER (%s):\n%s", fragment_shader, errMsgs.c_str());
  }
  if (!shader->linkProgram(lstShaders, errMsgs))
  {
    THROW_EXCEPTION_FMT(
        "Error linking Opengl Shader programs (%s, %s):\n%s", vertex_shader, fragment_shader,
        errMsgs.c_str());
  }

#if 0
	// Debug:
	std::cout << "Built Shader program #" << int(id) << "\n";
	shader->dumpProgramDescription(std::cout);
	std::cout << "\n";
#endif

  // Uniforms:
  for (const auto& name : uniforms)
  {
    try
    {
      shader->declareUniform(name);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception while declaring Uniform of shader ID #" << static_cast<int>(id)
                << std::endl;
      throw;
    }
  }

  // Attributes:
  for (const auto& name : attribs) shader->declareAttribute(name);

  return shader;
#else
  THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}
