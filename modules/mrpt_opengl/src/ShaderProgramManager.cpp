/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/ShaderProgramManager.h>

#include <iostream>

using namespace mrpt::opengl;

static const bool SHADER_VERBOSE = mrpt::get_env<bool>("MRPT_SHADER_VERBOSE", false);

ShaderProgramManager::ShaderProgramManager()
{
  m_contextThread = std::this_thread::get_id();
  m_verbose = SHADER_VERBOSE;

  // Build shader variant mapping for shadow rendering
  buildShaderVariantMap();

  if (m_verbose)
  {
    std::cout << "[ShaderProgramManager] Created\n";
  }
}

ShaderProgramManager::~ShaderProgramManager()
{
  try
  {
    clear();
  }
  catch (const std::exception& e)
  {
    std::cerr << "[ShaderProgramManager::~ShaderProgramManager] Exception: " << e.what() << "\n";
  }
}

void ShaderProgramManager::checkThread() const
{
  if (std::this_thread::get_id() != m_contextThread)
  {
    THROW_EXCEPTION(
        "ShaderProgramManager operations must be called from the OpenGL "
        "context thread!");
  }
}

Program::Ptr ShaderProgramManager::getProgram(shader_id_t id)
{
  MRPT_START
  checkThread();

  // Check cache first
  auto it = m_builtinPrograms.find(id);
  if (it != m_builtinPrograms.end())
  {
    return it->second;
  }

  // Not cached - compile it now
  std::string errorMsg;
  auto program = compileBuiltinShader(id, &errorMsg);

  if (!program)
  {
    THROW_EXCEPTION_FMT(
        "Failed to compile built-in shader ID %d: %s", static_cast<int>(id), errorMsg.c_str());
  }

  // Cache it
  m_builtinPrograms[id] = program;

  if (m_verbose)
  {
    std::cout << "[ShaderProgramManager] Compiled shader ID " << static_cast<int>(id) << "\n";
  }

  return program;

  MRPT_END
}

bool ShaderProgramManager::hasProgram(shader_id_t id) const
{
  return m_builtinPrograms.find(id) != m_builtinPrograms.end();
}

size_t ShaderProgramManager::preloadAllDefaultShaders(std::vector<std::string>* outErrors)
{
  MRPT_START
  checkThread();

  size_t successCount = 0;

  // List of all default shader IDs to preload
  const std::vector<shader_id_t> allDefaultShaders = {
      DefaultShaderID::POINTS,
      DefaultShaderID::WIREFRAME,
      DefaultShaderID::TRIANGLES_NO_LIGHT,
      DefaultShaderID::TRIANGLES_LIGHT,
      DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT,
      DefaultShaderID::TEXTURED_TRIANGLES_LIGHT,
      DefaultShaderID::TEXT,
      DefaultShaderID::TRIANGLES_SHADOW_1ST,
      DefaultShaderID::TRIANGLES_SHADOW_2ND,
      DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_1ST,
      DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND,
      DefaultShaderID::SKYBOX,
      DefaultShaderID::DEBUG_TEXTURE_TO_SCREEN};

  for (auto id : allDefaultShaders)
  {
    std::string errorMsg;
    auto program = compileBuiltinShader(id, &errorMsg);

    if (program)
    {
      m_builtinPrograms[id] = program;
      successCount++;

      if (m_verbose)
      {
        std::cout << "[ShaderProgramManager] Preloaded shader ID " << static_cast<int>(id) << "\n";
      }
    }
    else
    {
      if (outErrors != nullptr)
      {
        outErrors->push_back(
            mrpt::format("Shader ID %d: %s", static_cast<int>(id), errorMsg.c_str()));
      }

      std::cerr << "[ShaderProgramManager] Failed to preload shader ID " << static_cast<int>(id)
                << ": " << errorMsg << "\n";
    }
  }

  return successCount;

  MRPT_END
}

Program::Ptr ShaderProgramManager::compileBuiltinShader(shader_id_t id, std::string* outErrorMsg)
{
  MRPT_START

  // Get shader source from DefaultShaders
  auto shaderDef = LoadDefaultShader(id);

  if (!shaderDef)
  {
    if (outErrorMsg != nullptr)
    {
      *outErrorMsg = "Unknown shader ID";
    }
    return nullptr;
  }

  // The shader is already a compiled Program from LoadDefaultShader
  return shaderDef;

  MRPT_END
}

Program::Ptr ShaderProgramManager::loadCustomProgram(
    const std::string& name,
    const std::string& vertexShaderSource,
    const std::string& fragmentShaderSource,
    const std::string& geometryShaderSource,
    std::string* outErrorMsg)
{
  MRPT_START
  checkThread();

  // Check if name already exists
  if (hasCustomProgram(name))
  {
    THROW_EXCEPTION_FMT("Custom shader with name '%s' already exists!", name.c_str());
  }

  // Compile the shader
  auto program = compileShaderProgram(
      vertexShaderSource, fragmentShaderSource, geometryShaderSource, name, outErrorMsg);

  if (!program)
  {
    return nullptr;
  }

  // Cache it
  m_customPrograms[name] = program;

  if (m_verbose)
  {
    std::cout << "[ShaderProgramManager] Loaded custom shader '" << name << "'\n";
  }

  return program;

  MRPT_END
}

Program::Ptr ShaderProgramManager::loadCustomProgramFromFiles(
    const std::string& name,
    const std::string& vertexShaderFile,
    const std::string& fragmentShaderFile,
    const std::string& geometryShaderFile,
    std::string* outErrorMsg)
{
  MRPT_START

  // Read vertex shader
  std::string vertexSrc;
  if (!mrpt::io::CFileInputStream::readFileIntoString(vertexShaderFile, vertexSrc))
  {
    if (outErrorMsg) *outErrorMsg = "Failed to read vertex shader file: " + vertexShaderFile;
    return nullptr;
  }

  // Read fragment shader
  std::string fragmentSrc;
  if (!mrpt::io::CFileInputStream::readFileIntoString(fragmentShaderFile, fragmentSrc))
  {
    if (outErrorMsg) *outErrorMsg = "Failed to read fragment shader file: " + fragmentShaderFile;
    return nullptr;
  }

  // Read geometry shader (optional)
  std::string geometrySrc;
  if (!geometryShaderFile.empty())
  {
    if (!mrpt::io::CFileInputStream::readFileIntoString(geometryShaderFile, geometrySrc))
    {
      if (outErrorMsg) *outErrorMsg = "Failed to read geometry shader file: " + geometryShaderFile;
      return nullptr;
    }
  }

  return loadCustomProgram(name, vertexSrc, fragmentSrc, geometrySrc, outErrorMsg);

  MRPT_END
}

Program::Ptr ShaderProgramManager::getCustomProgram(const std::string& name) const
{
  auto it = m_customPrograms.find(name);
  return it != m_customPrograms.end() ? it->second : nullptr;
}

bool ShaderProgramManager::hasCustomProgram(const std::string& name) const
{
  return m_customPrograms.find(name) != m_customPrograms.end();
}

bool ShaderProgramManager::removeCustomProgram(const std::string& name)
{
  MRPT_START
  checkThread();

  auto it = m_customPrograms.find(name);
  if (it != m_customPrograms.end())
  {
    m_customPrograms.erase(it);
    return true;
  }
  return false;

  MRPT_END
}

shader_id_t ShaderProgramManager::getShaderVariant(
    shader_id_t baseShaderID, bool isShadowMapPass) const
{
  auto it = m_shaderVariants.find({baseShaderID, isShadowMapPass});
  if (it != m_shaderVariants.end())
  {
    return it->second;
  }

  // No variant exists, return original shader
  return baseShaderID;
}

void ShaderProgramManager::buildShaderVariantMap()
{
  using ID = DefaultShaderID;

  // Shadow rendering variants:
  // Format: {base_shader, is_shadow_pass} → variant_shader

  // Triangles with lighting
  m_shaderVariants[{ID::TRIANGLES_LIGHT, true}] = ID::TRIANGLES_SHADOW_1ST;
  m_shaderVariants[{ID::TRIANGLES_LIGHT, false}] = ID::TRIANGLES_SHADOW_2ND;

  // Textured triangles with lighting
  m_shaderVariants[{ID::TEXTURED_TRIANGLES_LIGHT, true}] = ID::TEXTURED_TRIANGLES_SHADOW_1ST;
  m_shaderVariants[{ID::TEXTURED_TRIANGLES_LIGHT, false}] = ID::TEXTURED_TRIANGLES_SHADOW_2ND;

  // Triangles without lighting (still cast shadows)
  m_shaderVariants[{ID::TRIANGLES_NO_LIGHT, true}] = ID::TRIANGLES_SHADOW_1ST;
  m_shaderVariants[{ID::TRIANGLES_NO_LIGHT, false}] = ID::TRIANGLES_NO_LIGHT;

  // Textured triangles without lighting (still cast shadows)
  m_shaderVariants[{ID::TEXTURED_TRIANGLES_NO_LIGHT, true}] = ID::TEXTURED_TRIANGLES_SHADOW_1ST;
  m_shaderVariants[{ID::TEXTURED_TRIANGLES_NO_LIGHT, false}] = ID::TEXTURED_TRIANGLES_NO_LIGHT;

  // Points don't cast shadows (use NONE for 1st pass to skip them)
  m_shaderVariants[{ID::POINTS, true}] = ID::NONE;
  m_shaderVariants[{ID::POINTS, false}] = ID::POINTS;

  // Lines don't cast shadows
  m_shaderVariants[{ID::WIREFRAME, true}] = ID::NONE;
  m_shaderVariants[{ID::WIREFRAME, false}] = ID::WIREFRAME;

  // Text doesn't cast shadows
  m_shaderVariants[{ID::TEXT, true}] = ID::NONE;
  m_shaderVariants[{ID::TEXT, false}] = ID::TEXT;

  // Skybox never in shadow passes
  m_shaderVariants[{ID::SKYBOX, true}] = ID::NONE;
  m_shaderVariants[{ID::SKYBOX, false}] = ID::SKYBOX;
}

Program::Ptr ShaderProgramManager::compileShaderProgram(
    const std::string& vertexSrc,
    const std::string& fragmentSrc,
    const std::string& geometrySrc,
    const std::string& shaderName,
    std::string* outErrorMsg)
{
  MRPT_START

  std::vector<Shader> shaders;

  // Compile vertex shader
  {
    Shader vertShader;
    std::string vertError;
    if (!vertShader.compile(GL_VERTEX_SHADER, {vertexSrc}, vertError))
    {
      if (outErrorMsg) *outErrorMsg = "Vertex shader error: " + vertError;
      return nullptr;
    }
    shaders.push_back(std::move(vertShader));
  }

  // Compile fragment shader
  {
    Shader fragShader;
    std::string fragError;
    if (!fragShader.compile(GL_FRAGMENT_SHADER, {fragmentSrc}, fragError))
    {
      if (outErrorMsg) *outErrorMsg = "Fragment shader error: " + fragError;
      return nullptr;
    }
    shaders.push_back(std::move(fragShader));
  }

  // Compile geometry shader (if provided)
  if (!geometrySrc.empty())
  {
    Shader geomShader;
    std::string geomError;
    if (!geomShader.compile(GL_GEOMETRY_SHADER, {geometrySrc}, geomError))
    {
      if (outErrorMsg) *outErrorMsg = "Geometry shader error: " + geomError;
      return nullptr;
    }
    shaders.push_back(std::move(geomShader));
  }

  // Link program
  auto program = std::make_shared<Program>();
  std::string linkError;
  if (!program->linkProgram(shaders, linkError))
  {
    if (outErrorMsg) *outErrorMsg = "Link error: " + linkError;
    return nullptr;
  }

  // Declare common uniforms and attributes
  declareCommonUniforms(*program);
  declareCommonAttributes(*program);

  return program;

  MRPT_END
}

void ShaderProgramManager::declareCommonUniforms(Program& program)
{
  // Common matrix uniforms
  static const char* commonUniforms[] = {
      "p_matrix",
      "v_matrix",
      "m_matrix",
      "mv_matrix",
      "pmv_matrix",
      "v_matrix_no_translation",
      "light_pv_matrix",
      "cam_position",
      "materialSpecular",
      nullptr  // sentinel
  };

  for (int i = 0; commonUniforms[i] != nullptr; i++)
  {
    try
    {
      program.declareUniform(commonUniforms[i]);
    }
    catch (...)
    {
      // Uniform doesn't exist in this shader - that's OK
    }
  }
}

void ShaderProgramManager::declareCommonAttributes(Program& program)
{
  // Common vertex attributes
  static const char* commonAttributes[] = {
      "position", "vertexColor", "vertexNormal", "vertexUV", nullptr  // sentinel
  };

  for (int i = 0; commonAttributes[i] != nullptr; i++)
  {
    try
    {
      program.declareAttribute(commonAttributes[i]);
    }
    catch (...)
    {
      // Attribute doesn't exist in this shader - that's OK
    }
  }
}

void ShaderProgramManager::clear()
{
  MRPT_START
  checkThread();

  if (m_verbose && !m_builtinPrograms.empty())
  {
    std::cout << "[ShaderProgramManager::clear] Releasing " << m_builtinPrograms.size()
              << " built-in and " << m_customPrograms.size() << " custom programs\n";
  }

  m_builtinPrograms.clear();
  m_customPrograms.clear();

  MRPT_END
}

std::vector<shader_id_t> ShaderProgramManager::getLoadedShaderIDs() const
{
  std::vector<shader_id_t> ids;
  ids.reserve(m_builtinPrograms.size());

  for (const auto& [id, prog] : m_builtinPrograms)
  {
    ids.push_back(id);
  }

  return ids;
}

std::vector<std::string> ShaderProgramManager::getLoadedCustomShaderNames() const
{
  std::vector<std::string> names;
  names.reserve(m_customPrograms.size());

  for (const auto& [name, prog] : m_customPrograms)
  {
    names.push_back(name);
  }

  return names;
}

std::unique_ptr<ShaderProgramManager> mrpt::opengl::createAndPreloadShaders()
{
  auto manager = std::make_unique<ShaderProgramManager>();

  std::vector<std::string> errors;
  // ----
  return {};
}
