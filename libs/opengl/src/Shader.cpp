/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>

#include <iostream>
#include <list>
#include <mutex>
#include <thread>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

// ============ Shader ============
Shader::~Shader() { clear(); }
Shader& Shader::operator=(Shader&& o)
{
	m_shader = o.m_shader;
	o.m_shader = 0;
	return *this;
}
Shader::Shader(Shader&& o)
{
	m_shader = o.m_shader;
	o.m_shader = 0;
}

void Shader::clear()
{
	if (!m_shader) return;	// Nothing to do
#if MRPT_HAS_OPENGL_GLUT
	glDeleteShader(m_shader);
	m_shader = 0;
#endif
}

bool Shader::compile(
	unsigned int type, const std::string& shaderCode,
	mrpt::optional_ref<std::string> outErrorMessages)
{
#if MRPT_HAS_OPENGL_GLUT
	clear();

	m_shader = glCreateShader(static_cast<GLenum>(type));

	const GLchar* source = shaderCode.c_str();
	const GLint length = shaderCode.size();

	glShaderSource(m_shader, 1, &source, &length);
	glCompileShader(m_shader);

	GLint shader_ok;
	glGetShaderiv(m_shader, GL_COMPILE_STATUS, &shader_ok);
	if (!shader_ok)
	{
		GLint log_length;
		std::string log;
		glGetShaderiv(m_shader, GL_INFO_LOG_LENGTH, &log_length);
		log.resize(log_length);
		glGetShaderInfoLog(m_shader, log_length, NULL, &log[0]);

		if (outErrorMessages) outErrorMessages.value().get() = std::move(log);
		else
			std::cerr << "[Shader::compile] Compile error: " << log << "\n";

		glDeleteShader(m_shader);
		m_shader = 0;
		return false;  // error
	}

	return true;  // ok
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}

// ========= Program =============
namespace mrpt::opengl::internal
{
static std::list<Program> pendingToClear;
static std::mutex pendingToClear_mtx;

// Frees those programs that were still waiting since they were attempted to
// delete from a wrong thread. If we are now at that thread, clean up:
void clearPendingIfPossible()
{
	std::lock_guard<std::mutex> lck(pendingToClear_mtx);
	for (auto it = pendingToClear.begin(); it != pendingToClear.end();)
	{
		if (!it->m_data)
		{
			it = pendingToClear.erase(it);
			continue;
		}
		if (it->m_data->linkedThread == std::this_thread::get_id())
		{
			it->internal_clear();
			it = pendingToClear.erase(it);
			continue;
		}
		else
		{
			++it;
		}
	}
}
}  // namespace mrpt::opengl::internal

namespace mrpt::opengl::internal
{
void clearPendingIfPossible();
}

Program::~Program() { clear(); }

void Program::clear()
{
	if (!m_data->program) return;

	// If we are in the same thread that created us, ok, clean up.
	// Otherwise, postpone it for later on:
	if (m_data->linkedThread == std::this_thread::get_id())
	{ internal_clear(); }
	else
	{
		// Postpone:
		{
			std::lock_guard<std::mutex> lck(internal::pendingToClear_mtx);
			internal::pendingToClear.emplace_back().m_data = std::move(m_data);
		}
		m_data = std::make_unique<Data>();
	}
	internal::clearPendingIfPossible();
}

void Program::internal_clear()
{
	if (!m_data->program) return;
#if MRPT_HAS_OPENGL_GLUT

	// See clear() comments
	ASSERT_(m_data->linkedThread == std::this_thread::get_id());

	// 1) Detach shaders from program.
	for (const Shader& s : m_data->shaders)
		glDetachShader(m_data->program, s.handle());

	// 2) Delete program.
	glDeleteProgram(m_data->program);

	// 3) Delete shaders.
	m_data->shaders.clear();

	// 4) Delete all variables:
	m_uniforms.clear();
	m_attribs.clear();
#endif

	m_data->program = 0;
}

bool Program::linkProgram(
	std::vector<Shader>& shaders,
	mrpt::optional_ref<std::string> outErrorMessages)
{
#if MRPT_HAS_OPENGL_GLUT
	clear();

	m_data->program = glCreateProgram();
	ASSERT_(m_data->program != 0);

	// Take ownership of shaders:
	m_data->shaders = std::move(shaders);
	m_data->linkedThread = std::this_thread::get_id();

	for (const auto& shader : m_data->shaders)
		glAttachShader(m_data->program, shader.handle());

	glLinkProgram(m_data->program);
	CHECK_OPENGL_ERROR();

	GLint program_ok;
	glGetProgramiv(m_data->program, GL_LINK_STATUS, &program_ok);
	if (!program_ok)
	{
		GLint log_length;
		std::string log;
		glGetProgramiv(m_data->program, GL_INFO_LOG_LENGTH, &log_length);
		log.resize(log_length);
		glGetProgramInfoLog(m_data->program, log_length, NULL, &log[0]);

		if (outErrorMessages) outErrorMessages.value().get() = std::move(log);
		else
			std::cerr << "[Program::linkProgram] Link error: " << log << "\n";
		clear();
		return false;  // error
	}

	return true;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}

void Program::declareUniform(const std::string& name)
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(!empty());

	if (m_uniforms.count(name) != 0)
		THROW_EXCEPTION_FMT(
			"declareUniform: Name `%s` already registered", name.c_str());

	const auto ret = glGetUniformLocation(
		m_data->program, static_cast<const GLchar*>(name.c_str()));
	if (ret < 0)
		THROW_EXCEPTION_FMT(
			"declareUniform: glGetUniformLocation() returned error for `%s`",
			name.c_str());
	m_uniforms[name] = ret;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}
void Program::declareAttribute(const std::string& name)
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(!empty());

	if (m_attribs.count(name) != 0)
		THROW_EXCEPTION_FMT(
			"declareAttribute: Name `%s` already registered", name.c_str());

	const auto ret = glGetAttribLocation(
		m_data->program, static_cast<const GLchar*>(name.c_str()));
	if (ret < 0)
		THROW_EXCEPTION_FMT(
			"declareAttribute: glGetAttribLocation() returned error for `%s`",
			name.c_str());
	m_attribs[name] = ret;

#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}

void Program::dumpProgramDescription(std::ostream& o) const
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(!empty());

	GLint count;

	GLint size;	 // size of the variable
	GLenum type;  // type of the variable (float, vec3 or mat4, etc)

	const GLsizei bufSize = 32;	 // maximum name length
	GLchar name[bufSize];  // variable name in GLSL
	GLsizei length;	 // name length

	// Attributes
	glGetProgramiv(m_data->program, GL_ACTIVE_ATTRIBUTES, &count);
	o << mrpt::format("Active Attributes: %d\n", count);

	for (GLint i = 0; i < count; i++)
	{
		glGetActiveAttrib(
			m_data->program, (GLuint)i, bufSize, &length, &size, &type, name);

		o << mrpt::format("Attribute #%d Type: %u Name: %s\n", i, type, name);
	}

	// Uniforms
	glGetProgramiv(m_data->program, GL_ACTIVE_UNIFORMS, &count);
	printf("Active Uniforms: %d\n", count);

	for (GLint i = 0; i < count; i++)
	{
		glGetActiveUniform(
			m_data->program, (GLuint)i, bufSize, &length, &size, &type, name);

		o << mrpt::format("Uniform #%d Type: %u Name: %s\n", i, type, name);
	}
#endif
}
