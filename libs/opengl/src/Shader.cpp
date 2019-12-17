/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/Shader.h>
#include "opengl_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

MRPT_TODO("Make m_color a program attribute");

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
	if (!m_shader) return;  // Nothing to do
#if MRPT_HAS_OPENGL_GLUT
	glDeleteShader(m_shader);
	m_shader = 0;
#endif
}

bool Shader::compileFromSource(
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

		if (outErrorMessages)
			outErrorMessages.value().get() = std::move(log);
		else
			std::cerr << "[Shader::compileFromSource] Compile error: " << log
					  << "\n";

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
Program::~Program() { clear(); }

void Program::clear()
{
	if (!m_program) return;

#if MRPT_HAS_OPENGL_GLUT
	// 1) Detach shaders from program.
	for (const Shader& s : m_shaders) glDetachShader(m_program, s.handle());

	// 2) Delete program.
	glDeleteProgram(m_program);

	// 3) Delete shaders.
	m_shaders.clear();

	// 4) Delete all variables:
	m_uniforms.clear();
	m_attribs.clear();
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}

bool Program::linkProgram(
	std::vector<Shader>& shaders,
	mrpt::optional_ref<std::string> outErrorMessages)
{
#if MRPT_HAS_OPENGL_GLUT
	clear();
	MRPT_TODO("Finish!");

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
		m_program, static_cast<const GLchar*>(name.c_str()));
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
		m_program, static_cast<const GLchar*>(name.c_str()));
	if (ret < 0)
		THROW_EXCEPTION_FMT(
			"declareAttribute: glGetAttribLocation() returned error for `%s`",
			name.c_str());
	m_attribs[name] = ret;

#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}
