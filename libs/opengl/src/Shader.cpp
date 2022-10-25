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
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>

#include <iostream>
#include <list>
#include <mutex>
#include <thread>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

// ========= clearPendingIfPossible =============
namespace mrpt::opengl::internal
{
// pending to clear (ppc) lists:
static std::list<std::shared_ptr<Program::Data>> pptc;
static std::list<std::shared_ptr<Shader::Data>> spptc;
static bool inClearPendingIfPossible = false;
static std::mutex pendingToClear_mtx;

// Frees those programs that were still waiting since they were attempted to
// delete from a wrong thread. If we are now at that thread, clean up:
void clearPendingIfPossible()
{
	std::lock_guard<std::mutex> lck(pendingToClear_mtx);
	inClearPendingIfPossible = true;

	for (auto it = pptc.begin(); it != pptc.end();)
	{
		if (!*it)
		{
			it = pptc.erase(it);
			continue;
		}
		if ((*it)->linkedThread == std::this_thread::get_id())
		{
			(*it)->destroy();
			it = pptc.erase(it);
			continue;
		}
		else
		{
			++it;
		}
	}

	for (auto it = spptc.begin(); it != spptc.end();)
	{
		if (!*it)
		{
			it = spptc.erase(it);
			continue;
		}
		if ((*it)->creationThread == std::this_thread::get_id())
		{
			(*it)->destroy();
			it = spptc.erase(it);
			continue;
		}
		else
		{
			++it;
		}
	}
	inClearPendingIfPossible = false;
}
}  // namespace mrpt::opengl::internal

// ============ Shader ============
Shader::~Shader() { clear(); }
Shader& Shader::operator=(Shader&& o)
{
	m_data = std::move(o.m_data);
	o.m_data.reset();
	return *this;
}
Shader::Shader(Shader&& o)
{
	m_data = std::move(o.m_data);
	o.m_data.reset();
}

void Shader::clear()
{
	if (!m_data || !m_data->shader) return;

	// If we are in the same thread that created us, ok, clean up.
	// Otherwise, postpone it for later on:
	if (m_data->creationThread == std::this_thread::get_id())
	{ m_data->destroy(); }
	else
	{
		// Postpone (except if we are already in the global dtor of the queue!)
		if (!m_data->inPostponedDestructionQueue)
		{
			std::lock_guard<std::mutex> lck(internal::pendingToClear_mtx);
			m_data->inPostponedDestructionQueue = true;
			internal::spptc.emplace_back(m_data);
		}
		m_data = std::make_shared<Data>();
	}
	// if (!internal::inClearPendingIfPossible)
	// internal::clearPendingIfPossible();
}

void Shader::Data::destroy()
{
	if (!shader) return;

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	// See clear() comments
	ASSERT_(creationThread == std::this_thread::get_id());

	glDeleteShader(shader);
#endif
	shader = 0;
}

bool Shader::compile(
	unsigned int type, const std::string& shaderCode,
	mrpt::optional_ref<std::string> outErrorMessages)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	clear();

	m_data->creationThread = std::this_thread::get_id();

	m_data->shader = glCreateShader(static_cast<GLenum>(type));

	const GLchar* source = shaderCode.c_str();
	const GLint length = shaderCode.size();

	glShaderSource(m_data->shader, 1, &source, &length);
	glCompileShader(m_data->shader);

	GLint shader_ok;
	glGetShaderiv(m_data->shader, GL_COMPILE_STATUS, &shader_ok);
	if (!shader_ok)
	{
		GLint log_length;
		std::string log;
		glGetShaderiv(m_data->shader, GL_INFO_LOG_LENGTH, &log_length);
		log.resize(log_length);
		glGetShaderInfoLog(m_data->shader, log_length, NULL, &log[0]);

		if (outErrorMessages) outErrorMessages.value().get() = std::move(log);
		else
			std::cerr << "[Shader::compile] Compile error: " << log << "\n";

		glDeleteShader(m_data->shader);
		m_data->shader = 0;
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
	if (!m_data || !m_data->program) return;

	MRPT_TODO("Remove this??");

	// If we are in the same thread that created us, ok, clean up.
	// Otherwise, postpone it for later on:
	if (m_data->linkedThread == std::this_thread::get_id())
	{ m_data->destroy(); }
	else
	{
		// Postpone (except if we are already in the global dtor of the queue!)
		if (!m_data->inPostponedDestructionQueue)
		{
			std::lock_guard<std::mutex> lck(internal::pendingToClear_mtx);
			m_data->inPostponedDestructionQueue = true;
			internal::pptc.emplace_back(m_data);
		}
		m_data = std::make_shared<Data>();
	}
	if (!internal::inClearPendingIfPossible) internal::clearPendingIfPossible();
}

void Program::Data::destroy()
{
	if (!program) return;
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// See clear() comments
	ASSERT_(linkedThread == std::this_thread::get_id());

	// 1) Detach shaders from program.
	for (const Shader& s : shaders)
		glDetachShader(program, s.handle());

	// 2) Delete program.
	glDeleteProgram(program);

	// 3) Delete shaders.
	shaders.clear();

	// 4) Delete all variables:
	uniforms.clear();
	attribs.clear();
#endif

	program = 0;
}

bool Program::linkProgram(
	std::vector<Shader>& shaders,
	mrpt::optional_ref<std::string> outErrorMessages)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	clear();

#if defined(MRPT_OS_LINUX)
	// Workaround to enfore wxWidgets to use GLSL>=3.3 even for wxWidgets<3.0.4
	// See CWxGLCanvasBase::CWxGLCanvasBase.
	if (!::getenv("MESA_GL_VERSION_OVERRIDE"))
	{ ::setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 1 /*overwrite*/); }
#endif

	m_data->program = glCreateProgram();
	CHECK_OPENGL_ERROR();
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
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	ASSERT_(!empty());

	if (m_data->uniforms.count(name) != 0)
		THROW_EXCEPTION_FMT(
			"declareUniform: Name `%s` already registered", name.c_str());

	const auto ret = glGetUniformLocation(
		m_data->program, static_cast<const GLchar*>(name.c_str()));
	if (ret < 0)
		THROW_EXCEPTION_FMT(
			"declareUniform: glGetUniformLocation() returned error for `%s`",
			name.c_str());
	m_data->uniforms[name] = ret;
#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}
void Program::declareAttribute(const std::string& name)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	ASSERT_(!empty());

	if (m_data->attribs.count(name) != 0)
		THROW_EXCEPTION_FMT(
			"declareAttribute: Name `%s` already registered", name.c_str());

	const auto ret = glGetAttribLocation(
		m_data->program, static_cast<const GLchar*>(name.c_str()));
	if (ret < 0)
		THROW_EXCEPTION_FMT(
			"declareAttribute: glGetAttribLocation() returned error for `%s`",
			name.c_str());
	m_data->attribs[name] = ret;

#else
	THROW_EXCEPTION("MRPT built without OpenGL support.");
#endif
}

void Program::dumpProgramDescription(std::ostream& o) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
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
