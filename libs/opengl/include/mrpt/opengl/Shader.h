/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/core/optional_ref.h>

#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace mrpt::opengl
{
/** Type for IDs of shaders.
 * \sa DefaultShaderID, LoadDefaultShader()
 * \ingroup mrpt_opengl_grp
 */
using shader_id_t = uint8_t;

/** A list of shader IDs */
using shader_list_t = std::vector<shader_id_t>;

/** A resource handling helper for OpenGL "Shader" compiled code fragment.
 *
 * The OpenGL shader resource will be freed upon destruction or when clear() is
 * called. Normally, users want shader(s) to be linked into a
 * mrpt::opengl::Program.
 *
 * \sa CRenderizable
 * \ingroup mrpt_opengl_grp
 */
class Shader
{
   public:
	Shader() = default;
	~Shader();
	Shader& operator=(const Shader&) = delete;
	Shader(const Shader&) = delete;
	Shader& operator=(Shader&&);
	Shader(Shader&&);

	bool empty() const { return m_data->shader == 0; }
	/** Frees the shader program in OpenGL. */
	void clear();

	/** Build a shader from source code.
	 * \param[in] type Any valid argument to glCreateShader()
	 * \param[in] shaderCode The shading source code. Tip: users can read it
	 * from a file with mrpt::io::file_get_contents().
	 * \param[out] outErrorMessages If provided, build errors will be saved
	 * here. If not, they will dumped to std::cerr
	 * \return false on error.
	 */
	bool compile(
		unsigned int type, const std::string& shaderCode,
		mrpt::optional_ref<std::string> outErrorMessages = std::nullopt);

	unsigned int handle() const { return m_data->shader; }

	struct Data
	{
		unsigned int shader = 0;
		std::thread::id creationThread{};
		bool inPostponedDestructionQueue = false;
		void destroy();
	};

   private:
	std::shared_ptr<Data> m_data = std::make_shared<Data>();
};

/** A resource handling helper for OpenGL Shader "programs".
 *
 * The OpenGL "program" resource will be freed upon destruction or when clear()
 * is called.
 *
 * \sa CRenderizable
 * \ingroup mrpt_opengl_grp
 */
class Program
{
   public:
	Program() = default;
	~Program();

	using Ptr = std::shared_ptr<Program>;

	bool empty() const { return m_data && m_data->program == 0; }
	/** Frees the shader program in OpenGL. */
	void clear();

	/** Links an OpenGL program with all shader code fragments previously
	 * inserted into shaders.
	 * \param[in,out] shaders The shader code fragments. Will be moved into this
	 * Program object, who will become the owner from now on and will eventually
	 * free its resources.
	 * \param[out] outErrorMessages If provided, build
	 * errors will be saved here. If not, they will dumped to std::cerr
	 *
	 * \return
	 * false on error.
	 */
	bool linkProgram(
		std::vector<Shader>& shaders,
		mrpt::optional_ref<std::string> outErrorMessages = std::nullopt);

	void declareUniform(const std::string& name);
	void declareAttribute(const std::string& name);

	unsigned int programId() const
	{
		ASSERT_(m_data && m_data->program != 0);
		return m_data->program;
	}

	int uniformId(const char* name) const { return m_data->uniforms.at(name); }
	int attributeId(const char* name) const { return m_data->attribs.at(name); }

	bool hasUniform(const char* name) const
	{
		return m_data->uniforms.count(name) != 0;
	}
	bool hasAttribute(const char* name) const
	{
		return m_data->attribs.count(name) != 0;
	}

	/** Prints a textual summary of the program */
	void dumpProgramDescription(std::ostream& o) const;

	struct Data
	{
		std::vector<Shader> shaders;
		unsigned int program = 0;
		std::thread::id linkedThread{};
		bool inPostponedDestructionQueue = false;

		/** OpenGL Uniforms/attribs defined by the user as inputs/outputs in
		 * shader code. \sa declareUniform(), declareAttribute();
		 */
		std::unordered_map<std::string, int> uniforms, attribs;

		void destroy();
	};

   private:
	std::shared_ptr<Data> m_data = std::make_shared<Data>();
};

}  // namespace mrpt::opengl
