/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>

#include <chrono>
#include <iostream>
#include <thread>

// We need OpenGL headers for "GL_VERTEX_SHADER","GL_FRAGMENT_SHADER"
#include <mrpt/config.h>
#if MRPT_HAS_OPENGL_GLUT
#include <mrpt/opengl/opengl_api.h>
#endif

// Demo of how to install a custom shader program:
static void installCustomShader(mrpt::opengl::COpenGLScene& scene)
{
#if MRPT_HAS_OPENGL_GLUT

	// Define shader program and properties
	// ------------------------------------------
	const char* vertex_shader = nullptr;
	const char* fragment_shader = nullptr;
	std::vector<std::string> attribs, uniforms;

	// For this example, use the standard vertex shader,...
	vertex_shader =
		R"XXX(
#version 330 core

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;

out vec3 frag_position, frag_normal;
out vec4 frag_materialColor;

void main()
{
    vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;

    frag_position = eye_position.xyz;
    frag_materialColor = vertexColor;
    frag_normal   = (mv_matrix * vec4(normalize(vertexNormal), 0.0)).xyz;
}
)XXX";

	// ...but modify the fragment shader:
	fragment_shader =
		R"XXX(
#version 330 core

// FRAGMENT SHADER: Demo for custom shaders. 
// Set color from depth.

uniform mat4 p_matrix;
uniform mat4 mv_matrix;

in vec3 frag_position;
in vec4 frag_materialColor;

out vec4 color;

void main()
{
    vec4 posWrtEye = p_matrix*mv_matrix * vec4(frag_position, 1.0);
    color = vec4(vec3(posWrtEye.z), 1.0);
}
)XXX";

	uniforms = {"p_matrix", "mv_matrix"};
	attribs = {"position"};

	// Compile shader:
	// ------------------------------------------
	std::string errMsgs;
	std::vector<mrpt::opengl::Shader> lstShaders;
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

	auto shader = std::make_shared<mrpt::opengl::Program>();

	if (!shader->linkProgram(lstShaders, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error linking Opengl Shader programs:\n%s", errMsgs.c_str());
	}

#if 1
	// Debug:
	std::cout << "Built Shader program\n";
	shader->dumpProgramDescription(std::cout);
	std::cout << "\n";
#endif

	// Uniforms:
	for (const auto& name : uniforms)
		shader->declareUniform(name);

	// Attributes:
	for (const auto& name : attribs)
		shader->declareAttribute(name);

	// Store in MRPT object:
	// ------------------------------------------

	// And store as the "TRIANGLES" shader in the MRPT viewport:
	// In MRPT, shaders are a property of viewports:
	auto vp = scene.getViewport();

	// Make sure default shaders are loaded (if not already):
	vp->loadDefaultShaders();

	// Overwrite the shaders we want to customize:
	const auto id = mrpt::opengl::DefaultShaderID::TRIANGLES_NO_LIGHT;
	vp->shaders()[id] = std::move(shader);

#endif
}

// ------------------------------------------------------
//				DemoCustomShaders
// ------------------------------------------------------
void DemoCustomShaders()
{
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::gui;
	using namespace mrpt::opengl;
	using namespace mrpt::math;
	using namespace std::string_literals;

	CDisplayWindow3D win("Demo of MRPT's custom shaders", 640, 480);

	COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	float off_x = 0;
	// Box
	{
		auto obj = opengl::CBox::Create(
			TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), true, 3.0);
		obj->setLocation(off_x, 0, 0);
		theScene->insert(obj);

		auto obj2 =
			opengl::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
		obj2->setLocation(off_x, 4, 0);
		theScene->insert(obj2);

		auto obj3 =
			opengl::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
		obj3->enableBoxBorder(true);
		obj3->setLineWidth(3);
		obj3->setColor_u8(0xff, 0x00, 0x00, 0xa0);
		obj3->setLocation(off_x, 8, 0);
		theScene->insert(obj3);
	}

	win.setCameraZoom(25);

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	// Wait for the window to be open:
	win.wait_for_GL_context();

	// Create the shader after the window is open, so we are sure it has a
	// proper GL context:
	{
		CDisplayWindow3DLocker locker(win, theScene);

		// Shaders must be created from the main OpenGL thread:
		win.sendFunctionToRunOnGUIThread(
			[theScene]() { installCustomShader(*theScene); });
	}

	win.repaint();

	cout << "Close the window to end.\n";

	while (win.isOpen())
	{
		std::this_thread::sleep_for(50ms);
		win.repaint();
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		DemoCustomShaders();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return 1;
	}
}
