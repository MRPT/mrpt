/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <Eigen/Dense>	// First! to avoid conflicts with X.h
//
#include <mrpt/core/get_env.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>	 // crossProduct3D()
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/Scene.h>
#include <mrpt/opengl/Viewport.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/CTimeLogger.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::serialization::metaprogramming;
using namespace std;

IMPLEMENTS_SERIALIZABLE(Viewport, CSerializable, mrpt::opengl)

const bool MRPT_OPENGL_DEBUG_SHOW_SHADOW_MAP =
	mrpt::get_env<bool>("MRPT_OPENGL_DEBUG_SHOW_SHADOW_MAP", false);

// #define OPENGLVIEWPORT_ENABLE_TIMEPROFILING

#if defined(OPENGLVIEWPORT_ENABLE_TIMEPROFILING)
mrpt::system::CTimeLogger glv_timlog;
#endif

/*--------------------------------------------------------------

			IMPLEMENTATION OF Viewport

  ---------------------------------------------------------------*/

/*--------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
Viewport::Viewport(Scene* parent, const string& name)
	: m_parent(parent), m_name(name)
{
}

Viewport::~Viewport() { clear(); }

void Viewport::setCloneView(const string& clonedViewport)
{
	clear();
	m_isCloned = true;
	m_clonedViewport = clonedViewport;
}

void Viewport::setViewportPosition(
	const double x, const double y, const double width, const double height)
{
	MRPT_START
	ASSERT_(m_view_width > 0);
	ASSERT_(m_view_height > 0);

	m_view_x = x;
	m_view_y = y;
	m_view_width = width;
	m_view_height = height;

	MRPT_END
}

/*--------------------------------------------------------------
					getViewportPosition
  ---------------------------------------------------------------*/
void Viewport::getViewportPosition(
	double& x, double& y, double& width, double& height)
{
	x = m_view_x;
	y = m_view_y;
	width = m_view_width;
	height = m_view_height;
}

/*--------------------------------------------------------------
					clear
  ---------------------------------------------------------------*/
void Viewport::clear() { m_objects.clear(); }
/*--------------------------------------------------------------
					insert
  ---------------------------------------------------------------*/
void Viewport::insert(const CRenderizable::Ptr& newObject)
{
	m_objects.push_back(newObject);
}

// Maps [0,1] to [0,Len], wrap negative numbers, etc.
static int sizeFromRatio(
	const int startCoord, const double dSize, const int iLength)
{
	if (dSize > 1)	// >1 -> absolute pixels:
		return static_cast<int>(dSize);
	else if (dSize < 0)
	{  // Negative numbers: Specify the right side coordinates instead of
		// the width:
		if (dSize >= -1)
			return static_cast<int>(-iLength * dSize - startCoord + 1);
		else
			return static_cast<int>(-dSize - startCoord + 1);
	}
	// Otherwise: a fraction
	return static_cast<int>(iLength * dSize);
}
static int startFromRatio(const double frac, const int dSize)
{
	const bool doWrap = (frac < 0);
	const auto fracAbs = std::abs(frac);

	const int L = fracAbs > 1 ? static_cast<int>(fracAbs)
							  : static_cast<int>(dSize * fracAbs);

	int ret = doWrap ? dSize - L : L;

	return ret;
}

// "Image mode" rendering:
void Viewport::renderImageMode() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
#if defined(OPENGLVIEWPORT_ENABLE_TIMEPROFILING)
	mrpt::system::CTimeLoggerEntry tle(
		glv_timlog, "Viewport::render imageview");
#endif

	// Do we have an actual image to render?
	if (!m_imageViewPlane || m_imageViewPlane->getTextureImage().isEmpty())
		return;

	auto _ = m_threadedData.get().state;  // make a copy

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Adjust the aspect ratio:
	const auto img_w = m_imageViewPlane->getTextureImage().getWidth();
	const auto img_h = m_imageViewPlane->getTextureImage().getHeight();
	const double img_ratio = double(img_w) / img_h;
	const double vw_ratio = double(_.viewport_width) / _.viewport_height;
	const double ratio = vw_ratio / img_ratio;

	_.matricesSetIdentity();

	if (ratio > 1) _.p_matrix(1, 1) *= ratio;
	else if (ratio > 0)
		_.p_matrix(0, 0) /= ratio;

	auto &p00 = _.p_matrix(0, 0), &p11 = _.p_matrix(1, 1);
	if (p00 > 0 && p11 > 0)
	{
		const double s = (p00 > p11) ? p00 : p11;
		p00 /= s;
		p11 /= s;
	}

	_.pmv_matrix.asEigen() =
		_.p_matrix.asEigen() * _.v_matrix.asEigen() * _.m_matrix.asEigen();

	// Pass 1: Process all objects (recursively for sets of objects):
	CListOpenGLObjects lst;
	lst.push_back(m_imageViewPlane);
	mrpt::opengl::RenderQueue rq;
	mrpt::opengl::enqueueForRendering(
		lst, _, rq, true /*skip cull*/, false /* isShadowMap*/);

	// pass 2: render, sorted by shader program:
	mrpt::opengl::processRenderQueue(rq, m_threadedData.get().shaders, m_light);

#endif
}

void Viewport::unloadShaders() { m_threadedData.get().shaders.clear(); }

void Viewport::loadDefaultShaders() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	MRPT_START

	using ID = DefaultShaderID;	 // save space below

	std::vector<shader_id_t> lstShaderIDs = {
		ID::POINTS,
		ID::WIREFRAME,
		ID::TRIANGLES_NO_LIGHT,
		ID::TRIANGLES_LIGHT,
		ID::TEXTURED_TRIANGLES_NO_LIGHT,
		ID::TEXTURED_TRIANGLES_LIGHT,
		ID::TEXT,
		ID::TRIANGLES_SHADOW_1ST,
		ID::TRIANGLES_SHADOW_2ND,
		ID::TEXTURED_TRIANGLES_SHADOW_1ST,
		ID::TEXTURED_TRIANGLES_SHADOW_2ND,
		ID::SKYBOX,
		ID::DEBUG_TEXTURE_TO_SCREEN};

	// -----------------------------------------------------------
	// Load general list of shaders
	// (and use them for the "no shadows" case):
	// -----------------------------------------------------------
	auto& shaders = m_threadedData.get().shaders;
	for (const auto& id : lstShaderIDs)
	{
		shaders[id] = mrpt::opengl::LoadDefaultShader(id);

		ASSERT_(shaders[id]);
		ASSERT_(!shaders[id]->empty());
	}

	// -----------------------------------------------------------
	// Shaders for rendering with shadows: 1st and 2nd passes use
	// different shader programs. Apply a replacement table.
	// -----------------------------------------------------------
	// clang-format off
	const std::map<shader_id_t, std::pair<shader_id_t, shader_id_t>>
	  replacements = {
		{ID::TRIANGLES_LIGHT,             {ID::TRIANGLES_SHADOW_1ST, ID::TRIANGLES_SHADOW_2ND}},
		{ID::TEXTURED_TRIANGLES_LIGHT,    {ID::TEXTURED_TRIANGLES_SHADOW_1ST, ID::TEXTURED_TRIANGLES_SHADOW_2ND}},
		{ID::POINTS,                      {ID::NONE, ID::POINTS}},
		{ID::WIREFRAME,                   {ID::NONE, ID::WIREFRAME}},
		{ID::TEXT,                        {ID::NONE, ID::TEXT}},
		{ID::TRIANGLES_NO_LIGHT,          {ID::TRIANGLES_SHADOW_1ST, ID::TRIANGLES_NO_LIGHT}},
		{ID::TEXTURED_TRIANGLES_NO_LIGHT, {ID::TRIANGLES_SHADOW_1ST, ID::TEXTURED_TRIANGLES_NO_LIGHT}},
		{ID::SKYBOX,                      {ID::NONE, ID::SKYBOX}}
		};
	//clang-format on

	// 1st pass. Replace shaders: we only need depth in the 1st stage.
	// 2nd pass. Replace shaders: we need to account for the shadow map.
	auto& shadowShaders1st = m_threadedData.get().shadersShadow1st;
	shadowShaders1st = shaders;
	auto& shadowShaders2nd = m_threadedData.get().shadersShadow2nd;
	shadowShaders2nd = shaders;

	for (const auto& kv : replacements)
	{
		if (kv.second.first!=ID::NONE)
			shadowShaders1st[kv.first] = shaders.at(kv.second.first);

		if (kv.second.second!=ID::NONE)
			shadowShaders2nd[kv.first] = shaders.at(kv.second.second);
	}

	MRPT_END
#endif
}

// for debugging only if MRPT_OPENGL_DEBUG_SHOW_SHADOW_MAP
#if (MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL)
// From: https://learnopengl.com/Advanced-Lighting/Shadows/Shadow-Mapping

// debugRenderQuad() renders a 1x1 XY quad in NDC
// -----------------------------------------
static unsigned int quadVAO = 0;
static unsigned int quadVBO;
static void debugRenderQuad()
{
	if (quadVAO == 0)
	{
		// clang-format off
        float quadVertices[] = {
            // positions        // texture Coords
            -1.0f, -0.5f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
            -0.5f, -0.5f, 0.0f, 1.0f, 1.0f,
            -0.5f, -1.0f, 0.0f, 1.0f, 0.0f,
        };
		// clang-format on

		// setup plane VAO
		glGenVertexArrays(1, &quadVAO);
		glGenBuffers(1, &quadVBO);
		glBindVertexArray(quadVAO);
		glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
		glBufferData(
			GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices,
			GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(
			0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(
			1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
			(void*)(3 * sizeof(float)));
	}
	glBindVertexArray(quadVAO);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}
#endif

/** Render a normal scene with 3D objects */
void Viewport::renderNormalSceneMode(
	const CCamera& useThisCamera, bool is1stShadowMapPass) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	MRPT_START

#ifdef MRPT_OPENGL_PROFILER
	mrpt::system::CTimeLoggerEntry tle(
		opengl_profiler(), "Viewport.renderNormalSceneMode");
#endif

	// Prepare camera (projection matrix):
	// Needed for both, regular, and 1st/2nd shadow passes:
	// Note: if we were in the 1st stage of shadow rendering,
	// the projection matrix is actually the light projection matrix,
	// but that's handled by the shaders themselves, not here.
	updateMatricesFromCamera(useThisCamera);

	const auto& _ = m_threadedData.get().state;

	// Get objects to render:
	const CListOpenGLObjects* objectsToRender = nullptr;

	if (m_isCloned)
	{  // Clone: render someone's else objects.
		ASSERT_(m_parent.get() != nullptr);

		const auto view = m_parent->getViewport(m_clonedViewport);
		if (!view)
			THROW_EXCEPTION_FMT(
				"Cloned viewport '%s' not found in parent Scene",
				m_clonedViewport.c_str());

		objectsToRender = &view->m_objects;
	}
	else
	{  // Normal case: render our own objects:
		objectsToRender = &m_objects;
	}

	// Optional pre-Render user code:
	if (hasSubscribers())
	{
		mrptEventGLPreRender ev(this);
		this->publishEvent(ev);
	}

	// Global OpenGL settings:
	// ---------------------------------
#if !defined(__EMSCRIPTEN__)
	glHint(
		GL_POLYGON_SMOOTH_HINT,
		m_OpenGL_enablePolygonNicest ? GL_NICEST : GL_FASTEST);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glEnable(GL_LINE_SMOOTH);
	CHECK_OPENGL_ERROR_IN_DEBUG();
#endif

	// Regular depth model:
	// 0: far, 1: near
	// -------------------------------
	glEnable(GL_DEPTH_TEST);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glDepthFunc(GL_LESS);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

// Enable point sizes>1
#if !defined(__EMSCRIPTEN__) && defined(GL_PROGRAM_POINT_SIZE)	// OSX undef?
	glEnable(GL_PROGRAM_POINT_SIZE);
	CHECK_OPENGL_ERROR_IN_DEBUG();
#endif

	// Enables Multisampling
	glEnable(GL_MULTISAMPLE);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// Pass 1: Process all objects (recursively for sets of objects):
	mrpt::opengl::RenderQueue rq;
	mrpt::opengl::RenderQueueStats rqStats;
	mrpt::opengl::enqueueForRendering(
		*objectsToRender, _, rq, false /*skip cull checks*/,
		is1stShadowMapPass /* is shadow map pass */, &rqStats);

	// pass 2: render, sorted by shader program:
	auto& shaders = m_shadowsEnabled
		? (is1stShadowMapPass ? m_threadedData.get().shadersShadow1st
							  : m_threadedData.get().shadersShadow2nd)
		: m_threadedData.get().shaders;

	// shadow map, if enabled:
	std::optional<unsigned int> depthMapTextureId;
	if (m_shadowsEnabled && m_ShadowMapFBO.initialized())
		depthMapTextureId = m_ShadowMapFBO.depthMapTextureId();

	// go render:
	mrpt::opengl::processRenderQueue(rq, shaders, m_light, depthMapTextureId);

#ifdef MRPT_OPENGL_PROFILER
	opengl_profiler().registerUserMeasure(
		"render.totalObjects", rqStats.numObjTotal);
	opengl_profiler().registerUserMeasure(
		"render.numObjRendered", rqStats.numObjRendered);
	opengl_profiler().registerUserMeasure(
		"render.ObjRenderedPercent",
		rqStats.numObjTotal != 0
			? 100.0 * rqStats.numObjRendered / rqStats.numObjTotal
			: 0);
#endif

	MRPT_END

#endif
}

void Viewport::renderViewportBorder() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	MRPT_START
	if (m_borderWidth < 1) return;

	auto _ = m_threadedData.get().state;

	_.matricesSetIdentity();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//
	if (!m_borderLines)
	{
		m_borderLines = mrpt::opengl::CSetOfLines::Create();
		m_borderLines->appendLine(-1, -1, 0, -1, 1, 0);
		m_borderLines->appendLine(-1, 1, 0, 1, 1, 0);
		m_borderLines->appendLine(1, 1, 0, 1, -1, 0);
		m_borderLines->appendLine(1, -1, 0, -1, -1, 0);
	}
	m_borderLines->setLineWidth(m_borderWidth);
	m_borderLines->setColor_u8(m_borderColor);

	CListOpenGLObjects lst;
	lst.push_back(m_borderLines);

	// Pass 1: Process all objects (recursively for sets of objects):
	mrpt::opengl::RenderQueue rq;
	mrpt::opengl::enqueueForRendering(
		lst, _, rq, true /*skip cull*/, false /* isShadowMap*/);

	// pass 2: render, sorted by shader program:
	mrpt::opengl::processRenderQueue(rq, m_threadedData.get().shaders, m_light);
	MRPT_END
#endif
}

void Viewport::renderTextMessages() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	MRPT_START

	// Ensure GL objects are up-to-date:
	m_2D_texts.regenerateGLobjects();

	// Prepare shaders upon first invokation:
	if (m_threadedData.get().shaders.empty()) loadDefaultShaders();

	// Prepare camera (projection matrix):
	TRenderMatrices _ = m_threadedData.get().state;	 // make a copy

	// Compute the projection matrix (p_matrix):
	// was: glLoadIdentity(); glOrtho(0, w, 0, h, -1, 1);
	const auto w = _.viewport_width, h = _.viewport_height;
	_.is_projective = false;

	// Reset model-view 4x4 matrix to the identity transformation:
	_.matricesSetIdentity();
	//_.computeOrthoProjectionMatrix(0, w, 0, h, m_clip_min, m_clip_max);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// Text messages should always be visible on top the rest:
	glDisable(GL_DEPTH_TEST);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// Collect all 2D text objects, and update their properties:
	CListOpenGLObjects objs;

	std::shared_lock<std::shared_mutex> lckRead2DTexts(m_2D_texts.mtx.data);
	for (auto& kv : m_2D_texts.messages)
	{
		const DataPerText& label = kv.second;

		// If (x,y) \in [0,1[, it's interpreted as a ratio, otherwise, as an
		// actual coordinate in pixels
		float x =
			label.x >= 1 ? label.x : (label.x < 0 ? w + label.x : label.x * w);
		float y =
			label.y >= 1 ? label.y : (label.y < 0 ? h + label.y : label.y * h);

		if (CText::Ptr& o = label.gl_text_shadow; o)
		{
			o->setFont(label.vfont_name, label.vfont_scale * 2);
			o->setString(label.text);
			o->setColor(label.shadow_color);
			// Change coordinates: mrpt text (0,0)-(1,1) to OpenGL
			// (-1,-1)-(+1,+1):
			o->setLocation(
				-1.0f + 2 * (x + 1) / w, -1.0f + 2 * (y - 1) / h, 0.1);
			objs.push_back(o);
		}
		if (CText::Ptr& o = label.gl_text; o)
		{
			o->setFont(label.vfont_name, label.vfont_scale * 2);
			o->setString(label.text);
			o->setColor(label.color);
			// Change coordinates: mrpt text (0,0)-(1,1) to OpenGL
			// (-1,-1)-(+1,+1):
			o->setLocation(-1.0f + 2 * x / w, -1.0f + 2 * y / h, 0);
			objs.push_back(o);
		}
	}
	lckRead2DTexts.unlock();

	// Pass 1: Process all objects (recursively for sets of objects):
	mrpt::opengl::RenderQueue rq;
	mrpt::opengl::enqueueForRendering(
		objs, _, rq, true /*skip cull*/, false /* isShadowMap*/);

	// pass 2: render, sorted by shader program:
	mrpt::opengl::processRenderQueue(rq, m_threadedData.get().shaders, m_light);
	MRPT_END
#endif
}

void Viewport::render(
	[[maybe_unused]] const int render_width,
	[[maybe_unused]] const int render_height,
	[[maybe_unused]] const int render_offset_x,
	[[maybe_unused]] const int render_offset_y,
	[[maybe_unused]] const CCamera* forceThisCamera) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	MRPT_START

#ifdef MRPT_OPENGL_PROFILER
	mrpt::system::CTimeLoggerEntry tle(opengl_profiler(), "Viewport.render");
#endif

	// Prepare shaders upon first invokation:
	if (m_threadedData.get().shaders.empty()) loadDefaultShaders();

	auto& _ = m_threadedData.get().state;

	auto* activeCameraPtr = internalResolveActiveCamera(forceThisCamera);

	// make a copy so the camera remains const over the rendering:
	const CCamera activeCamera = *activeCameraPtr;

	// Get former viewport
	GLint oldViewport[4];
	glGetIntegerv(GL_VIEWPORT, oldViewport);

	const GLint vx = render_offset_x + startFromRatio(m_view_x, render_width);
	const GLint vy = render_offset_y + startFromRatio(m_view_y, render_height);
	const GLint vw = sizeFromRatio(vx, m_view_width, render_width);
	const GLint vh = sizeFromRatio(vy, m_view_height, render_height);

	// Let the matrix generation stuff what's the viewport size (for real
	// rendering, not shadow 1st pass shadow map)
	_.viewport_width = vw;
	_.viewport_height = vh;

	// If we are rendering with shadows, run a camera-view depth map first
	// (Shadows 1st pass)
	// ----------------------------------------------------------------------
	if (m_shadowsEnabled && !isImageViewMode())
	{
		if (!m_ShadowMapFBO.initialized())
		{
			m_ShadowMapFBO.createDepthMap(m_ShadowMapSizeX, m_ShadowMapSizeY);
		}

		glEnable(GL_DEPTH_TEST);

		// Render scene to depth map, as seen from the light point of view:
		glViewport(0, 0, m_ShadowMapSizeX, m_ShadowMapSizeY);

		const auto oldFBs = m_ShadowMapFBO.bind();
		CHECK_OPENGL_ERROR_IN_DEBUG();

		glClear(GL_DEPTH_BUFFER_BIT);

		renderNormalSceneMode(activeCamera, true /* is1stShadowMapPass */);

		m_ShadowMapFBO.Bind(oldFBs);

		// The 2nd pass is done inside renderNormalSceneMode()

		// TODO: Any way to refactor the whole pipeline to avoid recursive
		// rendering twice?
	}

	// Change viewport:
	// -------------------------------------------
	glViewport(vx, vy, vw, vh);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// Clear depth&/color buffers:
	// -------------------------------------------

	glScissor(vx, vy, vw, vh);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glEnable(GL_SCISSOR_TEST);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (!m_isTransparent)
	{  // Clear color & depth buffers:
		// Save?

		glClearColor(
			m_background_color.R, m_background_color.G, m_background_color.B,
			m_background_color.A);
		CHECK_OPENGL_ERROR_IN_DEBUG();

		glClear(
			GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}
	else
	{  // Clear depth buffer only:
		glClear(GL_DEPTH_BUFFER_BIT);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}
	glDisable(GL_SCISSOR_TEST);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// If we are in "image mode", rendering is much simpler: just set
	//  ortho projection and render the image quad:
	if (isImageViewMode()) renderImageMode();
	else
		renderNormalSceneMode(activeCamera);

	// Draw text messages, if any:
	renderTextMessages();

	// Finally, draw the border:
	renderViewportBorder();

	// Optional post-Render user code:
	if (hasSubscribers())
	{
		mrptEventGLPostRender ev(this);
		this->publishEvent(ev);
	}

	// Debug:
	if (MRPT_OPENGL_DEBUG_SHOW_SHADOW_MAP && m_shadowsEnabled)
	{
		// render Depth map to quad for visual debugging
		auto& sh = shaders().at(DefaultShaderID::DEBUG_TEXTURE_TO_SCREEN);
		sh->use();
		sh->setInt("textureId", 0 /* Use GL_TEXTURE0 */);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_ShadowMapFBO.depthMapTextureId());
		debugRenderQuad();
	}

	// Restore viewport:
	glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);

	MRPT_END
#else
	THROW_EXCEPTION(
		"MRPT has been compiled with MRPT_HAS_OPENGL_GLUT=0! OpenGL "
		"functions are not implemented");
#endif
}

uint8_t Viewport::serializeGetVersion() const { return 9; }
void Viewport::serializeTo(mrpt::serialization::CArchive& out) const
{
	// Save data:
	out << m_camera << m_isCloned << m_isClonedCamera << m_clonedViewport
		<< m_name << m_isTransparent << m_borderWidth << m_view_x << m_view_y
		<< m_view_width << m_view_height;

	// Added in v1:
	out << m_background_color.R << m_background_color.G << m_background_color.B
		<< m_background_color.A;

	// Save objects:
	uint32_t n;
	n = (uint32_t)m_objects.size();
	out << n;
	for (const auto& m_object : m_objects)
		out << *m_object;

	// Added in v2: Global OpenGL settings:
	out << m_OpenGL_enablePolygonNicest;

	// Added in v3: Lights
	out << m_light;

	// Added in v4: text messages:
	std::shared_lock<std::shared_mutex> lckRead2DTexts(m_2D_texts.mtx.data);

	out.WriteAs<uint32_t>(m_2D_texts.messages.size());
	for (auto& kv : m_2D_texts.messages)
	{
		out << kv.first;  // id
		out << kv.second.x << kv.second.y << kv.second.text;

		const auto& fp = kv.second;

		out << fp.vfont_name << fp.vfont_scale << fp.color << fp.draw_shadow
			<< fp.shadow_color << fp.vfont_spacing << fp.vfont_kerning;
		out.WriteAs<uint8_t>(static_cast<uint8_t>(fp.vfont_style));
	}
	lckRead2DTexts.unlock();

	// Added in v5: image mode
	out.WriteAs<bool>(m_imageViewPlane);
	if (m_imageViewPlane) out << *m_imageViewPlane;

	// Added in v6:
	out << m_clonedCameraViewport;

	// Added in v8:
	out << m_shadowsEnabled << m_ShadowMapSizeX << m_ShadowMapSizeY;

	// Added in v9:
	out << m_clip_max << m_clip_min << m_lightShadowClipMin
		<< m_lightShadowClipMax;
}

void Viewport::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		{
			// Load data:
			in >> m_camera >> m_isCloned >> m_isClonedCamera >>
				m_clonedViewport >> m_name >> m_isTransparent >>
				m_borderWidth >> m_view_x >> m_view_y >> m_view_width >>
				m_view_height;

			// in v1:
			if (version >= 1)
			{
				if (version < 7)
				{
					// field removed in v7:
					bool was_m_custom_backgb_color;
					in >> was_m_custom_backgb_color;
				}

				in >> m_background_color.R >> m_background_color.G >>
					m_background_color.B >> m_background_color.A;
			}

			// Load objects:
			uint32_t n;
			in >> n;
			clear();
			m_objects.resize(n);

			for_each(
				m_objects.begin(), m_objects.end(), ObjectReadFromStream(&in));

			// Added in v2: Global OpenGL settings:
			if (version >= 2) { in >> m_OpenGL_enablePolygonNicest; }
			else
			{
				// Defaults
			}

			// Added in v3: Lights
			if (version >= 3) in >> m_light;
			else
			{
				// Default:
				m_light = TLightParameters();
			}

			// v4: text:
			m_2D_texts.messages.clear();
			uint32_t nTexts = 0;
			if (version >= 4) nTexts = in.ReadAs<uint32_t>();

			for (uint32_t i = 0; i < nTexts; i++)
			{
				const auto id = in.ReadAs<uint32_t>();
				double x, y;
				std::string text;
				in >> x >> y >> text;

				TFontParams fp;

				in >> fp.vfont_name >> fp.vfont_scale >> fp.color >>
					fp.draw_shadow >> fp.shadow_color >> fp.vfont_spacing >>
					fp.vfont_kerning;
				fp.vfont_style =
					static_cast<TOpenGLFontStyle>(in.ReadAs<uint8_t>());

				this->addTextMessage(x, y, text, id, fp);
			}

			// Added in v5: image mode
			if (in.ReadAs<bool>()) { in >> m_imageViewPlane; }
			else
			{
				m_imageViewPlane.reset();
			}

			if (version >= 6) in >> m_clonedCameraViewport;
			else
				m_clonedCameraViewport.clear();

			if (version >= 8)
			{
				in >> m_shadowsEnabled >> m_ShadowMapSizeX >> m_ShadowMapSizeY;
			}
			else
			{
				m_shadowsEnabled = false;
			}

			if (version >= 9)
			{
				in >> m_clip_max >> m_clip_min >> m_lightShadowClipMin >>
					m_lightShadowClipMax;
			}
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizable::Ptr Viewport::getByName(const string& str)
{
	for (auto& m_object : m_objects)
	{
		if (m_object->getName() == str) return m_object;
		else if (
			m_object->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, opengl))
		{
			if (CRenderizable::Ptr ret =
					std::dynamic_pointer_cast<CSetOfObjects>(m_object)
						->getByName(str);
				ret)
				return ret;
		}
	}
	return {};
}

void Viewport::initializeTextures()
{
	for (auto& obj : m_objects)
		obj->initializeTextures();
}

void Viewport::dumpListOfObjects(std::vector<std::string>& lst) const
{
	for (auto& obj : m_objects)
	{
		// Single obj:
		string s(obj->GetRuntimeClass()->className);
		if (!obj->getName().empty())
			s += string(" (") + obj->getName() + string(")");
		lst.emplace_back(s);

		if (obj->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			std::vector<std::string> auxLst;

			dynamic_cast<CSetOfObjects*>(obj.get())->dumpListOfObjects(auxLst);

			for (const auto& i : auxLst)
				lst.emplace_back(string(" ") + i);
		}
	}
}

mrpt::containers::yaml Viewport::asYAML() const
{
	mrpt::containers::yaml d = mrpt::containers::yaml::Sequence();

	d.asSequence().resize(m_objects.size());

	for (uint32_t i = 0; i < m_objects.size(); i++)
	{
		const auto obj = m_objects.at(i);
		mrpt::containers::yaml de = mrpt::containers::yaml::Map();

		// class-specific properties:
		obj->toYAMLMap(de);

		de["index"] = i;  // type for "i" must be a stdint type
		if (!obj)
		{
			de["class"] = "nullptr";
			continue;
		}
		de["class"] = obj->GetRuntimeClass()->className;

		// Single obj:
		if (obj->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			de["obj_children"] =
				dynamic_cast<CSetOfObjects*>(obj.get())->asYAML();
		}
		d.asSequence().at(i) = std::move(de);
	}
	return d;
}

/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void Viewport::removeObject(const CRenderizable::Ptr& obj)
{
	for (auto it = m_objects.begin(); it != m_objects.end(); ++it)
		if (*it == obj)
		{
			m_objects.erase(it);
			return;
		}
		else if (
			(*it)->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, opengl))
			dynamic_cast<CSetOfObjects*>(it->get())->removeObject(obj);
}

void Viewport::setViewportClipDistances(
	const float clip_min, const float clip_max)
{
	ASSERT_GT_(clip_max, clip_min);

	m_clip_min = clip_min;
	m_clip_max = clip_max;
}

void Viewport::getViewportClipDistances(float& clip_min, float& clip_max) const
{
	clip_min = m_clip_min;
	clip_max = m_clip_max;
}

void Viewport::setLightShadowClipDistances(
	const float clip_min, const float clip_max)
{
	ASSERT_GT_(clip_max, clip_min);

	m_lightShadowClipMin = clip_min;
	m_lightShadowClipMax = clip_max;
}

void Viewport::getLightShadowClipDistances(
	float& clip_min, float& clip_max) const
{
	clip_min = m_lightShadowClipMin;
	clip_max = m_lightShadowClipMax;
}

/*--------------------------------------------------------------
					get3DRayForPixelCoord
  ---------------------------------------------------------------*/
void Viewport::get3DRayForPixelCoord(
	const double x_coord, const double y_coord, mrpt::math::TLine3D& out_ray,
	mrpt::poses::CPose3D* out_cameraPose) const
{
	auto& _ = m_threadedData.get().state;

	if (!_.initialized)
	{
		updateMatricesFromCamera(*internalResolveActiveCamera());
		ASSERT_(_.initialized);
	}

	ASSERTDEB_(_.viewport_height > 0 && _.viewport_width > 0);

	const double ASPECT = _.viewport_width / double(_.viewport_height);

	// unitary vector between (eye) -> (pointing):
	const TPoint3D pointing_dir = {
		-cos(_.azimuth) * cos(_.elev), -sin(_.azimuth) * cos(_.elev),
		-sin(_.elev)};

	// The camera X vector (in 3D) can be computed from the camera azimuth
	// angle:
	const TPoint3D cam_x_3d = {-sin(_.azimuth), cos(_.azimuth), 0};

	// The camera real UP vector (in 3D) is the cross product:
	//     X3d x pointing_dir:
	const auto cam_up_3d = mrpt::math::crossProduct3D(cam_x_3d, pointing_dir);

	if (!_.is_projective)
	{
		// Ortho projection:
		// -------------------------------
		double Ax = _.eyeDistance * 0.25;
		double Ay = Ax;

		if (ASPECT > 1) Ax *= ASPECT;
		else
		{
			if (ASPECT != 0) Ay /= ASPECT;
		}

		const double point_lx = (-0.5 + x_coord / _.viewport_width) * 2 * Ax;
		const double point_ly = -(-0.5 + y_coord / _.viewport_height) * 2 * Ay;

		const TPoint3D ray_origin(
			_.eye.x + point_lx * cam_x_3d.x + point_ly * cam_up_3d.x,
			_.eye.y + point_lx * cam_x_3d.y + point_ly * cam_up_3d.y,
			_.eye.z + point_lx * cam_x_3d.z + point_ly * cam_up_3d.z);

		out_ray.pBase = ray_origin;
		out_ray.director[0] = pointing_dir.x;
		out_ray.director[1] = pointing_dir.y;
		out_ray.director[2] = pointing_dir.z;
	}
	else
	{
		// Perspective camera
		// -------------------------------

		// JL: This can be derived from:
		// http://www.opengl.org/sdk/docs/man/xhtml/gluPerspective.xml
		//  where one arrives to:
		//    tan(FOVx/2) = ASPECT_RATIO * tan(FOVy/2)
		//
		const double FOVy = DEG2RAD(_.FOV);
		const double FOVx = 2.0 * atan(ASPECT * tan(FOVy * 0.5));

		const auto vw = _.viewport_width;
		const auto vh = _.viewport_height;
		const double len_horz = 2.0 * (-0.5 + x_coord / vw) * tan(0.5 * FOVx);
		const double len_vert = -2.0 * (-0.5 + y_coord / vh) * tan(0.5 * FOVy);
		// Point in camera local reference frame
		const auto l = mrpt::math::TPoint3D(len_horz, len_vert, 1.0);

		const mrpt::math::TPoint3D ray_director(
			l.x * cam_x_3d.x + l.y * cam_up_3d.x + l.z * pointing_dir.x,
			l.x * cam_x_3d.y + l.y * cam_up_3d.y + l.z * pointing_dir.y,
			l.x * cam_x_3d.z + l.y * cam_up_3d.z + l.z * pointing_dir.z);

		// Set out ray:
		out_ray.pBase = _.eye;
		out_ray.director[0] = ray_director.x;
		out_ray.director[1] = ray_director.y;
		out_ray.director[2] = ray_director.z;

	}  // end projective

	// Camera pose:
	if (out_cameraPose)
	{
		mrpt::math::CMatrixDouble44 M(UNINITIALIZED_MATRIX);
		M(0, 0) = cam_x_3d.x;
		M(1, 0) = cam_x_3d.y;
		M(2, 0) = cam_x_3d.z;
		M(3, 0) = 0;

		M(0, 1) = cam_up_3d.x;
		M(1, 1) = cam_up_3d.y;
		M(2, 1) = cam_up_3d.z;
		M(3, 1) = 0;

		M(0, 2) = pointing_dir.x;
		M(1, 2) = pointing_dir.y;
		M(2, 2) = pointing_dir.z;
		M(3, 2) = 0;

		M(0, 3) = _.eye.x;
		M(1, 3) = _.eye.y;
		M(2, 3) = _.eye.z;
		M(3, 3) = 1;

		*out_cameraPose = CPose3D(M);
	}
}

void Viewport::setCurrentCameraFromPose(mrpt::poses::CPose3D& p)
{
	m_camera.set6DOFMode(true);
	m_camera.setPose(p);
}

void Viewport::getCurrentCameraPose(mrpt::poses::CPose3D& out_cameraPose) const
{
	mrpt::math::TLine3D dum;
	get3DRayForPixelCoord(0, 0, dum, &out_cameraPose);
}

/** Resets the viewport to a normal 3D viewport \sa setCloneView, setImageView
 */
void Viewport::setNormalMode()
{
	// If this was an image-mode viewport, remove the quad object to disable it.
	m_imageViewPlane.reset();

	m_isCloned = false;
	m_isClonedCamera = false;
}

void Viewport::setImageView(const mrpt::img::CImage& img)
{
	internal_enableImageView();
	m_imageViewPlane->assignImage(img);
}
void Viewport::setImageView(mrpt::img::CImage&& img)
{
	internal_enableImageView();
	m_imageViewPlane->assignImage(img);
}

void Viewport::internal_enableImageView()
{
	// If this is the first time, we have to create the quad object:
	if (!m_imageViewPlane)
	{
		m_imageViewPlane = mrpt::opengl::CTexturedPlane::Create();
		// Flip vertically:
		m_imageViewPlane->setPlaneCorners(-1, 1, 1, -1);
	}
}

/** Evaluates the bounding box of this object (including possible children) in
 * the coordinate frame of the object parent. */
auto Viewport::getBoundingBox() const -> mrpt::math::TBoundingBox
{
	mrpt::math::TBoundingBox bb;
	bool first = true;

	for (const auto& o : m_objects)
	{
		if (first)
		{
			bb = o->getBoundingBox();
			first = false;
		}
		else
			bb.unionWith(o->getBoundingBox());
	}

	return bb;
}

void Viewport::setCloneCamera(bool enable)
{
	m_isClonedCamera = enable;
	if (!enable) { m_clonedCameraViewport.clear(); }
	else
	{
		ASSERTMSG_(
			!m_clonedViewport.empty(),
			"Error: cannot setCloneCamera(true) on a viewport before calling "
			"setCloneView()");

		m_clonedCameraViewport = m_clonedViewport;
	}
}

const CCamera* Viewport::internalResolveActiveCamera(
	const CCamera* forceThisCamera) const
{
	// Prepare camera (projection matrix):
	Viewport* viewForGetCamera = nullptr;

	if (!m_clonedCameraViewport.empty())
	{
		const auto view = m_parent->getViewport(m_clonedCameraViewport);
		if (!view)
			THROW_EXCEPTION_FMT(
				"Cloned viewport '%s' not found in parent Scene",
				m_clonedViewport.c_str());

		viewForGetCamera =
			m_isClonedCamera ? view.get() : const_cast<Viewport*>(this);
	}
	else
	{  // Normal case: render our own objects:
		viewForGetCamera = const_cast<Viewport*>(this);
	}

	// Get camera:
	// 1st: if there is a CCamera in the scene (nullptr if no camera found):
	const auto camPtr = viewForGetCamera->getByClass<CCamera>();
	const auto* myCamera = camPtr ? camPtr.get() : nullptr;

	// 2nd: the internal camera of all viewports:
	if (!myCamera) myCamera = &viewForGetCamera->m_camera;

	// forced cam?
	if (forceThisCamera) myCamera = forceThisCamera;

	return myCamera;
}

void Viewport::updateMatricesFromCamera(const CCamera& myCamera) const
{
	auto& _ = m_threadedData.get().state;

	if (myCamera.isNoProjection())
	{
		// No translation nor rotation nor perspective:
		_.computeNoProjectionMatrix(m_clip_min, m_clip_max);
	}
	else
	{
		// Projective or orthogonal camera models:
		ASSERT_(myCamera.getZoomDistance() > 0);

		_.is_projective = myCamera.isProjective();

		_.FOV = myCamera.getProjectiveFOVdeg();
		_.pinhole_model = myCamera.getPinholeModel();
		_.eyeDistance = myCamera.getZoomDistance();
		_.azimuth = DEG2RAD(myCamera.getAzimuthDegrees());
		_.elev = DEG2RAD(myCamera.getElevationDegrees());

		if (myCamera.is6DOFMode())
		{
			// In 6DOFMode eye is set viewing towards the direction of the
			// positive Z axis
			// Up is set as -Y axis
			const auto pose = mrpt::poses::CPose3D(myCamera.getPose());
			const auto viewDirection =
				mrpt::poses::CPose3D::FromTranslation(0, 0, 1);
			const auto at = pose + viewDirection;

			_.eye = pose.translation();
			_.pointing = at.translation();
			// "UP" = -Y axis
			pose.getRotationMatrix().extractColumn(1, _.up);
			_.up *= -1.0;
		}
		else
		{
			// Normal mode: use "camera orbit" parameters to compute pointing-to
			// point:
			_.pointing = myCamera.getPointingAt();

			const double dis =
				std::max<double>(0.001, myCamera.getZoomDistance());
			_.eye.x = _.pointing.x + dis * cos(_.azimuth) * cos(_.elev);
			_.eye.y = _.pointing.y + dis * sin(_.azimuth) * cos(_.elev);
			_.eye.z = _.pointing.z + dis * sin(_.elev);

			_.up.x = -cos(_.azimuth) * sin(_.elev);
			_.up.y = -sin(_.azimuth) * sin(_.elev);
			_.up.z = cos(_.elev);
		}

		// Compute the projection matrix (p_matrix):
		_.computeProjectionMatrix(m_clip_min, m_clip_max);

		// Apply eye center and lookAt to p_matrix:
		_.computeViewMatrix();
	}

	// Reset model4x4 matrix to the identity transformation:
	_.m_matrix.setIdentity();

	// Compute the directional light projection matrix (light_pv)
	_.computeLightProjectionMatrix(
		m_lightShadowClipMin, m_lightShadowClipMax, m_light);

	_.initialized = true;
}

void Viewport::enableShadowCasting(
	bool enabled, unsigned int SHADOW_MAP_SIZE_X,
	unsigned int SHADOW_MAP_SIZE_Y)
{
	m_shadowsEnabled = enabled;
	if (SHADOW_MAP_SIZE_X) m_ShadowMapSizeX = SHADOW_MAP_SIZE_X;
	if (SHADOW_MAP_SIZE_Y) m_ShadowMapSizeY = SHADOW_MAP_SIZE_Y;
}
