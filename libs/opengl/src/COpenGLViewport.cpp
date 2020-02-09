/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/TLine3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/COpenGLViewport.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/CTimeLogger.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::serialization::metaprogramming;
using namespace std;

IMPLEMENTS_SERIALIZABLE(COpenGLViewport, CSerializable, mrpt::opengl)

//#define OPENGLVIEWPORT_ENABLE_TIMEPROFILING

#if defined(OPENGLVIEWPORT_ENABLE_TIMEPROFILING)
mrpt::system::CTimeLogger glv_timlog;
#endif

/*--------------------------------------------------------------

			IMPLEMENTATION OF COpenGLViewport

  ---------------------------------------------------------------*/

/*--------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
COpenGLViewport::COpenGLViewport(COpenGLScene* parent, const string& name)
	: m_camera(),
	  m_parent(parent),

	  m_clonedViewport(),
	  m_name(name),

	  m_background_color(0.6f, 0.6f, 0.6f),

	  m_imageview_img(),
	  m_objects(),

	  m_lights()
{
	// Default: one light from default direction
	m_lights.emplace_back();
	m_lights.emplace_back();

	m_lights[0].setPosition(1, 1, 1, 0);
	m_lights[0].setDirection(-1, -1, -1);

	m_lights[1].light_ID = 1;
	m_lights[1].setPosition(1, 2, -1, 0);
	m_lights[1].setDirection(1, 2, 1);

	m_lights[1].color_diffuse[0] = 0.3f;
	m_lights[1].color_diffuse[1] = 0.3f;
	m_lights[1].color_diffuse[2] = 0.3f;

	m_lights[1].color_ambient[0] = 0.3f;
	m_lights[1].color_ambient[1] = 0.3f;
	m_lights[1].color_ambient[2] = 0.3f;
}

/*--------------------------------------------------------------
					Destructor
  ---------------------------------------------------------------*/
COpenGLViewport::~COpenGLViewport() { clear(); }
/*--------------------------------------------------------------
					setCloneView
  ---------------------------------------------------------------*/
void COpenGLViewport::setCloneView(const string& clonedViewport)
{
	clear();
	m_isCloned = true;
	m_clonedViewport = clonedViewport;
}

/*--------------------------------------------------------------
					setViewportPosition
  ---------------------------------------------------------------*/
void COpenGLViewport::setViewportPosition(
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
void COpenGLViewport::getViewportPosition(
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
void COpenGLViewport::clear() { m_objects.clear(); }
/*--------------------------------------------------------------
					insert
  ---------------------------------------------------------------*/
void COpenGLViewport::insert(const CRenderizable::Ptr& newObject)
{
	m_objects.push_back(newObject);
}

// Maps [0,1] to [0,Len], wrap negative numbers, etc.
static int sizeFromRatio(
	const int startCoord, const double dSize, const int iLength)
{
	if (dSize > 1)  // >1 -> absolute pixels:
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
	return frac > 1 ? static_cast<int>(frac)
					: (frac < 0 ? static_cast<int>(dSize + frac)
								: static_cast<int>(dSize * frac));
}

// "Image mode" rendering:
void COpenGLViewport::renderImageMode() const
{
#if MRPT_HAS_OPENGL_GLUT
#if defined(OPENGLVIEWPORT_ENABLE_TIMEPROFILING)
	mrpt::system::CTimeLoggerEntry tle(
		glv_timlog, "COpenGLViewport::render imageview");
#endif
	// Do we have an actual image to render?
	if (!m_imageview_img) return;

	// Note: The following code is inspired in the implementations:
	//  - libcvd, by Edward Rosten http://www.edwardrosten.com/cvd/
	//  - PTAM, by Klein & Murray
	//  http://www.robots.ox.ac.uk/~gk/PTAM/
	mrpt::img::CImage* img = m_imageview_img.get();
	const int img_w = img->getWidth();
	const int img_h = img->getHeight();

	// Empty image?
	if (img_w == 0 || img_h == 0) return;

	// Need to adjust the aspect ratio?
	const auto vw = m_state.viewport_width, vh = m_state.viewport_height;

	const double ratio = vw * img_h / double(vh * img_w);
	double ortho_w = img_w;
	double ortho_h = img_h;
	if (ratio > 1)
		ortho_w *= ratio;
	else if (ratio != 0)
		ortho_h /= ratio;

	glOrtho(-0.5, ortho_h - 0.5, ortho_w - 0.5, -0.5, -1, 1);

	// Prepare raster pos & pixel copy direction in -Y.
	glRasterPos2f(-0.5f, -0.5f);
	glPixelZoom(vw / float(ortho_w), -vh / float(ortho_h));

	// Prepare image data types:
	const GLenum img_type = GL_UNSIGNED_BYTE;
	const int nBytesPerPixel = img->isColor() ? 3 : 1;
	// Reverse RGB <-> BGR order?
	const bool is_RGB_order = (img->getChannelsOrder() == std::string("RGB"));
	const GLenum img_format =
		nBytesPerPixel == 3 ? (is_RGB_order ? GL_RGB : GL_BGR) : GL_LUMINANCE;

	// autodetect image row alignment, if any:
	const auto row_stride = img->getRowStride();
	const auto row_bytes = img->getWidth() * nBytesPerPixel;

	ASSERT_ABOVEEQ_(row_stride, row_bytes);

	// Alignment in bytes. Refer to OpenGL docs for
	// GL_UNPACK_ALIGNMENT
	const int img_store_alignment = (row_stride - row_bytes) + 1;
	ASSERT_(
		img_store_alignment == 1 || img_store_alignment == 2 ||
		img_store_alignment == 4 || img_store_alignment == 8);

	// Send image data to OpenGL:
	glPixelStorei(GL_UNPACK_ALIGNMENT, img_store_alignment);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, img->getWidth());
	glDrawPixels(img_w, img_h, img_format, img_type, img->ptrLine<uint8_t>(0));
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset
	CHECK_OPENGL_ERROR();
#endif
}

void COpenGLViewport::loadDefaultShaders() const
{
#if MRPT_HAS_OPENGL_GLUT
	// Vertex shader:
	const char* DEFAULT_VERTEX_SHADER_CODE =
#include "default_vertex_shader.glsl"
		;

	// Fragment shader:
	const char* DEFAULT_FRAGMENT_SHADER_CODE =
#include "default_fragment_shader.glsl"
		;

	if (!m_shaders) m_shaders = std::make_shared<Program>();
	m_shaders->clear();

	std::string errMsgs;
	std::vector<Shader> lstShaders;
	lstShaders.resize(2);
	if (!lstShaders[0].compileFromSource(
			GL_VERTEX_SHADER, DEFAULT_VERTEX_SHADER_CODE, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error compiling GL_VERTEX_SHADER:\n%s", errMsgs.c_str());
	}
	if (!lstShaders[1].compileFromSource(
			GL_FRAGMENT_SHADER, DEFAULT_FRAGMENT_SHADER_CODE, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error compiling GL_FRAGMENT_SHADER:\n%s", errMsgs.c_str());
	}
	if (!m_shaders->linkProgram(lstShaders, errMsgs))
	{
		THROW_EXCEPTION_FMT(
			"Error linking Opengl Shader programs:\n%s", errMsgs.c_str());
	}

	// Uniforms:
	m_shaders->declareUniform("p_matrix");
	m_shaders->declareUniform("mv_matrix");

	// Attributes:
	m_shaders->declareAttribute("position");
	m_shaders->declareAttribute("vertexColor");
	m_shaders->declareAttribute("vertexNormal");

#endif
}

/** Render a normal scene with 3D objects */
void COpenGLViewport::renderNormalSceneMode() const
{
#if MRPT_HAS_OPENGL_GLUT
	// Prepare shaders upon first invokation:
	if (!m_shaders || m_shaders->empty()) loadDefaultShaders();

	// Prepare camera (projection matrix):
	const CListOpenGLObjects* objectsToRender = nullptr;
	COpenGLViewport* viewForGetCamera = nullptr;

	if (m_isCloned)
	{  // Clone: render someone's else objects.
		ASSERT_(m_parent.get() != nullptr);

		COpenGLViewport::Ptr view = m_parent->getViewport(m_clonedViewport);
		if (!view)
			THROW_EXCEPTION_FMT(
				"Cloned viewport '%s' not found in parent COpenGLScene",
				m_clonedViewport.c_str());

		objectsToRender = &view->m_objects;
		viewForGetCamera =
			m_isClonedCamera ? view.get() : const_cast<COpenGLViewport*>(this);
	}
	else
	{  // Normal case: render our own objects:
		objectsToRender = &m_objects;
		viewForGetCamera = const_cast<COpenGLViewport*>(this);
	}

	// Get camera:
	// 1st: if there is a CCamera in the scene (nullptr if no camera found):
	const CCamera* myCamera =
		dynamic_cast<CCamera*>(viewForGetCamera->getByClass<CCamera>().get());

	// 2nd: the internal camera of all viewports:
	if (!myCamera) myCamera = &viewForGetCamera->m_camera;

	ASSERT_(m_camera.m_eyeDistance > 0);

	auto& _ = m_state;

	_.is_projective = myCamera->m_projectiveModel;
	_.FOV = myCamera->m_projectiveFOVdeg;
	_.eyeDistance = myCamera->m_eyeDistance;
	_.azimuth = DEG2RAD(myCamera->m_azimuthDeg);
	_.elev = DEG2RAD(myCamera->m_elevationDeg);

	if (myCamera->is6DOFMode())
	{
		// In 6DOFMode eye is set viewing towards the direction of the
		// positive Z axis
		// Up is set as Y axis
		mrpt::poses::CPose3D viewDirection, pose, at;
		viewDirection.z(+1);
		pose = mrpt::poses::CPose3D(myCamera->getPose());
		at = pose + viewDirection;

		_.eye.x = pose.x();
		_.eye.y = pose.y();
		_.eye.z = pose.z();
		_.pointing.x = at.x();
		_.pointing.y = at.y();
		_.pointing.z = at.z();
		_.up.x = pose.getRotationMatrix()(0, 1);
		_.up.y = pose.getRotationMatrix()(1, 1);
		_.up.z = pose.getRotationMatrix()(2, 1);
	}
	else
	{
		// Normal mode: use "camera orbit" parameters to compute pointing-to
		// point:
		const float dis = max(0.01f, myCamera->m_eyeDistance);
		_.eye.x = _.pointing.x + dis * cos(_.azimuth) * cos(_.elev);
		_.eye.y = _.pointing.y + dis * sin(_.azimuth) * cos(_.elev);
		_.eye.z = _.pointing.z + dis * sin(_.elev);

		_.pointing.x = myCamera->m_pointingX;
		_.pointing.y = myCamera->m_pointingY;
		_.pointing.z = myCamera->m_pointingZ;

		if (fabs(fabs(myCamera->m_elevationDeg) - 90) > 1e-6)
		{
			_.up.x = 0;
			_.up.y = 0;
			_.up.z = 1;
		}
		else
		{
			float sgn = myCamera->m_elevationDeg > 0 ? 1 : -1;
			_.up.x = -cos(_.azimuth) * sgn;
			_.up.y = -sin(_.azimuth) * sgn;
			_.up.z = 0;
		}
	}

	// Compute the projection matrix (p_matrix):
	_.computeProjectionMatrix(m_clip_min, m_clip_max);

	// Apply eye center and lookAt to p_matrix:
	_.applyLookAt();

	// Optional pre-Render user code:
	if (hasSubscribers())
	{
		mrptEventGLPreRender ev(this);
		this->publishEvent(ev);
	}

	// Global OpenGL settings:
	// ---------------------------------
	glHint(
		GL_POLYGON_SMOOTH_HINT,
		m_OpenGL_enablePolygonNicest ? GL_NICEST : GL_FASTEST);
	CHECK_OPENGL_ERROR();

	// Reset model-view 4x4 matrix to the identity transformation:
	_.mv_matrix.setIdentity();

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);  // GL_LESS
	CHECK_OPENGL_ERROR();

	MRPT_TODO("Port Lights!");
	// for (const auto& m_light : m_lights) m_light.sendToOpenGL();

	// Render all the objects:
	mrpt::opengl::gl_utils::renderSetOfObjects(*objectsToRender, _, *m_shaders);

#endif
}

void COpenGLViewport::renderViewportBorder() const
{
#if MRPT_HAS_OPENGL_GLUT
	if (m_borderWidth < 1) return;

	glLineWidth(2 * m_borderWidth);
	glColor4f(0, 0, 0, 1);
	glDisable(GL_DEPTH_TEST);
	CHECK_OPENGL_ERROR();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINE_LOOP);
	glVertex2f(-1, -1);
	glVertex2f(-1, 1);
	glVertex2f(1, 1);
	glVertex2f(1, -1);
	glEnd();

	glEnable(GL_DEPTH_TEST);
#endif
}

/*---------------------------------------------------------------
						render
 ---------------------------------------------------------------*/
void COpenGLViewport::render(
	const int render_width, const int render_height) const
{
#if MRPT_HAS_OPENGL_GLUT
	// Declared here for usage in the "catch"
	const CRenderizable* it = nullptr;
	try
	{
		// Change viewport:
		// -------------------------------------------
		const GLint vx = startFromRatio(m_view_x, render_width);
		const GLint vy = startFromRatio(m_view_y, render_height);
		const GLint vw = sizeFromRatio(vx, m_view_width, render_width);
		const GLint vh = sizeFromRatio(vy, m_view_height, render_height);

		glViewport(vx, vy, vw, vh);
		CHECK_OPENGL_ERROR();

		// Clear depth&/color buffers:
		// -------------------------------------------
		m_state.viewport_width = vw;
		m_state.viewport_height = vh;

		glScissor(vx, vy, vw, vh);
		CHECK_OPENGL_ERROR();

		glEnable(GL_SCISSOR_TEST);
		CHECK_OPENGL_ERROR();

		if (!m_isTransparent)
		{  // Clear color & depth buffers:
			// Save?
			GLdouble prevCol[4];
			if (m_custom_backgb_color)
			{
				glGetDoublev(GL_COLOR_CLEAR_VALUE, prevCol);
				CHECK_OPENGL_ERROR();
				glClearColor(
					m_background_color.R, m_background_color.G,
					m_background_color.B, m_background_color.A);
				CHECK_OPENGL_ERROR();
			}

			glClear(
				GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |
				GL_STENCIL_BUFFER_BIT);
			CHECK_OPENGL_ERROR();

			// Restore old colors:
			if (m_custom_backgb_color)
			{
				glClearColor(prevCol[0], prevCol[1], prevCol[2], prevCol[3]);
				CHECK_OPENGL_ERROR();
			}
		}
		else
		{  // Clear depth buffer only:
			glClear(GL_DEPTH_BUFFER_BIT);
			CHECK_OPENGL_ERROR();
		}
		glDisable(GL_SCISSOR_TEST);
		CHECK_OPENGL_ERROR();

		// If we are in "image mode", rendering is much simpler: just set
		//  ortho projection and render the image quad:
		if (m_isImageView)
			renderImageMode();
		else
			renderNormalSceneMode();

		// Finally, draw the border:
		renderViewportBorder();

		// Optional post-Render user code:
		if (hasSubscribers())
		{
			mrptEventGLPostRender ev(this);
			this->publishEvent(ev);
		}
	}
	catch (const exception& e)
	{
		const auto sErr = mrpt::exception_to_str(e);
		string msg;
		if (it != nullptr)
			msg = format(
				"Exception while rendering a class '%s'\n%s",
				it->GetRuntimeClass()->className, sErr.c_str());
		else
			msg = format("Exception while rendering:\n%s", sErr.c_str());

		THROW_EXCEPTION(msg);
	}
	catch (...)
	{
		THROW_EXCEPTION("Runtime error!");
	}
#else
	MRPT_UNUSED_PARAM(render_width);
	MRPT_UNUSED_PARAM(render_height);
	THROW_EXCEPTION(
		"The MRPT has been compiled with MRPT_HAS_OPENGL_GLUT=0! OpenGL "
		"functions are not implemented");
#endif
}

uint8_t COpenGLViewport::serializeGetVersion() const { return 3; }
void COpenGLViewport::serializeTo(mrpt::serialization::CArchive& out) const
{
	// Save data:
	out << m_camera << m_isCloned << m_isClonedCamera << m_clonedViewport
		<< m_name << m_isTransparent << m_borderWidth << m_view_x << m_view_y
		<< m_view_width << m_view_height;

	// Added in v1:
	out << m_custom_backgb_color << m_background_color.R << m_background_color.G
		<< m_background_color.B << m_background_color.A;

	// Save objects:
	uint32_t n;
	n = (uint32_t)m_objects.size();
	out << n;
	for (const auto& m_object : m_objects) out << *m_object;

	// Added in v2: Global OpenGL settings:
	out << m_OpenGL_enablePolygonNicest;

	// Added in v3: Lights
	out << m_lights;
}

void COpenGLViewport::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			// Load data:
			in >> m_camera >> m_isCloned >> m_isClonedCamera >>
				m_clonedViewport >> m_name >> m_isTransparent >>
				m_borderWidth >> m_view_x >> m_view_y >> m_view_width >>
				m_view_height;

			// in v1:
			if (version >= 1)
			{
				in >> m_custom_backgb_color >> m_background_color.R >>
					m_background_color.G >> m_background_color.B >>
					m_background_color.A;
			}
			else
			{
				m_custom_backgb_color = false;
			}

			// Load objects:
			uint32_t n;
			in >> n;
			clear();
			m_objects.resize(n);

			for_each(
				m_objects.begin(), m_objects.end(), ObjectReadFromStream(&in));

			// Added in v2: Global OpenGL settings:
			if (version >= 2)
			{
				in >> m_OpenGL_enablePolygonNicest;
			}
			else
			{
				// Defaults
			}

			// Added in v3: Lights
			if (version >= 3)
				in >> m_lights;
			else
			{
				// Default: one light from default direction
				m_lights.clear();
				m_lights.emplace_back();
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
							getByName
  ---------------------------------------------------------------*/
CRenderizable::Ptr COpenGLViewport::getByName(const string& str)
{
	for (auto& m_object : m_objects)
	{
		if (m_object->m_name == str)
			return m_object;
		else if (
			m_object->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, opengl))
		{
			CRenderizable::Ptr ret =
				std::dynamic_pointer_cast<CSetOfObjects>(m_object)->getByName(
					str);
			if (ret) return ret;
		}
	}
	return CRenderizable::Ptr();
}

/*---------------------------------------------------------------
					initializeAllTextures
  ---------------------------------------------------------------*/
void COpenGLViewport::initializeAllTextures()
{
#if MRPT_HAS_OPENGL_GLUT
	for (auto& obj : m_objects)
	{
		if (IS_DERIVED(*obj, CTexturedObject))
			std::dynamic_pointer_cast<CTexturedObject>(obj)
				->loadTextureInOpenGL();
		else if (IS_CLASS(*obj, CSetOfObjects))
			std::dynamic_pointer_cast<CSetOfObjects>(obj)
				->initializeAllTextures();
	}
#endif
}

void COpenGLViewport::dumpListOfObjects(std::vector<std::string>& lst)
{
	for (auto& m_object : m_objects)
	{
		// Single obj:
		string s(m_object->GetRuntimeClass()->className);
		if (m_object->m_name.size())
			s += string(" (") + m_object->m_name + string(")");
		lst.emplace_back(s);

		if (m_object->GetRuntimeClass() ==
			CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::opengl))
		{
			std::vector<std::string> auxLst;

			dynamic_cast<CSetOfObjects*>(m_object.get())
				->dumpListOfObjects(auxLst);

			for (const auto& i : auxLst) lst.emplace_back(string(" ") + i);
		}
	}
}

/*--------------------------------------------------------------
					removeObject
  ---------------------------------------------------------------*/
void COpenGLViewport::removeObject(const CRenderizable::Ptr& obj)
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

/*--------------------------------------------------------------
					setViewportClipDistances
  ---------------------------------------------------------------*/
void COpenGLViewport::setViewportClipDistances(
	const double clip_min, const double clip_max)
{
	ASSERT_(clip_max > clip_min);

	m_clip_min = clip_min;
	m_clip_max = clip_max;
}

/*--------------------------------------------------------------
					getViewportClipDistances
  ---------------------------------------------------------------*/
void COpenGLViewport::getViewportClipDistances(
	double& clip_min, double& clip_max) const
{
	clip_min = m_clip_min;
	clip_max = m_clip_max;
}

/*--------------------------------------------------------------
					get3DRayForPixelCoord
  ---------------------------------------------------------------*/
void COpenGLViewport::get3DRayForPixelCoord(
	const double x_coord, const double y_coord, mrpt::math::TLine3D& out_ray,
	mrpt::poses::CPose3D* out_cameraPose) const
{
	ASSERTDEB_(m_state.viewport_height > 0 && m_state.viewport_width > 0);

	const double ASPECT =
		m_state.viewport_width / double(m_state.viewport_height);

	// unitary vector between (eye) -> (pointing):
	TPoint3D pointing_dir;
	pointing_dir.x = -cos(m_state.azimuth) * cos(m_state.elev);
	pointing_dir.y = -sin(m_state.azimuth) * cos(m_state.elev);
	pointing_dir.z = -sin(m_state.elev);

	// The camera X vector (in 3D) can be computed from the camera azimuth
	// angle:
	TPoint3D cam_x_3d;
	cam_x_3d.x = -sin(m_state.azimuth);
	cam_x_3d.y = cos(m_state.azimuth);
	cam_x_3d.z = 0;

	// The camera real UP vector (in 3D) is the cross product:
	//     X3d x pointing_dir:
	TPoint3D cam_up_3d;
	crossProduct3D(cam_x_3d, pointing_dir, cam_up_3d);

	if (!m_state.is_projective)
	{
		// Ortho projection:
		// -------------------------------
		double Ax = m_state.eyeDistance * 0.5;
		double Ay = Ax;

		if (ASPECT > 1)
			Ax *= ASPECT;
		else
		{
			if (ASPECT != 0) Ay /= ASPECT;
		}

		const double point_lx =
			(-0.5 + x_coord / m_state.viewport_width) * 2 * Ax;
		const double point_ly =
			-(-0.5 + y_coord / m_state.viewport_height) * 2 * Ay;

		const TPoint3D ray_origin(
			m_state.eye.x + point_lx * cam_x_3d.x + point_ly * cam_up_3d.x,
			m_state.eye.y + point_lx * cam_x_3d.y + point_ly * cam_up_3d.y,
			m_state.eye.z + point_lx * cam_x_3d.z + point_ly * cam_up_3d.z);

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
		const double FOVy = DEG2RAD(m_state.FOV);
		const double FOVx = 2.0 * atan(ASPECT * tan(FOVy * 0.5));

		const auto vw = m_state.viewport_width;
		const auto vh = m_state.viewport_height;
		const double len_horz = 2.0 * (-0.5 + x_coord / vw) * tan(0.5 * FOVx);
		const double len_vert = -2.0 * (-0.5 + y_coord / vh) * tan(0.5 * FOVy);
		// Point in camera local reference frame
		const auto l = mrpt::math::TPoint3D(len_horz, len_vert, 1.0);

		const mrpt::math::TPoint3D ray_director(
			l.x * cam_x_3d.x + l.y * cam_up_3d.x + l.z * pointing_dir.x,
			l.x * cam_x_3d.y + l.y * cam_up_3d.y + l.z * pointing_dir.y,
			l.x * cam_x_3d.z + l.y * cam_up_3d.z + l.z * pointing_dir.z);

		// Set out ray:
		out_ray.pBase = m_state.eye;
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

		M(0, 3) = m_state.eye.x;
		M(1, 3) = m_state.eye.y;
		M(2, 3) = m_state.eye.z;
		M(3, 3) = 1;

		*out_cameraPose = CPose3D(M);
	}
}

MRPT_TODO("Implement a setCurrentCameraFromPose() method")

void COpenGLViewport::getCurrentCameraPose(
	mrpt::poses::CPose3D& out_cameraPose) const
{
	mrpt::math::TLine3D dum;
	get3DRayForPixelCoord(0, 0, dum, &out_cameraPose);
}

/** Resets the viewport to a normal 3D viewport \sa setCloneView, setImageView
 */
void COpenGLViewport::setNormalMode()
{
	// If this was a m_isImageView, remove the quad object:
	if (m_isImageView && m_imageview_img) m_imageview_img.reset();

	m_isCloned = false;
	m_isClonedCamera = false;
	m_isImageView = false;
}

void COpenGLViewport::setImageView(const mrpt::img::CImage& img)
{
	internal_enableImageView();
	*m_imageview_img = img;
}
void COpenGLViewport::setImageView(mrpt::img::CImage&& img)
{
	internal_enableImageView();
	*m_imageview_img = std::move(img);
}

void COpenGLViewport::internal_enableImageView()
{
	// If this is the first time, we have to create the quad object:
	if (!m_isImageView || !m_imageview_img)
		m_imageview_img = mrpt::img::CImage::Create();
	m_isImageView = true;
}

/** Evaluates the bounding box of this object (including possible children) in
 * the coordinate frame of the object parent. */
void COpenGLViewport::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& m_object : m_objects)
	{
		TPoint3D child_bbmin(
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max());
		TPoint3D child_bbmax(
			-std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max());
		m_object->getBoundingBox(child_bbmin, child_bbmax);

		keep_min(bb_min.x, child_bbmin.x);
		keep_min(bb_min.y, child_bbmin.y);
		keep_min(bb_min.z, child_bbmin.z);

		keep_max(bb_max.x, child_bbmax.x);
		keep_max(bb_max.y, child_bbmax.y);
		keep_max(bb_max.z, child_bbmax.z);
	}
}
