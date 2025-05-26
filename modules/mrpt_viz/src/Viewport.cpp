/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "viz-precomp.h"  // Precompiled header
//
#include <Eigen/Dense>  // First! to avoid conflicts with X.h
//
#include <mrpt/core/get_env.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>  // crossProduct3D()
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace mrpt::serialization::metaprogramming;
using namespace std;

IMPLEMENTS_SERIALIZABLE(Viewport, CSerializable, mrpt::viz)

/*--------------------------------------------------------------

      IMPLEMENTATION OF Viewport

  ---------------------------------------------------------------*/

/*--------------------------------------------------------------
          Constructor
  ---------------------------------------------------------------*/
Viewport::Viewport(Scene* parent, const string& name) : m_parent(parent), m_name(name) {}

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

  m_view_x = x;
  m_view_y = y;
  m_view_width = width;
  m_view_height = height;

  MRPT_END
}

/*--------------------------------------------------------------
          getViewportPosition
  ---------------------------------------------------------------*/
void Viewport::getViewportPosition(double& x, double& y, double& width, double& height)
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
void Viewport::insert(const CVisualObject::Ptr& newObject) { m_objects.push_back(newObject); }

uint8_t Viewport::serializeGetVersion() const { return 10; }
void Viewport::serializeTo(mrpt::serialization::CArchive& out) const
{
  // Save data:
  out << m_camera << m_isCloned << m_isClonedCamera << m_clonedViewport << m_name << m_isTransparent
      << m_borderWidth << m_view_x << m_view_y << m_view_width << m_view_height;

  // Added in v1:
  out << m_background_color.R << m_background_color.G << m_background_color.B
      << m_background_color.A;

  // Save objects:
  uint32_t n;
  n = (uint32_t)m_objects.size();
  out << n;
  for (const auto& m_object : m_objects) out << *m_object;

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

    out << fp.vfont_name << fp.vfont_scale << fp.color << fp.draw_shadow << fp.shadow_color
        << fp.vfont_spacing << fp.vfont_kerning;
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
  out << m_clip_max << m_clip_min << m_lightShadowClipMin << m_lightShadowClipMax;

  // v10:
  out << m_isViewportVisible;
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
    case 10:
    {
      // Load data:
      in >> m_camera >> m_isCloned >> m_isClonedCamera >> m_clonedViewport >> m_name >>
          m_isTransparent >> m_borderWidth >> m_view_x >> m_view_y >> m_view_width >> m_view_height;

      // in v1:
      if (version >= 1)
      {
        if (version < 7)
        {
          // field removed in v7:
          bool was_m_custom_backgb_color;
          in >> was_m_custom_backgb_color;
        }

        in >> m_background_color.R >> m_background_color.G >> m_background_color.B >>
            m_background_color.A;
      }

      // Load objects:
      uint32_t n;
      in >> n;
      clear();
      m_objects.resize(n);

      for_each(m_objects.begin(), m_objects.end(), ObjectReadFromStream(&in));

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
        in >> m_light;
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

        in >> fp.vfont_name >> fp.vfont_scale >> fp.color >> fp.draw_shadow >> fp.shadow_color >>
            fp.vfont_spacing >> fp.vfont_kerning;
        fp.vfont_style = static_cast<TOpenGLFontStyle>(in.ReadAs<uint8_t>());

        this->addTextMessage(x, y, text, id, fp);
      }

      // Added in v5: image mode
      if (in.ReadAs<bool>())
      {
        in >> m_imageViewPlane;
      }
      else
      {
        m_imageViewPlane.reset();
      }

      if (version >= 6)
        in >> m_clonedCameraViewport;
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
        in >> m_clip_max >> m_clip_min >> m_lightShadowClipMin >> m_lightShadowClipMax;
      }

      if (version >= 10)
      {
        in >> m_isViewportVisible;
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
CVisualObject::Ptr Viewport::getByName(const string& str)
{
  for (auto& m_object : m_objects)
  {
    if (m_object->getName() == str)
      return m_object;
    else if (m_object->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::viz))
    {
      if (CVisualObject::Ptr ret =
              std::dynamic_pointer_cast<CSetOfObjects>(m_object)->getByName(str);
          ret)
        return ret;
    }
  }
  return {};
}

void Viewport::dumpListOfObjects(std::vector<std::string>& lst) const
{
  for (auto& obj : m_objects)
  {
    // Single obj:
    string s(obj->GetRuntimeClass()->className);
    if (!obj->getName().empty()) s += string(" (") + obj->getName() + string(")");
    lst.emplace_back(s);

    if (obj->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::viz))
    {
      std::vector<std::string> auxLst;

      dynamic_cast<CSetOfObjects*>(obj.get())->dumpListOfObjects(auxLst);

      for (const auto& i : auxLst) lst.emplace_back(string(" ") + i);
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
    if (obj->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::viz))
    {
      de["obj_children"] = dynamic_cast<CSetOfObjects*>(obj.get())->asYAML();
    }
    d.asSequence().at(i) = std::move(de);
  }
  return d;
}

/*--------------------------------------------------------------
          removeObject
  ---------------------------------------------------------------*/
void Viewport::removeObject(const CVisualObject::Ptr& obj)
{
  for (auto it = m_objects.begin(); it != m_objects.end(); ++it)
    if (*it == obj)
    {
      m_objects.erase(it);
      return;
    }
    else if ((*it)->GetRuntimeClass() == CLASS_ID_NAMESPACE(CSetOfObjects, mrpt::viz))
      dynamic_cast<CSetOfObjects*>(it->get())->removeObject(obj);
}

void Viewport::setViewportClipDistances(const float clip_min, const float clip_max)
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

void Viewport::setLightShadowClipDistances(const float clip_min, const float clip_max)
{
  ASSERT_GT_(clip_max, clip_min);

  m_lightShadowClipMin = clip_min;
  m_lightShadowClipMax = clip_max;
}

void Viewport::getLightShadowClipDistances(float& clip_min, float& clip_max) const
{
  clip_min = m_lightShadowClipMin;
  clip_max = m_lightShadowClipMax;
}

void Viewport::setCurrentCameraFromPose(mrpt::poses::CPose3D& p)
{
  m_camera.set6DOFMode(true);
  m_camera.setPose(p);
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

void Viewport::setImageView(const mrpt::img::CImage& img, bool transparentBackground)
{
  internal_enableImageView(transparentBackground);
  m_imageViewPlane->assignImage(img);
}
void Viewport::setImageView(mrpt::img::CImage&& img, bool transparentBackground)
{
  internal_enableImageView(transparentBackground);
  m_imageViewPlane->assignImage(img);
}

void Viewport::internal_enableImageView(bool transparentBackground)
{
  // If this is the first time, we have to create the quad object:
  if (!m_imageViewPlane)
  {
    m_imageViewPlane = mrpt::viz::CTexturedPlane::Create();
    // Flip vertically:
    m_imageViewPlane->setPlaneCorners(-1, 1, 1, -1);
  }
  setTransparent(transparentBackground);
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
      bb = bb.unionWith(o->getBoundingBox());
  }

  return bb;
}

void Viewport::setCloneCamera(bool enable)
{
  m_isClonedCamera = enable;
  if (!enable)
  {
    m_clonedCameraViewport.clear();
  }
  else
  {
    ASSERTMSG_(
        !m_clonedViewport.empty(),
        "Error: cannot setCloneCamera(true) on a viewport before calling "
        "setCloneView()");

    m_clonedCameraViewport = m_clonedViewport;
  }
}

const CCamera* Viewport::internalResolveActiveCamera(const CCamera* forceThisCamera) const
{
  // Prepare camera (projection matrix):
  Viewport* viewForGetCamera = nullptr;

  if (!m_clonedCameraViewport.empty())
  {
    const auto view = m_parent->getViewport(m_clonedCameraViewport);
    if (!view)
      THROW_EXCEPTION_FMT(
          "Cloned viewport '%s' not found in parent Scene", m_clonedViewport.c_str());

    viewForGetCamera = m_isClonedCamera ? view.get() : const_cast<Viewport*>(this);
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

void Viewport::enableShadowCasting(
    bool enabled, unsigned int SHADOW_MAP_SIZE_X, unsigned int SHADOW_MAP_SIZE_Y)
{
  m_shadowsEnabled = enabled;
  if (SHADOW_MAP_SIZE_X) m_ShadowMapSizeX = SHADOW_MAP_SIZE_X;
  if (SHADOW_MAP_SIZE_Y) m_ShadowMapSizeY = SHADOW_MAP_SIZE_Y;
}

Viewport::Ptr& mrpt::viz::operator<<(Viewport::Ptr& s, const CVisualObject::Ptr& r)
{
  s->insert(r);
  return s;
}

Viewport::Ptr& mrpt::viz::operator<<(Viewport::Ptr& s, const std::vector<CVisualObject::Ptr>& v)
{
  for (const auto& it : v) s->insert(it);
  return s;
}
