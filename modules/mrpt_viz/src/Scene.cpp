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

#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>
#include <mrpt/viz/Scene.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::serialization::metaprogramming;
using namespace mrpt::math;
using namespace std;

// Include libraries in linking:
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
#ifdef _WIN32
// WINDOWS:
#if defined(_MSC_VER)
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GlU32.lib")
#endif
#endif  // _WIN32
#endif  // MRPT_HAS_OPENGL_GLUT

IMPLEMENTS_SERIALIZABLE(Scene, CSerializable, mrpt::viz)

/*---------------------------------------------------------------
            Constructor
---------------------------------------------------------------*/
Scene::Scene() { createViewport("main"); }
/*--------------------------------------------------------------
          Copy constructor
  ---------------------------------------------------------------*/
Scene::Scene(const Scene& obj) : CSerializable() { (*this) = obj; }

Scene::~Scene() { m_viewports.clear(); }

void Scene::clear(bool createMainViewport)
{
  m_viewports.clear();

  if (createMainViewport)
  {
    createViewport("main");
  }
}

/*---------------------------------------------------------------
              =
 ---------------------------------------------------------------*/
Scene& Scene::operator=(const Scene& obj)
{
  if (this != &obj)
  {
    m_followCamera = obj.m_followCamera;

    clear();
    m_viewports = obj.m_viewports;
    for_each(
        m_viewports.begin(), m_viewports.end(),
        [](auto& ptr)
        {
          // make a unique copy of each object (copied as a shared ptr)
          ptr.reset(dynamic_cast<mrpt::viz::Viewport*>(ptr->clone()));
        });
  }
  return *this;
}

uint8_t Scene::serializeGetVersion() const { return 1; }
void Scene::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << m_followCamera;

  uint32_t n;
  n = (uint32_t)m_viewports.size();
  out << n;
  for (const auto& m_viewport : m_viewports)
  {
    out << *m_viewport;
  }
}

void Scene::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      // Old style: Just one viewport:
      clear(true);
      Viewport::Ptr view = m_viewports[0];

      // Load objects:
      uint32_t n;
      in >> n;

      view->clear();
      view->m_objects.resize(n);
      for_each(view->m_objects.begin(), view->m_objects.end(), ObjectReadFromStream(&in));
    }
    break;
    case 1:
    {
      in >> m_followCamera;

      uint32_t i, n;
      in >> n;
      clear(false);

      for (i = 0; i < n; i++)
      {
        CSerializable::Ptr newObj;
        in >> newObj;

        Viewport::Ptr newView = std::dynamic_pointer_cast<Viewport>(newObj);
        newView->m_parent = this;
        m_viewports.push_back(newView);
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

/*---------------------------------------------------------------
              insert
  ---------------------------------------------------------------*/
void Scene::insert(const CVisualObject::Ptr& newObject, const std::string& viewportName)
{
  MRPT_START
  for (auto& m_viewport : m_viewports)
  {
    if (m_viewport->m_name == viewportName)
    {
      m_viewport->insert(newObject);
      return;
    }
  }
  THROW_EXCEPTION_FMT("Error: viewport '%s' not found.", viewportName.c_str());
  MRPT_END
}

/*---------------------------------------------------------------
              getByName
  ---------------------------------------------------------------*/
CVisualObject::Ptr Scene::getByName(const string& str, [[maybe_unused]] const string& viewportName)
{
  CVisualObject::Ptr obj;
  for (auto& m_viewport : m_viewports)
    if ((obj = m_viewport->getByName(str))) break;
  return obj;
}

/*--------------------------------------------------------------
          dumpListOfObjects
  ---------------------------------------------------------------*/
void Scene::dumpListOfObjects(std::vector<std::string>& lst) const
{
  using namespace std::string_literals;
  lst.clear();

  for (auto& v : m_viewports)
  {
    lst.emplace_back("Viewport: '"s + v->m_name + "'"s);
    lst.emplace_back("============================================");
    v->dumpListOfObjects(lst);
  }
}

mrpt::containers::yaml Scene::asYAML() const
{
  mrpt::containers::yaml d = mrpt::containers::yaml::Map();
  auto vs = d["viewports"];

  for (auto& v : m_viewports) vs[v->m_name] = v->asYAML();

  return d;
}

/*--------------------------------------------------------------
          createViewport
  ---------------------------------------------------------------*/
Viewport::Ptr Scene::createViewport(const string& viewportName)
{
  MRPT_START

  Viewport::Ptr old = getViewport(viewportName);
  if (old) return old;

  auto theNew = std::make_shared<Viewport>(this, viewportName);
  m_viewports.push_back(theNew);
  return theNew;

  MRPT_END
}

/*--------------------------------------------------------------
          getViewport
  ---------------------------------------------------------------*/
Viewport::Ptr Scene::getViewport(const std::string& viewportName) const
{
  MRPT_START
  for (const auto& m_viewport : m_viewports)
  {
    if (m_viewport->m_name == viewportName)
    {
      return m_viewport;
    }
  }
  return {};
  MRPT_END
}

/*--------------------------------------------------------------
          removeObject
  ---------------------------------------------------------------*/
void Scene::removeObject(const CVisualObject::Ptr& obj, const std::string& viewportName)
{
  MRPT_START

  Viewport::Ptr view = getViewport(viewportName);
  ASSERT_(view);
  view->removeObject(obj);

  MRPT_END
}

bool Scene::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  bool found = false;
  double tmp = 0;
  for (const auto& vp : m_viewports)
  {
    for (auto it2 = vp->m_objects.begin(); it2 != vp->m_objects.end(); ++it2)
    {
      if ((*it2)->traceRay(o, tmp))
      {
        if (!found)
        {
          found = true;
          dist = tmp;
        }
        else if (tmp < dist)
        {
          dist = tmp;
        }
      }
    }
  }
  return found;
}

bool Scene::saveToFile(const std::string& fil) const
{
  try
  {
    mrpt::io::CCompressedOutputStream f(fil);
    mrpt::serialization::archiveFrom(f) << *this;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

bool Scene::loadFromFile(const std::string& fil)
{
  try
  {
    mrpt::io::CCompressedInputStream f(fil);
    mrpt::serialization::archiveFrom(f) >> *this;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[mrpt::viz::Scene] Error loading '" << fil << "': " << e.what() << "\n";
    return false;
  }
}

/** Evaluates the bounding box of this object (including possible children) in
 * the coordinate frame of the object parent. */
auto Scene::getBoundingBox(const std::string& vpn) const -> mrpt::math::TBoundingBox
{
  Viewport::Ptr vp = this->getViewport(vpn);
  ASSERTMSG_(vp, "No opengl viewport exists with the given name");

  return vp->getBoundingBox();
}

Scene::Ptr& mrpt::viz::operator<<(Scene::Ptr& s, const CVisualObject::Ptr& r)
{
  s->insert(r);
  return s;
}
