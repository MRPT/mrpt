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
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/LinesProxy.h>
#include <mrpt/opengl/PointsProxy.h>
#include <mrpt/opengl/SkyBoxProxy.h>
#include <mrpt/opengl/TexturedTrianglesProxy.h>
#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CLight.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>

#include <Eigen/Dense>
#include <unordered_map>

#include "gltext.h"

using namespace mrpt::opengl;
using namespace mrpt::viz;

// ============================================================================
// Text3DProxy: generates text geometry from CText3D using the gltext system
// ============================================================================
namespace
{
class Text3DProxy : public TrianglesProxyBase
{
 public:
  void compile(const mrpt::viz::CVisualObject* sourceObj) override
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    MRPT_START

    const auto* text3d = dynamic_cast<const mrpt::viz::CText3D*>(sourceObj);
    if (text3d == nullptr)
    {
      return;
    }

    m_lightEnabled = false;
    m_cullFace = mrpt::viz::TCullFace::NONE;

    // Generate text geometry using gltext
    std::vector<mrpt::viz::TTriangle> tris;
    std::vector<mrpt::math::TPoint3Df> lineVerts;
    std::vector<mrpt::img::TColor> lineColors;

    internal::glSetFont(text3d->getFont());
    // Use scale=1.0 here; the actual scale is applied via the model matrix
    // (CVisualObject::setScale), so we don't want to bake it into geometry.
    internal::glDrawTextTransformed(
        text3d->getString(), tris, lineVerts, lineColors, mrpt::poses::CPose3D(), 1.0f,
        text3d->getColor_u8(), text3d->getTextStyle(), text3d->setTextSpacing(),
        text3d->setTextKerning());

    m_triangleCount = tris.size();
    if (m_triangleCount == 0)
    {
      m_localBBox.reset();
      return;
    }

    const size_t vertexCount = m_triangleCount * 3;

    std::vector<mrpt::math::TPoint3Df> vertices;
    std::vector<mrpt::math::TVector3Df> normals;
    std::vector<mrpt::img::TColor> colors;
    vertices.reserve(vertexCount);
    normals.reserve(vertexCount);
    colors.reserve(vertexCount);

    auto bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
    m_transparent = false;
    for (const auto& tri : tris)
    {
      for (int i = 0; i < 3; ++i)
      {
        vertices.push_back(tri.vertices[i].xyzrgba.pt);
        normals.push_back(tri.vertices[i].normal);
        const auto& rgba = tri.vertices[i].xyzrgba;
        colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
        bbox.updateWithPoint(tri.vertices[i].xyzrgba.pt);
        m_transparent = m_transparent || rgba.a != 0xff;
      }
    }

    // Create VAO and upload to GPU
    m_vao.createOnce();
    m_vao.bind();

    m_vertexBuffer.createOnce();
    m_vertexBuffer.bind();
    m_vertexBuffer.allocate(
        vertices.data(), static_cast<int>(sizeof(mrpt::math::TPoint3Df) * vertexCount));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint3Df), nullptr);

    m_colorBuffer.createOnce();
    m_colorBuffer.bind();
    m_colorBuffer.allocate(
        colors.data(), static_cast<int>(sizeof(mrpt::img::TColor) * vertexCount));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(mrpt::img::TColor), nullptr);

    m_normalBuffer.createOnce();
    m_normalBuffer.bind();
    m_normalBuffer.allocate(
        normals.data(), static_cast<int>(sizeof(mrpt::math::TVector3Df) * vertexCount));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TVector3Df), nullptr);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    m_localBBox = bbox;
    CHECK_OPENGL_ERROR_IN_DEBUG();

    MRPT_END
#endif
  }

  const char* typeName() const override { return "Text3DProxy"; }
};

// ============================================================================
// Text2DLabelProxy: generates text geometry from CText using the gltext system.
// Used for CText objects and for enableShowName() labels.
// ============================================================================
class Text2DLabelProxy : public TrianglesProxyBase
{
 public:
  void compile(const mrpt::viz::CVisualObject* sourceObj) override
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    MRPT_START

    const auto* textObj = dynamic_cast<const mrpt::viz::CText*>(sourceObj);
    if (textObj == nullptr)
    {
      return;
    }

    m_lightEnabled = false;
    m_cullFace = mrpt::viz::TCullFace::NONE;
    m_fontHeight = textObj->getFontHeight();

    // Generate text geometry using gltext
    std::vector<mrpt::viz::TTriangle> tris;
    std::vector<mrpt::math::TPoint3Df> lineVerts;
    std::vector<mrpt::img::TColor> lineColors;

    internal::glSetFont(textObj->getFont());
    // CText uses NICE style by default, scale=1.0 (model matrix handles scaling)
    internal::glDrawTextTransformed(
        textObj->getString(), tris, lineVerts, lineColors, mrpt::poses::CPose3D(), 1.0f,
        textObj->getColor_u8(), mrpt::viz::NICE, 1.5, 0.1);

    m_triangleCount = tris.size();
    if (m_triangleCount == 0)
    {
      m_localBBox.reset();
      return;
    }

    const size_t vertexCount = m_triangleCount * 3;

    std::vector<mrpt::math::TPoint3Df> vertices;
    std::vector<mrpt::math::TVector3Df> normals;
    std::vector<mrpt::img::TColor> colors;
    vertices.reserve(vertexCount);
    normals.reserve(vertexCount);
    colors.reserve(vertexCount);

    auto bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
    m_transparent = false;
    for (const auto& tri : tris)
    {
      for (int i = 0; i < 3; ++i)
      {
        vertices.push_back(tri.vertices[i].xyzrgba.pt);
        normals.push_back(tri.vertices[i].normal);
        const auto& rgba = tri.vertices[i].xyzrgba;
        colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
        bbox.updateWithPoint(tri.vertices[i].xyzrgba.pt);
        m_transparent = m_transparent || rgba.a != 0xff;
      }
    }

    // Create VAO and upload to GPU
    m_vao.createOnce();
    m_vao.bind();

    m_vertexBuffer.createOnce();
    m_vertexBuffer.bind();
    m_vertexBuffer.allocate(
        vertices.data(), static_cast<int>(sizeof(mrpt::math::TPoint3Df) * vertexCount));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint3Df), nullptr);

    m_colorBuffer.createOnce();
    m_colorBuffer.bind();
    m_colorBuffer.allocate(
        colors.data(), static_cast<int>(sizeof(mrpt::img::TColor) * vertexCount));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(mrpt::img::TColor), nullptr);

    m_normalBuffer.createOnce();
    m_normalBuffer.bind();
    m_normalBuffer.allocate(
        normals.data(), static_cast<int>(sizeof(mrpt::math::TVector3Df) * vertexCount));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TVector3Df), nullptr);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Drawn in screen space (see render()): no bounding box, never culled.
    m_localBBox.reset();
    CHECK_OPENGL_ERROR_IN_DEBUG();

    MRPT_END
#endif
  }

  /** CText is a 2D label: it is placed at the projection of its 3D origin,
   * but drawn at a fixed size in pixels, unaffected by the viewport
   * projection. The glyph geometry is generated with unit height, so the
   * whole transformation chain is replaced here by a translation to that
   * projected point plus the pixel-to-NDC scale.
   */
  void render(const RenderContext& rc) const override
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    MRPT_START

    if (m_triangleCount == 0 || rc.shader == nullptr || rc.state == nullptr)
    {
      return;
    }

    const auto& pmv = rc.state->pmv_matrix;
    if (std::abs(pmv(3, 3)) < 1e-10f)
    {
      return;
    }
    if (rc.state->viewport_width <= 0 || rc.state->viewport_height <= 0)
    {
      return;
    }

    const float vpH = static_cast<float>(rc.state->viewport_height);
    const float vpW = static_cast<float>(rc.state->viewport_width);

    const float scale = static_cast<float>(m_fontHeight) / vpH;
    const float aspect = vpW / vpH;

    // A negative Y row in the projection means the renderer is drawing
    // upside-down (e.g. into an FBO): keep the glyphs readable.
    const float yFlip = rc.state->p_matrix(1, 1) < 0 ? -1.0f : +1.0f;

    auto m = mrpt::math::CMatrixFloat44::Identity();
    m(0, 0) = scale / aspect;
    m(1, 1) = scale * yFlip;
    m(0, 3) = pmv(0, 3) / pmv(3, 3);
    m(1, 3) = pmv(1, 3) / pmv(3, 3);
    m(2, 3) = pmv(2, 3) / pmv(3, 3);  // keep depth, so labels can be occluded

    const auto IS_TRANSPOSED = GL_TRUE;
    glUniformMatrix4fv(rc.shader->uniformId("pmv_matrix"), 1, IS_TRANSPOSED, m.data());

    TrianglesProxyBase::render(rc);

    MRPT_END
#endif
  }

  /** Screen-space labels have no meaningful position in the light's frame. */
  bool castsShadows() const override { return false; }

  const char* typeName() const override { return "Text2DLabelProxy"; }

 private:
  int m_fontHeight = 20;
};
}  // namespace

// ============================================================================
// Scene graph nodes
// ============================================================================

struct CompiledScene::Node
{
  /** The object. A weak pointer, so objects removed from the scene are not
   * kept alive by the renderer. */
  std::weak_ptr<const CVisualObject> obj;
  const CVisualObject* raw = nullptr;

  bool isContainer = false;
  bool isLight = false;       //!< A CLight
  bool compiled = false;      //!< Proxies created (objects other than containers)
  bool hasTransform = false;  //!< worldMatrix, etc. computed at least once

  uint64_t dataVersion = 0;       //!< CVisualObject::dataVersion() uploaded to the proxies
  uint64_t transformVersion = 0;  //!< CVisualObject::transformVersion() of worldMatrix, etc.

  mrpt::math::CMatrixFloat44 worldMatrix = mrpt::math::CMatrixFloat44::Identity();
  bool visible = false;
  bool castShadows = true;

  std::vector<RenderableProxy::Ptr> proxies;

  /** Containers: their objects. Other objects: their internal children (e.g.
   * axis labels), then their name label, if shown. */
  std::vector<std::unique_ptr<Node>> children;

  /** Whether this node is for that object instance (not another one later
   * created at the same memory address) */
  [[nodiscard]] bool isFor(const std::shared_ptr<const CVisualObject>& o) const
  {
    return raw == o.get() && !obj.owner_before(o) && !o.owner_before(obj);
  }
};

struct CompiledScene::ViewportEntry
{
  std::weak_ptr<const Viewport> source;
  const Viewport* raw = nullptr;

  CompiledViewport::Ptr compiled;

  /** Nodes of the viewport objects */
  std::vector<std::unique_ptr<Node>> roots;

  /** The textured plane of a viewport in image view mode */
  std::unique_ptr<Node> imagePlane;

  /** Lights of the visible CLight objects, in world coordinates */
  std::vector<TLight> sceneLights;

  /** Proxies of removed nodes, to be removed from `compiled` in one pass */
  std::vector<const RenderableProxy*> removedProxies;

  [[nodiscard]] bool isFor(const Viewport::Ptr& v) const
  {
    return raw == v.get() && !source.owner_before(v) && !v.owner_before(source);
  }
};

namespace
{
/** Brings the clone and image-view state of a compiled viewport in line with
 * the source viewport, whose mode can change after the first compilation.
 * \return true if anything changed */
bool syncViewportModes(const mrpt::viz::Viewport& viz, mrpt::opengl::CompiledViewport& compiled)
{
  bool changed = false;

  // Objects cloned from another viewport:
  if (viz.isCloned())
  {
    if (!compiled.isCloningObjects() ||
        compiled.getClonedViewportName() != viz.getClonedViewportName())
    {
      compiled.setCloneMode(viz.getClonedViewportName(), false);
      changed = true;
    }
  }
  else if (compiled.isCloningObjects())
  {
    compiled.clearCloneMode();
    changed = true;
  }

  // Camera taken from another viewport (which does not need to be a clone):
  if (viz.isClonedCamera())
  {
    if (!compiled.isCloningCamera() ||
        compiled.getCameraSourceViewportName() != viz.isClonedCameraFrom())
    {
      compiled.setCloneCameraFrom(viz.isClonedCameraFrom());
      changed = true;
    }
  }
  else if (compiled.isCloningCamera())
  {
    compiled.clearCloneCamera();
    changed = true;
  }

  // Image view mode:
  if (!viz.isImageViewMode() && compiled.isImageViewMode())
  {
    compiled.clearImageViewMode();
    changed = true;
  }
  return changed;
}

/** Transparent objects are sorted by the depth of their representative point
 * if set, or else by that of the center of their bounding box. */
void setSortPoints(const std::vector<RenderableProxy::Ptr>& proxies, const CVisualObject& obj)
{
  const auto rep = obj.getLocalRepresentativePoint();
  const bool hasRep = rep.x != 0 || rep.y != 0 || rep.z != 0;
  for (const auto& p : proxies)
  {
    if (hasRep)
    {
      p->m_sortPointLocal = rep;
    }
    else if (const auto& bb = p->localBoundingBox(); bb)
    {
      p->m_sortPointLocal = {
          0.5f * (bb->min.x + bb->max.x), 0.5f * (bb->min.y + bb->max.y),
          0.5f * (bb->min.z + bb->max.z)};
    }
    else
    {
      p->m_sortPointLocal = {0, 0, 0};
    }
  }
}

/** A light given in the frame of a CLight, transformed to world coordinates */
TLight lightInWorld(const TLight& local, const mrpt::math::CMatrixFloat44& M)
{
  TLight l = local;
  const auto& p = local.position;
  l.position = {
      M(0, 0) * p.x + M(0, 1) * p.y + M(0, 2) * p.z + M(0, 3),
      M(1, 0) * p.x + M(1, 1) * p.y + M(1, 2) * p.z + M(1, 3),
      M(2, 0) * p.x + M(2, 1) * p.y + M(2, 2) * p.z + M(2, 3)};

  const auto& d = local.direction;
  const mrpt::math::TVector3Df dir = {
      M(0, 0) * d.x + M(0, 1) * d.y + M(0, 2) * d.z, M(1, 0) * d.x + M(1, 1) * d.y + M(1, 2) * d.z,
      M(2, 0) * d.x + M(2, 1) * d.y + M(2, 2) * d.z};
  // Normalized again, in case of scaled parents:
  const float n = dir.norm();
  if (n > 0)
  {
    l.direction = dir * (1.0f / n);
  }
  return l;
}

const mrpt::math::CMatrixFloat44& identityMatrix()
{
  static const auto I = mrpt::math::CMatrixFloat44::Identity();
  return I;
}

/** Identifies the current OpenGL context, so textures are shared only among
 * scenes rendered in the same context. Contexts not created with EGL cannot be
 * told apart here, so they all share one scope. */
const void* currentContextScope()
{
#if MRPT_HAS_EGL
  if (EGLContext ctx = eglGetCurrentContext(); ctx != EGL_NO_CONTEXT)
  {
    return ctx;
  }
#endif
  static const int nonEglContexts = 0;
  return &nonEglContexts;
}
}  // namespace

// ============================================================================
// CompiledScene Implementation
// ============================================================================

CompiledScene::CompiledScene() { m_contextThread = std::this_thread::get_id(); }

CompiledScene::~CompiledScene() = default;

void CompiledScene::compile(const Scene& scene, CompilationStats* stats)
{
  MRPT_START

  checkContextThread();

  // Clear any existing compilation
  clear();

  m_sourceScene = &scene;
  m_isCompiled = true;
  m_textureShareScope = currentContextScope();

  CompilationStats localStats;
  updateIfNeeded(stats ? stats : &localStats);
  m_lastStats = stats ? *stats : localStats;

  MRPT_END
}

bool CompiledScene::updateIfNeeded(CompilationStats* stats)
{
  MRPT_START

  if (!m_isCompiled || !m_sourceScene)
  {
    return false;
  }

  checkContextThread();

  CompilationStats localStats;
  CompilationStats& s = stats ? *stats : localStats;
  s.reset();

  // Viewports are cheap to check, and their properties (camera, lights...)
  // are not covered by the scene change counter, so they are always synced:
  const bool viewportsChanged = syncViewports(s);

  // Read before traversing, so changes made meanwhile are seen next time:
  const uint64_t changeCount = mrpt::viz::sceneChangeCount();
  if (!viewportsChanged && changeCount == m_lastSceneChangeCount)
  {
    return false;
  }
  m_lastSceneChangeCount = changeCount;

  for (auto& e : m_entries)
  {
    syncViewportObjects(*e, s);
    e->compiled->removeProxies(e->removedProxies);
    e->removedProxies.clear();
  }

  // Viewports cloning the objects of another one also get its lights:
  for (auto& e : m_entries)
  {
    if (!e->raw->isCloned())
    {
      continue;
    }
    e->sceneLights.clear();
    for (const auto& src : m_entries)
    {
      if (src->compiled->getName() == e->raw->getClonedViewportName())
      {
        e->sceneLights = src->sceneLights;
        break;
      }
    }
    e->compiled->setSceneLights(e->sceneLights);
  }

  const bool anyChanges = viewportsChanged || s.numObjectsUpdated > 0 || s.numNewObjects > 0 ||
                          s.numOrphanedProxies > 0;
  if (anyChanges)
  {
    m_lastStats = s;
  }
  return anyChanges;

  MRPT_END
}

bool CompiledScene::syncViewports(CompilationStats& stats)
{
  bool changed = false;

  auto oldEntries = std::move(m_entries);
  m_entries.clear();

  for (const auto& vp : m_sourceScene->viewports())
  {
    if (!vp)
    {
      continue;
    }
    std::unique_ptr<ViewportEntry> entry;
    for (size_t i = 0; i < oldEntries.size(); i++)
    {
      if (oldEntries[i] && oldEntries[i]->isFor(vp))
      {
        changed = changed || i != m_entries.size();  // reordered
        entry = std::move(oldEntries[i]);
        break;
      }
    }
    if (!entry)
    {
      entry = std::make_unique<ViewportEntry>();
      entry->source = vp;
      entry->raw = vp.get();
      entry->compiled = std::make_shared<CompiledViewport>(vp->getName());
      changed = true;
    }
    m_entries.push_back(std::move(entry));
  }

  // Viewports no longer in the scene:
  for (auto& e : oldEntries)
  {
    if (!e)
    {
      continue;
    }
    for (auto& n : e->roots)
    {
      releaseNode(*n, *e, stats);
    }
    changed = true;
  }

  if (changed)
  {
    m_viewports.clear();
    for (const auto& e : m_entries)
    {
      m_viewports[e->compiled->getName()] = e->compiled;
    }
  }

  for (auto& e : m_entries)
  {
    e->compiled->updateFromVizViewport(*e->raw);
    changed = syncViewportModes(*e->raw, *e->compiled) || changed;
  }
  return changed;
}

void CompiledScene::syncViewportObjects(ViewportEntry& vp, CompilationStats& stats)
{
  MRPT_START

  const Viewport& viz = *vp.raw;

  const auto releaseRoots = [&]()
  {
    for (auto& n : vp.roots)
    {
      releaseNode(*n, vp, stats);
    }
    vp.roots.clear();
  };

  // A viewport cloning the objects of another one has none of its own:
  if (viz.isCloned())
  {
    releaseRoots();
    return;
  }

  vp.sceneLights.clear();

  // Image view mode: only the textured plane is drawn.
  if (viz.isImageViewMode())
  {
    releaseRoots();
    vp.compiled->setSceneLights(vp.sceneLights);
    const auto plane = viz.getImageViewPlane();
    if (vp.imagePlane && !vp.imagePlane->isFor(plane))
    {
      releaseNode(*vp.imagePlane, vp, stats);
      vp.imagePlane.reset();
    }
    if (!vp.imagePlane)
    {
      auto proxies = createProxiesByType(*plane);
      if (proxies.empty())
      {
        return;
      }
      plane->updateBuffersIfNeeded();
      auto node = std::make_unique<Node>();
      node->obj = plane;
      node->raw = plane.get();
      node->compiled = true;
      node->dataVersion = plane->dataVersion();
      // Only the first (TexturedTriangles) proxy is used for image view
      auto& proxy = proxies.front();
      proxy->setSourceObject(plane);
      proxy->setResourceScope(m_textureShareScope);
      proxy->m_modelMatrix = mrpt::math::CMatrixFloat44::Identity();
      proxy->m_visible = true;
      proxy->compile(plane.get());
      node->proxies.push_back(proxy);
      vp.compiled->setImageViewMode(proxy);
      vp.imagePlane = std::move(node);
      stats.numProxiesCreated++;
      stats.numObjectsCompiled++;
    }
    else if (const uint64_t dv = plane->dataVersion(); dv != vp.imagePlane->dataVersion)
    {
      plane->updateBuffersIfNeeded();
      for (auto& p : vp.imagePlane->proxies)
      {
        p->updateBuffers(plane.get());
      }
      vp.imagePlane->dataVersion = dv;
      stats.numObjectsUpdated++;
    }
    return;
  }
  if (vp.imagePlane)
  {
    releaseNode(*vp.imagePlane, vp, stats);
    vp.imagePlane.reset();
  }

  ParentState root;
  root.worldMatrix = &identityMatrix();
  syncNodes(vp.roots, viz, root, vp, stats);
  vp.compiled->setSceneLights(vp.sceneLights);

  MRPT_END
}

template <class OBJECTS>
void CompiledScene::syncNodes(
    std::vector<std::unique_ptr<Node>>& nodes,
    const OBJECTS& objects,
    const ParentState& parent,
    ViewportEntry& vp,
    CompilationStats& stats)
{
  // The common case: the same objects as last time, in the same order.
  bool same = true;
  size_t n = 0;
  for (const auto& o : objects)
  {
    if (!o)
    {
      continue;
    }
    if (n >= nodes.size() || !nodes[n]->isFor(o))
    {
      same = false;
      break;
    }
    n++;
  }
  same = same && n == nodes.size();

  if (!same)
  {
    // Keep the nodes of objects still present, so they keep their proxies:
    auto oldNodes = std::move(nodes);
    nodes.clear();
    std::unordered_multimap<const CVisualObject*, size_t> oldIndex;
    for (size_t i = 0; i < oldNodes.size(); i++)
    {
      oldIndex.emplace(oldNodes[i]->raw, i);
    }

    for (const auto& o : objects)
    {
      if (!o)
      {
        continue;
      }
      std::unique_ptr<Node> node;
      const auto range = oldIndex.equal_range(o.get());
      for (auto it = range.first; it != range.second; ++it)
      {
        auto& candidate = oldNodes[it->second];
        if (candidate && candidate->isFor(o))
        {
          node = std::move(candidate);
          break;
        }
      }
      if (!node)
      {
        node = std::make_unique<Node>();
        node->obj = o;
        node->raw = o.get();
        node->isContainer = dynamic_cast<const CSetOfObjects*>(o.get()) != nullptr;
        node->isLight = dynamic_cast<const CLight*>(o.get()) != nullptr;
        stats.numNewObjects++;
      }
      nodes.push_back(std::move(node));
    }

    // Objects no longer at these positions:
    for (auto& old : oldNodes)
    {
      if (old)
      {
        releaseNode(*old, vp, stats);
        stats.numOrphanedProxies++;
      }
    }
  }

  n = 0;
  for (const auto& o : objects)
  {
    if (o)
    {
      updateNode(*nodes[n++], o, parent, vp, stats);
    }
  }
}

void CompiledScene::updateNode(
    Node& node,
    const std::shared_ptr<const CVisualObject>& objPtr,
    const ParentState& parent,
    ViewportEntry& vp,
    CompilationStats& stats)
{
  const CVisualObject& obj = *objPtr;
  stats.numObjectsTotal++;

  // Where and whether it is drawn:
  const uint64_t tv = obj.transformVersion();
  bool transformChanged = false;
  if (parent.changed || !node.hasTransform || tv != node.transformVersion)
  {
    // Setting the same pose again is not a change, so cached results (e.g.
    // shadow maps) that depend on this object remain valid:
    const auto ps = obj.getPoseAndScale();
    const auto worldMatrix = computeModelMatrix(ps, *parent.worldMatrix);
    const bool visible = parent.visible && ps.visible;
    const bool castShadows = parent.castShadows && obj.castShadows();
    transformChanged = !node.hasTransform || worldMatrix != node.worldMatrix ||
                       visible != node.visible || castShadows != node.castShadows;
    node.worldMatrix = worldMatrix;
    node.visible = visible;
    node.castShadows = castShadows;
    node.transformVersion = tv;
    node.hasTransform = true;
  }

  ParentState asParent;
  asParent.worldMatrix = &node.worldMatrix;
  asParent.changed = transformChanged;
  asParent.visible = node.visible;
  asParent.castShadows = node.castShadows;

  // Lights are not drawn: only collected, if switched on.
  if (node.isLight)
  {
    if (node.visible)
    {
      vp.sceneLights.push_back(
          lightInWorld(static_cast<const CLight&>(obj).light(), node.worldMatrix));
    }
    return;
  }

  if (node.isContainer)
  {
    syncNodes(node.children, static_cast<const CSetOfObjects&>(obj), asParent, vp, stats);
    return;
  }

  // Objects are compiled the first time they are visible:
  if (!node.compiled && !node.visible)
  {
    return;
  }

  bool dataUpdated = false;
  if (!node.compiled)
  {
    compileNodeProxies(node, obj, vp, stats);
    dataUpdated = true;
  }
  else if (const uint64_t dv = obj.dataVersion(); dv != node.dataVersion)
  {
    obj.updateBuffersIfNeeded();
    for (auto& p : node.proxies)
    {
      p->updateBuffers(&obj);
    }
    setSortPoints(node.proxies, obj);
    node.dataVersion = dv;
    dataUpdated = true;
  }

  if (dataUpdated || transformChanged)
  {
    for (auto& p : node.proxies)
    {
      p->m_modelMatrix = node.worldMatrix;
      p->m_visible = node.visible;
      p->m_castShadows = node.castShadows;
      p->m_changeCount++;
    }
    stats.numObjectsUpdated++;
  }

  // Children of other objects: internal ones (e.g. axis labels), and the
  // label with the object name.
  const bool showName = obj.isShowNameEnabled();
  if (obj.isCompositeObject() || showName || !node.children.empty())
  {
    std::vector<std::shared_ptr<const CVisualObject>> children;
    if (obj.isCompositeObject())
    {
      const auto& internal = obj.getInternalChildren();
      children.assign(internal.begin(), internal.end());
    }
    if (showName)
    {
      auto label = obj.labelObjectPtr();
      label->setString(obj.getName());  // only notifies a change if different
      children.push_back(label);
    }
    syncNodes(node.children, children, asParent, vp, stats);
  }
}

void CompiledScene::compileNodeProxies(
    Node& node, const CVisualObject& obj, ViewportEntry& vp, CompilationStats& stats)
{
  // Read before regenerating the buffers, so changes made meanwhile are
  // uploaded next time:
  node.dataVersion = obj.dataVersion();
  node.compiled = true;
  node.proxies = createProxiesByType(obj);
  if (node.proxies.empty())
  {
    return;
  }

  obj.updateBuffersIfNeeded();
  for (auto& proxy : node.proxies)
  {
    proxy->setSourceObject(node.obj);
    proxy->setResourceScope(m_textureShareScope);
    proxy->compile(&obj);
    vp.compiled->addProxy(proxy);
    stats.numProxiesCreated++;
  }
  setSortPoints(node.proxies, obj);
  stats.numObjectsCompiled++;
}

void CompiledScene::releaseNode(Node& node, ViewportEntry& vp, CompilationStats& stats)
{
  for (const auto& p : node.proxies)
  {
    vp.removedProxies.push_back(p.get());
    stats.numProxiesDeleted++;
  }
  node.proxies.clear();
  for (auto& child : node.children)
  {
    releaseNode(*child, vp, stats);
  }
  node.children.clear();
}

mrpt::math::CMatrixFloat44 CompiledScene::computeModelMatrix(
    const CVisualObject::PoseAndScale& ps, const mrpt::math::CMatrixFloat44& parentModelMatrix)
{
  mrpt::math::CMatrixFloat44 HM =
      ps.pose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>().cast_float();

  // Apply scaling if any axis differs from 1.0
  if (ps.scaleX != 1 || ps.scaleY != 1 || ps.scaleZ != 1)
  {
    auto scale = mrpt::math::CMatrixFloat44::Identity();
    scale(0, 0) = ps.scaleX;
    scale(1, 1) = ps.scaleY;
    scale(2, 2) = ps.scaleZ;
    HM.asEigen() = HM.asEigen() * scale.asEigen();
  }

  // Compose with parent transform
  mrpt::math::CMatrixFloat44 result;
  result.asEigen() = parentModelMatrix.asEigen() * HM.asEigen();
  return result;
}

std::vector<RenderableProxy::Ptr> CompiledScene::createProxiesByType(const CVisualObject& obj)
{
  std::vector<RenderableProxy::Ptr> proxies;

  // Check for CSkyBox (special case: rendered with cube map, no triangle data)
  if (dynamic_cast<const mrpt::viz::CSkyBox*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<SkyBoxProxy>());
    return proxies;
  }

  // Check for CText first (special case: generates geometry in the proxy)
  if (dynamic_cast<const mrpt::viz::CText*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<Text2DLabelProxy>());
    return proxies;
  }

  // Check for CText3D (special case: generates geometry in the proxy)
  if (dynamic_cast<const mrpt::viz::CText3D*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<Text3DProxy>());
    return proxies;
  }

  // Check for textured triangles (more specific than plain triangles)
  if (dynamic_cast<const VisualObjectParams_TexturedTriangles*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<TexturedTrianglesProxy>());
  }
  // Check for plain triangles (only if NOT textured, since textured already
  // handles the triangle data)
  else if (dynamic_cast<const VisualObjectParams_Triangles*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<TrianglesProxy>());
  }

  // Check for points (independent of triangles: an object can have both)
  if (dynamic_cast<const VisualObjectParams_Points*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<PointsProxy>());
  }

  // Check for lines (independent of triangles: an object can have both)
  if (dynamic_cast<const VisualObjectParams_Lines*>(&obj) != nullptr)
  {
    proxies.push_back(std::make_shared<LinesProxy>());
  }

  return proxies;
}

void CompiledScene::recompile()
{
  MRPT_START

  if (!m_sourceScene)
  {
    return;
  }

  compile(*m_sourceScene, &m_lastStats);

  MRPT_END
}

void CompiledScene::clear()
{
  MRPT_START

  m_entries.clear();
  m_viewports.clear();
  m_shaderManager.clear();
  m_sourceScene = nullptr;
  m_isCompiled = false;
  m_lastSceneChangeCount = 0;

  MRPT_END
}

bool CompiledScene::isCompiledFrom(const mrpt::viz::Scene& scene) const
{
  if (!m_isCompiled || m_sourceScene != &scene)
  {
    return false;
  }
  if (m_entries.empty() || scene.viewports().empty())
  {
    return true;
  }
  // At least one of the compiled viewports must be still in the scene:
  for (const auto& e : m_entries)
  {
    for (const auto& vp : scene.viewports())
    {
      if (vp && e->isFor(vp))
      {
        return true;
      }
    }
  }
  return false;
}

bool CompiledScene::hasPendingUpdates() const
{
  return m_isCompiled && mrpt::viz::sceneChangeCount() != m_lastSceneChangeCount;
}

size_t CompiledScene::getProxyCount() const
{
  size_t count = 0;
  for (const auto& e : m_entries)
  {
    count += e->compiled->getProxyCount();
    if (e->imagePlane)
    {
      count += e->imagePlane->proxies.size();
    }
  }
  return count;
}

void CompiledScene::render(int renderWidth, int renderHeight, int renderOffsetX, int renderOffsetY)
{
  MRPT_START

#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!m_isCompiled)
  {
    return;
  }

  checkContextThread();

  // Auto-update if enabled
  if (m_autoUpdate)
  {
    updateIfNeeded();
  }

  // Render all viewports in the order of the source scene (so overlay
  // viewports render after the main viewport).
  for (const auto& e : m_entries)
  {
    auto& viewport = e->compiled;

    // For cloned viewports, resolve the source viewport
    const CompiledViewport* sourceVp = nullptr;
    if (viewport->isCloningObjects())
    {
      auto srcIt = m_viewports.find(viewport->getClonedViewportName());
      if (srcIt != m_viewports.end())
      {
        sourceVp = srcIt->second.get();
      }
    }

    // A viewport can use the camera of another one, whether it clones its
    // objects or not. The camera is used with the dimensions of this viewport
    // for the matrix computation.
    if (viewport->isCloningCamera())
    {
      auto srcVizVp = m_sourceScene->getViewport(viewport->getCameraSourceViewportName());
      if (srcVizVp)
      {
        viewport->updateCamera(srcVizVp->getCamera());
        viewport->forceMatrixUpdate();
      }
    }
    viewport->render(
        renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager, sourceVp);
  }
#endif

  MRPT_END
}

void CompiledScene::renderViewport(
    const std::string& viewportName,
    int renderWidth,
    int renderHeight,
    int renderOffsetX,
    int renderOffsetY)
{
  MRPT_START

#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  checkContextThread();

  // Auto-update if enabled
  if (m_autoUpdate)
  {
    updateIfNeeded();
  }

  auto it = m_viewports.find(viewportName);
  if (it == m_viewports.end())
  {
    THROW_EXCEPTION_FMT("Viewport '%s' not found", viewportName.c_str());
  }

  it->second->render(renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager);
#endif

  MRPT_END
}

void CompiledScene::checkContextThread() const
{
#ifndef NDEBUG
  if (std::this_thread::get_id() != m_contextThread)
  {
    THROW_EXCEPTION(
        "CompiledScene methods must be called from the same thread that created it "
        "(the OpenGL context thread)");
  }
#endif
}
