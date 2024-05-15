/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/utils.h>
#include <mrpt/opengl/CRenderizable.h>  // Include these before windows.h!!
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <mutex>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CRenderizable, CSerializable, mrpt::opengl)

// Destructor:
CRenderizable::~CRenderizable() = default;

void CRenderizable::writeToStreamRender(mrpt::serialization::CArchive& out) const
{
  std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
  const auto& _ = m_state;

  // MRPT 0.9.5 svn 2774 (Dec 14th 2011):
  // Added support of versioning at this level of serialization too.
  // Should have been done from the beginning, terrible mistake on my part.
  // Now, the only solution is something as ugly as this:
  //
  // For reference: In the past this started as:
  // out << m_name << (float)(m_color.R) << (float)(m_color.G) <<
  // (float)(m_color.B) << (float)(m_color.A);
  // ...

  // can't be >31 (but it would be mad getting to that situation!)
  const uint8_t serialization_version = 2;

  const bool all_scales_equal = (_.scale_x == _.scale_y && _.scale_z == _.scale_x);
  const bool all_scales_unity = (all_scales_equal && _.scale_x == 1.0f);

  const uint8_t magic_signature[2] = {
      0xFF,
      // bit7: fixed to 1 to mark this new header format
      // bit6: whether the 3 scale{x,y,z} are equal to 1.0
      // bit5: whether the 3 scale{x,y,z} are equal to each other
      static_cast<uint8_t>(
          serialization_version | (all_scales_unity ? 0xC0 : (all_scales_equal ? 0xA0 : 0x80)))};

  out << magic_signature[0] << magic_signature[1];

  // "m_name"
  const auto nameLen = static_cast<uint16_t>(_.name.size());
  out << nameLen;
  if (nameLen) out.WriteBuffer(_.name.c_str(), _.name.size());

  // Color, as u8:
  out << _.color.R << _.color.G << _.color.B << _.color.A;

  // the rest of fields:
  out << (float)_.pose.x() << (float)_.pose.y() << (float)_.pose.z() << (float)_.pose.yaw()
      << (float)_.pose.pitch() << (float)_.pose.roll();

  if (!all_scales_unity)
  {
    if (all_scales_equal)
      out << _.scale_x;
    else
      out << _.scale_x << _.scale_y << _.scale_z;
  }

  out << _.show_name << _.visible;
  out << _.representativePoint;                 // v1
  out << _.materialShininess << _.castShadows;  // v2
}

void CRenderizable::readFromStreamRender(mrpt::serialization::CArchive& in)
{
  // MRPT 0.9.5 svn 2774 (Dec 14th 2011):
  // See comments in CRenderizable::writeToStreamRender() for the employed
  // serialization mechanism.
  //
  // MRPT 1.9.9 (Aug 2019): Was
  // union {
  //  uint8_t magic_signature[2 + 2];
  //    (the extra 4 bytes will be used only for the old format)
  //  uint32_t magic_signature_uint32;
  //    So we can interpret the 4bytes above as a 32bit number cleanly.
  // };
  // Get rid of the "old" serialization format to avoid using "union".

  uint8_t magic_signature[2];

  in >> magic_signature[0] >> magic_signature[1];

  const bool is_new_format = (magic_signature[0] == 0xFF) && ((magic_signature[1] & 0x80) != 0);

  std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
  auto& _ = m_state;

  if (is_new_format)
  {
    // NEW FORMAT:
    uint8_t serialization_version = (magic_signature[1] & 0x1F);
    const bool all_scales_unity = ((magic_signature[1] & 0x40) != 0);
    const bool all_scales_equal_but_not_unity = ((magic_signature[1] & 0x20) != 0);

    switch (serialization_version)
    {
      case 0:
      case 1:
      case 2:
      {
        // "m_name"
        uint16_t nameLen;
        in >> nameLen;
        _.name.resize(nameLen);
        if (nameLen) in.ReadBuffer((void*)(&_.name[0]), _.name.size());

        // Color, as u8:
        in >> _.color.R >> _.color.G >> _.color.B >> _.color.A;

        // the rest of fields:
        float x, y, z, yaw, pitch, roll;
        in >> x >> y >> z >> yaw >> pitch >> roll;
        _.pose.x(x);
        _.pose.y(y);
        _.pose.z(z);
        _.pose.setYawPitchRoll(yaw, pitch, roll);

        if (all_scales_unity)
          _.scale_x = _.scale_y = _.scale_z = 1;
        else
        {
          if (all_scales_equal_but_not_unity)
          {
            in >> _.scale_x;
            _.scale_y = _.scale_z = _.scale_x;
          }
          else
            in >> _.scale_x >> _.scale_y >> _.scale_z;
        }

        in >> _.show_name >> _.visible;
        if (serialization_version >= 1)
          in >> _.representativePoint;
        else
          _.representativePoint = mrpt::math::TPoint3Df(0, 0, 0);

        if (serialization_version >= 2)
          in >> _.materialShininess >> _.castShadows;
        else
        {
          // default
          _.materialShininess = 0.2f;
          _.castShadows = true;
        }
      }
      break;
      default:
        THROW_EXCEPTION_FMT(
            "Can't parse CRenderizable standard data field: corrupt "
            "data stream or format in a newer MRPT format? "
            "(serialization version=%u)",
            static_cast<unsigned int>(serialization_version));
    };
  }
  else
  {
    // OLD FORMAT:
    THROW_EXCEPTION("Serialized object is too old! Unsupported format.");
  }
}

/*--------------------------------------------------------------
          setPose
  ---------------------------------------------------------------*/
CRenderizable& CRenderizable::setPose(const mrpt::poses::CPose3D& o)
{
  m_stateMtx.data.lock();
  m_state.pose = o;
  m_stateMtx.data.unlock();
  return *this;
}
CRenderizable& CRenderizable::setPose(const mrpt::poses::CPose2D& o)
{
  return setPose(mrpt::poses::CPose3D(o));
}
CRenderizable& CRenderizable::setPose(const mrpt::math::TPose3D& o)
{
  return setPose(mrpt::poses::CPose3D(o));
}
CRenderizable& CRenderizable::setPose(const mrpt::math::TPose2D& o)
{
  return setPose(mrpt::poses::CPose3D(o));
}

CRenderizable& CRenderizable::setPose(const mrpt::poses::CPoint3D& o)
{
  m_stateMtx.data.lock();
  m_state.pose.setFromValues(o.x(), o.y(), o.z(), 0, 0, 0);
  m_stateMtx.data.unlock();
  return *this;
}
CRenderizable& CRenderizable::setPose(const mrpt::poses::CPoint2D& o)
{
  m_stateMtx.data.lock();
  m_state.pose.setFromValues(o.x(), o.y(), 0, 0, 0, 0);
  m_stateMtx.data.unlock();
  return *this;
}

mrpt::math::TPose3D CRenderizable::getPose() const
{
  std::shared_lock<std::shared_mutex> lckRead(m_stateMtx.data);
  return m_state.pose.asTPose();
}
bool CRenderizable::traceRay(const mrpt::poses::CPose3D&, double&) const { return false; }

CRenderizable& CRenderizable::setColor_u8(const mrpt::img::TColor& c)
{
  std::unique_lock<std::shared_mutex> lckWrite(m_stateMtx.data);
  m_state.color.R = c.R;
  m_state.color.G = c.G;
  m_state.color.B = c.B;
  m_state.color.A = c.A;
  notifyChange();
  return *this;
}

CText& CRenderizable::labelObject() const
{
  if (!m_label_obj)
  {
    m_label_obj = std::make_shared<mrpt::opengl::CText>();
    m_label_obj->setString(getName());
  }
  return *m_label_obj;
}

void CRenderizable::toYAMLMap(mrpt::containers::yaml& propertiesMap) const
{
  propertiesMap["name"] = getName();
  propertiesMap["show_name"] = isShowNameEnabled();
  propertiesMap["location"] = getPose().asString();
  propertiesMap["visible"] = isVisible();
}

#ifdef MRPT_OPENGL_PROFILER
mrpt::system::CTimeLogger& mrpt::opengl::opengl_profiler()
{
  static mrpt::system::CTimeLogger tl;
  return tl;
}
#endif

auto CRenderizable::getBoundingBoxLocalf() const -> mrpt::math::TBoundingBoxf
{
  if (!m_cachedLocalBBox)
  {
    std::unique_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
    m_cachedLocalBBox = internalBoundingBoxLocal();
    return m_cachedLocalBBox.value();
  }
  else
  {
    std::shared_lock<std::shared_mutex> lckWrite(m_outdatedStateMtx.data);
    return m_cachedLocalBBox.value();
  }
}

auto CRenderizable::getBoundingBoxLocal() const -> mrpt::math::TBoundingBox
{
  const auto& bb = getBoundingBoxLocalf();
  return {bb.min.cast<double>(), bb.max.cast<double>()};
}
