/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/TPose3D.h>
#include <mrpt/math/utils.h>
#include <mrpt/opengl/CRenderizable.h>  // Include these before windows.h!!
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <mutex>

#include "opengl_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CRenderizable, CSerializable, mrpt::opengl)

#define MAX_GL_TEXTURE_IDS 0x10000
#define MAX_GL_TEXTURE_IDS_MASK 0x0FFFF

struct TOpenGLNameBooker
{
   private:
	TOpenGLNameBooker() : freeTextureNames(MAX_GL_TEXTURE_IDS, false) {}

   public:
	std::vector<bool> freeTextureNames;
	unsigned int next_free_texture{1};
	std::recursive_mutex cs;

	static TOpenGLNameBooker& instance()
	{
		static TOpenGLNameBooker dat;
		return dat;
	}
};

// Default constructor:
CRenderizable::CRenderizable()
	: m_name(),

	  m_color(255, 255, 255, 255),
	  m_pose()

{
}

// Destructor:
CRenderizable::~CRenderizable() = default;
/** Returns the lowest, free texture name.
 */
unsigned int CRenderizable::getNewTextureNumber()
{
	MRPT_START

	TOpenGLNameBooker& booker = TOpenGLNameBooker::instance();

	std::lock_guard<std::recursive_mutex> lock(booker.cs);

	unsigned int ret = booker.next_free_texture;
	unsigned int tries = 0;
	while (ret != 0 && booker.freeTextureNames[ret])
	{
		ret++;
		ret = ret % MAX_GL_TEXTURE_IDS_MASK;

		if (++tries >= MAX_GL_TEXTURE_IDS)
			THROW_EXCEPTION_FMT(
				"Maximum number of textures (%u) excedeed! (are you deleting "
				"them?)",
				(unsigned int)MAX_GL_TEXTURE_IDS);
	}

	booker.freeTextureNames[ret] = true;  // mark as used.
	booker.next_free_texture = ret + 1;
	return ret;
	MRPT_END
}

void CRenderizable::releaseTextureName(unsigned int i)
{
	TOpenGLNameBooker& booker = TOpenGLNameBooker::instance();
	std::lock_guard<std::recursive_mutex> lock(booker.cs);
	booker.freeTextureNames[i] = false;
	if (i < booker.next_free_texture)
		booker.next_free_texture = i;  // try to reuse texture numbers.
	// "glDeleteTextures" seems not to be neeeded, since we do the reservation
	// of texture names by our own.
}

void CRenderizable::writeToStreamRender(
	mrpt::serialization::CArchive& out) const
{
	// MRPT 0.9.5 svn 2774 (Dec 14th 2011):
	// Added support of versioning at this level of serialization too.
	// Should have been done from the beginning, terrible mistake on my part.
	// Now, the only solution is something as ugly as this:
	//
	// For reference: In the past this started as:
	// out << m_name << (float)(m_color.R) << (float)(m_color.G) <<
	// (float)(m_color.B) << (float)(m_color.A);
	// ...

	const uint8_t serialization_version =
		0;  // can't be >31 (but it would be mad geting to that situation!)

	const bool all_scales_equal =
		(m_scale_x == m_scale_y && m_scale_z == m_scale_x);
	const bool all_scales_unity = (all_scales_equal && m_scale_x == 1.0f);

	const uint8_t magic_signature[2] = {
		0xFF,
		// bit7: fixed to 1 to mark this new header format
		// bit6: whether the 3 scale{x,y,z} are equal to 1.0
		// bit5: whether the 3 scale{x,y,z} are equal to each other
		static_cast<uint8_t>(
			serialization_version |
			(all_scales_unity ? 0xC0 : (all_scales_equal ? 0xA0 : 0x80)))};

	out << magic_signature[0] << magic_signature[1];

	// "m_name"
	const auto nameLen = static_cast<uint16_t>(m_name.size());
	out << nameLen;
	if (nameLen) out.WriteBuffer(m_name.c_str(), m_name.size());

	// Color, as u8:
	out << m_color.R << m_color.G << m_color.B << m_color.A;

	// the rest of fields:
	out << (float)m_pose.x() << (float)m_pose.y() << (float)m_pose.z()
		<< (float)m_pose.yaw() << (float)m_pose.pitch() << (float)m_pose.roll();

	if (!all_scales_unity)
	{
		if (all_scales_equal)
			out << m_scale_x;
		else
			out << m_scale_x << m_scale_y << m_scale_z;
	}

	out << m_show_name << m_visible;
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

	const bool is_new_format =
		(magic_signature[0] == 0xFF) && ((magic_signature[1] & 0x80) != 0);

	if (is_new_format)
	{
		// NEW FORMAT:
		uint8_t serialization_version = (magic_signature[1] & 0x1F);
		const bool all_scales_unity = ((magic_signature[1] & 0x40) != 0);
		const bool all_scales_equal_but_not_unity =
			((magic_signature[1] & 0x20) != 0);

		switch (serialization_version)
		{
			case 0:
			{
				// "m_name"
				uint16_t nameLen;
				in >> nameLen;
				m_name.resize(nameLen);
				if (nameLen) in.ReadBuffer((void*)(&m_name[0]), m_name.size());

				// Color, as u8:
				in >> m_color.R >> m_color.G >> m_color.B >> m_color.A;

				// the rest of fields:
				float x, y, z, yaw, pitch, roll;
				in >> x >> y >> z >> yaw >> pitch >> roll;
				m_pose.x(x);
				m_pose.y(y);
				m_pose.z(z);
				m_pose.setYawPitchRoll(yaw, pitch, roll);

				if (all_scales_unity)
					m_scale_x = m_scale_y = m_scale_z = 1;
				else
				{
					if (all_scales_equal_but_not_unity)
					{
						in >> m_scale_x;
						m_scale_y = m_scale_z = m_scale_x;
					}
					else
						in >> m_scale_x >> m_scale_y >> m_scale_z;
				}

				in >> m_show_name >> m_visible;
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
	m_pose = o;
	return *this;
}
CRenderizable& CRenderizable::setPose(const mrpt::poses::CPose2D& o)
{
	m_pose = mrpt::poses::CPose3D(o);
	return *this;
}
CRenderizable& CRenderizable::setPose(const mrpt::math::TPose3D& o)
{
	m_pose = mrpt::poses::CPose3D(o);
	return *this;
}
CRenderizable& CRenderizable::setPose(const mrpt::math::TPose2D& o)
{
	m_pose = mrpt::poses::CPose3D(o);
	return *this;
}

/** Set the 3D pose from a mrpt::poses::CPose3D object */
CRenderizable& CRenderizable::setPose(const mrpt::poses::CPoint3D& o)
{
	m_pose.setFromValues(o.x(), o.y(), o.z(), 0, 0, 0);
	return *this;
}
/** Set the 3D pose from a mrpt::poses::CPose3D object */
CRenderizable& CRenderizable::setPose(const mrpt::poses::CPoint2D& o)
{
	m_pose.setFromValues(o.x(), o.y(), 0, 0, 0, 0);
	return *this;
}

mrpt::math::TPose3D CRenderizable::getPose() const { return m_pose.asTPose(); }
bool CRenderizable::traceRay(const mrpt::poses::CPose3D&, double&) const
{
	return false;
}

CRenderizable::Ptr& mrpt::opengl::operator<<(
	CRenderizable::Ptr& r, const mrpt::poses::CPose3D& p)
{
	r->setPose(p + mrpt::poses::CPose3D(r->getPose()));
	return r;
}

CRenderizable& CRenderizable::setColor_u8(const mrpt::img::TColor& c)
{
	m_color.R = c.R;
	m_color.G = c.G;
	m_color.B = c.B;
	m_color.A = c.A;
	return *this;
}

/** This method is safe for calling from within ::render() methods \sa
 * renderTextBitmap */
void CRenderizable::renderTextBitmap(const char* str, void* fontStyle)
{
	gl_utils::renderTextBitmap(str, fontStyle);
}

/** Return the exact width in pixels for a given string, as will be rendered by
 * renderTextBitmap().
 * \sa renderTextBitmap
 */
int CRenderizable::textBitmapWidth(
	const std::string& str, mrpt::opengl::TOpenGLFont font)
{
	return gl_utils::textBitmapWidth(str, font);
}
