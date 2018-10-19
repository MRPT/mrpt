/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
#define INVALID_DISPLAY_LIST_ID static_cast<unsigned int>(-1)

/** A renderizable object suitable for rendering with OpenGL's display lists.
 *   The idea is to use the derived classes' ::render() method to save all the
 *primitives
 *   into one display list, then in subsequent rendering events, just execute
 *the list.
 *   This method is normally faster since it avoids the bottleneck between
 *CPU-GPU. On the
 *   other hand, it demands more memory on the graphic card.
 *
 *  Instructions for implementing derived classes:
 *		- Each time the object is modified is some way that modifies its
 *appearance, you must call notifyChange()
 *		- Implement the rendering method: render_dl(), calling to OpenGL
 *primitives
 *as usual. They'll be saved in a display list transparently.
 *
 *  \sa mrpt::opengl::CRenderizable
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableDisplayList : public mrpt::opengl::CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableDisplayList)

   private:
	/** Display list ID, for derived classes that want to use it (it's
	 * automatically deleted and freed on destruction of this base class). */
	mutable unsigned int m_dl;
	/** If using display lists, this is true when the list must be updated (the
	 * object changes, it's the first rendering, etc...). */
	mutable bool m_dl_recreate{true};

   protected:
	/** @name Methods accesible or implemented by derived classes
		@{ */

	/** Must be called to notify that the object has changed (so, the display
	 * list must be updated) */
	EIGEN_STRONG_INLINE void notifyChange() const { m_dl_recreate = true; }
	/** Derived classes must implement this method to the render the object. */
	virtual void render_dl() const = 0;

	/** Optional: If the object has some state in which creating a display list
	 * is NOT preferred over direct rendering,
	 *  implement this method and return "true" in those cases. */
	virtual bool should_skip_display_list_cache() const { return false; }
	inline void readFromStreamRender(mrpt::serialization::CArchive& in)
	{
		CRenderizable::readFromStreamRender(in);
		notifyChange();
	}

	/** @} */

   public:
	CRenderizableDisplayList();
	~CRenderizableDisplayList() override;

	/** Render the object, regenerating the display list if needed, otherwise
	 * just calling it. */
	void render() const override;

	/** @name Changes the appearance of the object to render (methods from
	   CRenderizable that need to be redefined)
		@{ */
	/**Color components in the range [0,255] \return a ref to this */
	CRenderizable& setColorR_u8(const uint8_t r) override
	{
		m_color.R = r;
		notifyChange();
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	CRenderizable& setColorG_u8(const uint8_t g) override
	{
		m_color.G = g;
		notifyChange();
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	CRenderizable& setColorB_u8(const uint8_t b) override
	{
		m_color.B = b;
		notifyChange();
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	CRenderizable& setColorA_u8(const uint8_t a) override
	{
		m_color.A = a;
		notifyChange();
		return *this;
	}
	/** Changes the default object color \return a ref to this */
	CRenderizable& setColor_u8(const mrpt::img::TColor& c) override
	{
		CRenderizable::setColor_u8(c);
		notifyChange();
		return *this;
	}
	/**Color components in the range [0,255] \return a ref to this */
	inline CRenderizable& setColor_u8(
		uint8_t R, uint8_t G, uint8_t B, uint8_t A = 255)
	{
		CRenderizable::setColor_u8(R, G, B, A);
		notifyChange();
		return *this;
	}
	/** @} */
};

}  // namespace mrpt::opengl
