/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CRenderizableDisplayList_H
#define opengl_CRenderizableDisplayList_H

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt
{
	namespace opengl
	{
		#define INVALID_DISPLAY_LIST_ID  static_cast<unsigned int>(-1)

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRenderizableDisplayList, CRenderizable, OPENGL_IMPEXP )

		/** A renderizable object suitable for rendering with OpenGL's display lists.
		  *   The idea is to use the derived classes' ::render() method to save all the primitives
		  *   into one display list, then in subsequent rendering events, just execute the list.
		  *   This method is normally faster since it avoids the bottleneck between CPU-GPU. On the 
		  *   other hand, it demands more memory on the graphic card.
		  *
		  *  Instructions for implementing derived classes:
		  *		- Each time the object is modified is some way that modifies its appearance, you must call notifyChange()
		  *		- Implement the rendering method: render_dl(), calling to OpenGL primitives as usual. They'll be saved in a display list transparently.
		  *
		  *  \sa mrpt::opengl::CRenderizable
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CRenderizableDisplayList : public mrpt::opengl::CRenderizable
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CRenderizableDisplayList  )

		private:
			mutable unsigned int	m_dl; //!< Display list ID, for derived classes that want to use it (it's automatically deleted and freed on destruction of this base class).
			mutable bool			m_dl_recreate; //!< If using display lists, this is true when the list must be updated (the object changes, it's the first rendering, etc...).

		protected:
			/** @name Methods accesible or implemented by derived classes 
			    @{ */

			/** Must be called to notify that the object has changed (so, the display list must be updated) */
			EIGEN_STRONG_INLINE void notifyChange() const { m_dl_recreate=true;}

			/** Derived classes must implement this method to the render the object. */
			virtual void render_dl() const = 0;

			/** Optional: If the object has some state in which creating a display list is NOT preferred over direct rendering, 
			  *  implement this method and return "true" in those cases. */
			virtual bool should_skip_display_list_cache() const { return false; }

			inline void  readFromStreamRender(mrpt::utils::CStream &in)
			{ 
				CRenderizable::readFromStreamRender(in);
				notifyChange();
			}

			/** @} */

 		public:
			CRenderizableDisplayList();
			virtual ~CRenderizableDisplayList();

			/** Interface for the stlplus smart pointer class. */
			inline CRenderizableDisplayList * clone() const
			{
				return static_cast<CRenderizableDisplayList*>( this->duplicate() );
			}

			/** Render the object, regenerating the display list if needed, otherwise just calling it. */
			virtual void render() const;


			/** @name Changes the appearance of the object to render (methods from CRenderizable that need to be redefined)
			    @{ */

			virtual CRenderizable&  setColorR_u8(const uint8_t r)	{m_color.R=r; notifyChange(); return *this;}	//!<Color components in the range [0,255] \return a ref to this
			virtual CRenderizable&  setColorG_u8(const uint8_t g)	{m_color.G=g; notifyChange(); return *this;}	//!<Color components in the range [0,255] \return a ref to this
			virtual CRenderizable&  setColorB_u8(const uint8_t b)	{m_color.B=b; notifyChange(); return *this;}	//!<Color components in the range [0,255] \return a ref to this
			virtual CRenderizable&  setColorA_u8(const uint8_t a)	{m_color.A=a; notifyChange(); return *this;}	//!<Color components in the range [0,255] \return a ref to this
			virtual CRenderizable& setColor_u8( const mrpt::utils::TColor &c) { CRenderizable::setColor_u8(c); notifyChange(); return *this; } //!< Changes the default object color \return a ref to this

			/** @} */

		};

	} // end namespace

} // End of namespace


#endif
