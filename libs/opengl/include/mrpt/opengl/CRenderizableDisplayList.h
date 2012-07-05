/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
