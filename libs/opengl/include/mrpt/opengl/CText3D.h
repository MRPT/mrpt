/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#ifndef opengl_CText3D_H
#define opengl_CText3D_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CText3D;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CText3D, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A 3D text (rendered with OpenGL primitives), with selectable font face and drawing style.
		  *  Use \a setString and \a setFont to change the text displayed by this object (can be multi-lined).
		  *
		  *  Text is drawn along the (+X,+Y) axes.
		  *
		  *  Default size of characters is "1.0 units". Change it with the standard method \a CRenderizable::setScale() as with any other 3D object.
		  *  The color can be also changed with standard methods in the base class \a CRenderizable.
		  *
		  *  \sa opengl::COpenGLScene, CText
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CText3D </td> <td> \image html preview_CText3D.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  * \note This class is based on code from libcvd (LGPL, http://www.edwardrosten.com/cvd/ )
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CText3D : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CText3D )
		protected:
			std::string		m_str;
            std::string		m_fontName;
            TOpenGLFontStyle m_text_style;
			double          m_text_spacing;
			double          m_text_kerning;

		public:
			/** Sets the displayed string */
			inline void setString( const std::string &s ) {
				m_str=s;
				CRenderizableDisplayList::notifyChange();
			}
			/** Returns the currently text associated to this object */
			inline const std::string &getString() const { return m_str; }

			/** Changes the font name, among accepted values: "sans", "mono", "serif" */
			inline void setFont( const std::string &font ) {
				m_fontName=font;
				CRenderizableDisplayList::notifyChange();
			}
			/** Returns the text font  */
			inline const std::string &getFont() const { return m_fontName; }

			/** Change drawing style: FILL, OUTLINE, NICE */
			void setTextStyle(const mrpt::opengl::TOpenGLFontStyle text_style) {
				m_text_style = text_style;
				CRenderizableDisplayList::notifyChange();
			}
			/** Gets the current drawing style */
			mrpt::opengl::TOpenGLFontStyle getTextStyle() const { return m_text_style; }

			void setTextSpacing(const double text_spacing) {
				m_text_spacing = text_spacing;
				CRenderizableDisplayList::notifyChange();
			 }
			double setTextSpacing() const { return m_text_spacing; }

			void setTextKerning(const double text_kerning) {
				m_text_kerning = text_kerning;
				CRenderizableDisplayList::notifyChange();
			 }
			double setTextKerning() const { return m_text_kerning; }


			/** Render */
			void  render_dl() const;


			/** Class factory  */
			static CText3DPtr Create(
				const std::string &str,
				const std::string &fontName = std::string("sans"),
				const double scale = 1.0,
				const mrpt::opengl::TOpenGLFontStyle text_style = mrpt::opengl::FILL,
				const double text_spacing = 1.5,
				const double text_kerning = 0.1 )
				{
					return CText3DPtr( new CText3D(str,fontName,scale,text_style,text_spacing,text_kerning) );
				}

		private:
			/** Constructor */
			CText3D(
				const std::string &str = std::string(""),
				const std::string &fontName = std::string("sans"),
				const double scale = 1.0,
				const mrpt::opengl::TOpenGLFontStyle text_style = mrpt::opengl::FILL,
				const double text_spacing = 1.5,
				const double text_kerning = 0.1 );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CText3D();
		};

	} // end namespace

} // End of namespace


#endif
