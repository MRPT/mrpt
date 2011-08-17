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
