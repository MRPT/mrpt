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

#ifndef opengl_CText_H
#define opengl_CText_H

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CText;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CText, CRenderizable, OPENGL_IMPEXP )

		/** A 2D text (bitmap rendering): it always "faces the observer" despite it's at some 3D location.
		  *  Use setString and setFont to change the text displayed by this object.
		  *
		  *  \note All texts appear with the font GLUT_BITMAP_TIMES_ROMAN_10 for now (i.e. setFont is ignored)
		  *  \sa opengl::COpenGLScene
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CText </td> <td> \image html preview_CText.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  *  \sa CText3D
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CText : public CRenderizable
		{
			DEFINE_SERIALIZABLE( CText )
		protected:
			std::string		m_str;
            std::string		m_fontName;
            int				m_fontHeight, m_fontWidth;

		public:
			void setString( const std::string &s ) { m_str=s; } //!< Sets the text to display
			std::string getString() const { return m_str; }  //!< Return the current text associated to this label

			void setFont(const std::string &s, int height ) { m_fontName=s; m_fontHeight=height; } //!< Sets the font (It has no effect yet!)
			std::string getFont() const { return m_fontName; }

			/** Render */
			void  render() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/** Class factory  */
			static CTextPtr Create(const std::string &str) { return CTextPtr( new CText(str) ); }

		private:
			/** Constructor */
			CText( const std::string &str = std::string("") );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CText();
		};

	} // end namespace

} // End of namespace


#endif
