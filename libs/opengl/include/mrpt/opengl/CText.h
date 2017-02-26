/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CText_H
#define opengl_CText_H

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt
{
	namespace opengl
	{


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
			void  render() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/** Class factory  */
			static CTextPtr Create(const std::string &str);

		private:
			/** Constructor */
			CText( const std::string &str = std::string("") );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CText();
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CText, CRenderizable, OPENGL_IMPEXP )

	} // end namespace

} // End of namespace


#endif
