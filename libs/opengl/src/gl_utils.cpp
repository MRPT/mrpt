/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/gl_utils.h>		// Include these before windows.h!!
#include <mrpt/system/os.h>
#include "opengl_internals.h"

#include <map>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace mrpt::utils;


/** For each object in the list:
*   - checks visibility of each object
*   - prepare the GL_MODELVIEW matrix according to its coordinates
*   - call its ::render()
*   - shows its name (if enabled).
*/
void gl_utils::renderSetOfObjects(const CListOpenGLObjects &objectsToRender)
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_PROFILE_FUNC_START // Just the non-try/catch part of MRPT_START

	CListOpenGLObjects::const_iterator	itP;
	try
	{
		for (itP=objectsToRender.begin();itP!=objectsToRender.end();++itP)
		{
			if (!itP->present()) continue;
			const CRenderizable * it = itP->pointer(); // Use plain pointers, faster than smart pointers:
			if (!it->isVisible()) continue;

			// 3D coordinates transformation:
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			//glPushAttrib(GL_ALL_ATTRIB_BITS);
			//gl_utils::checkOpenGLError();

			// It's more efficient to prepare the 4x4 matrix ourselves and load it directly into opengl stack:
			//  A homogeneous transformation matrix, in this order:
			//
			//     0  4  8  12
			//     1  5  9  13
			//     2  6  10 14
			//     3  7  11 15
			//
			const CPose3D & pos = it->getPoseRef();
			const CMatrixDouble33 &R = pos.getRotationMatrix();
			const GLdouble m[16] = {
				R.coeff(0,0),R.coeff(1,0),R.coeff(2,0), 0,
				R.coeff(0,1),R.coeff(1,1),R.coeff(2,1), 0,
				R.coeff(0,2),R.coeff(1,2),R.coeff(2,2), 0,
				pos.m_coords[0],pos.m_coords[1],pos.m_coords[2], 1
				};
			glMultMatrixd( m );  // Multiply so it's composed with the previous, current MODELVIEW matrix

			// Do scaling after the other transformations!
			if (it->getScaleX()!=1 || it->getScaleY()!=1 || it->getScaleZ()!=1)
				glScalef(it->getScaleX(),it->getScaleY(),it->getScaleZ());

			// Set color:
			glColor4f( it->getColorR(),it->getColorG(),it->getColorB(),it->getColorA());

			it->render();
			gl_utils::checkOpenGLError();

			if (it->isShowNameEnabled())
			{
				glDisable(GL_DEPTH_TEST);
				glColor3f(1.f,1.f,1.f);  // Must be called BEFORE glRasterPos3f
				glRasterPos3f(0.0f,0.0f,0.0f);

				GLfloat		raster_pos[4];
				glGetFloatv( GL_CURRENT_RASTER_POSITION, raster_pos);
				float eye_distance= raster_pos[3];

				void *font=NULL;
				if (eye_distance<2)
						font = GLUT_BITMAP_TIMES_ROMAN_24;
				else if(eye_distance<200)
					font = GLUT_BITMAP_TIMES_ROMAN_10;

				if (font)
					CRenderizable::renderTextBitmap( it->getName().c_str(), font);

				glEnable(GL_DEPTH_TEST);
			}

			//glPopAttrib();
//			gl_utils::checkOpenGLError();

			glPopMatrix();
			gl_utils::checkOpenGLError();

		} // end foreach object
	}
	catch(exception &e)
	{
		char str[1000];
		os::sprintf(str,1000,"Exception while rendering a class '%s'\n%s",
			(*itP)->GetRuntimeClass()->className,
			e.what() );
		THROW_EXCEPTION(str);
	}
	catch(...)
	{
		THROW_EXCEPTION("Runtime error!");
	}
#else
	MRPT_UNUSED_PARAM(objectsToRender);
#endif
}

/*---------------------------------------------------------------
					checkOpenGLError
  ---------------------------------------------------------------*/
void	gl_utils::checkOpenGLError()
{
#if MRPT_HAS_OPENGL_GLUT
	int	 openglErr;
	if ( ( openglErr= glGetError()) != GL_NO_ERROR )
	{
		const std::string sErr = std::string("OpenGL error: ") + std::string( (char*)gluErrorString(openglErr) );
		std::cerr << "[gl_utils::checkOpenGLError] " << sErr << std::endl;
		//THROW_EXCEPTION(sErr)
	}
#endif
}

void gl_utils::renderTriangleWithNormal( const mrpt::math::TPoint3D &p1,const mrpt::math::TPoint3D &p2,const mrpt::math::TPoint3D &p3 )
{
#if MRPT_HAS_OPENGL_GLUT
	const float	ax= p2.x - p1.x;
    const float	ay= p2.y - p1.y;
	const float	az= p2.z - p1.z;

	const float	bx= p3.x - p1.x;
	const float	by= p3.y - p1.y;
	const float	bz= p3.z - p1.z;

	glNormal3f(ay*bz-az*by,-ax*bz+az*bx,ax*by-ay*bx);

	glVertex3f(p1.x,p1.y,p1.z);
	glVertex3f(p2.x,p2.y,p2.z);
	glVertex3f(p3.x,p3.y,p3.z);
#else
	MRPT_UNUSED_PARAM(p1); MRPT_UNUSED_PARAM(p2); MRPT_UNUSED_PARAM(p3);
#endif
}
void gl_utils::renderTriangleWithNormal( const mrpt::math::TPoint3Df &p1,const mrpt::math::TPoint3Df &p2,const mrpt::math::TPoint3Df &p3 )
{
#if MRPT_HAS_OPENGL_GLUT
	const float	ax= p2.x - p1.x;
    const float	ay= p2.y - p1.y;
	const float	az= p2.z - p1.z;

	const float	bx= p3.x - p1.x;
	const float	by= p3.y - p1.y;
	const float	bz= p3.z - p1.z;

	glNormal3f(ay*bz-az*by,-ax*bz+az*bx,ax*by-ay*bx);

	glVertex3f(p1.x,p1.y,p1.z);
	glVertex3f(p2.x,p2.y,p2.z);
	glVertex3f(p3.x,p3.y,p3.z);
#else
	MRPT_UNUSED_PARAM(p1); MRPT_UNUSED_PARAM(p2); MRPT_UNUSED_PARAM(p3);
#endif
}
void gl_utils::renderQuadWithNormal( const mrpt::math::TPoint3Df &p1,const mrpt::math::TPoint3Df &p2,const mrpt::math::TPoint3Df &p3, const mrpt::math::TPoint3Df &p4 )
{
	renderTriangleWithNormal(p1,p2,p3);
	renderTriangleWithNormal(p3,p4,p1);
}


/** Gather useful information on the render parameters.
  *  It can be called from within the render() method of derived classes.
  */
void gl_utils::getCurrentRenderingInfo(TRenderInfo &ri)
{
#if MRPT_HAS_OPENGL_GLUT
	// Viewport geometry:
	GLint	win_dims[4];
	glGetIntegerv( GL_VIEWPORT, win_dims );
	ri.vp_x      = win_dims[0];
	ri.vp_y      = win_dims[1];
	ri.vp_width  = win_dims[2];
	ri.vp_height = win_dims[3];

	// Get the inverse camera position:
	GLfloat  mat_proj[16];
	glGetFloatv(GL_PROJECTION_MATRIX,mat_proj);
	ri.proj_matrix = Eigen::Matrix<float,4,4,Eigen::ColMajor>(mat_proj);

	// Extract the camera position:
	Eigen::Matrix<float,4,1> cam_pose_hm = ri.proj_matrix.inverse().col(3);
	if (cam_pose_hm[3]!=0)
	{
		ri.camera_position.x = cam_pose_hm[0]/cam_pose_hm[3];
		ri.camera_position.y = cam_pose_hm[1]/cam_pose_hm[3];
		ri.camera_position.z = cam_pose_hm[2]/cam_pose_hm[3];
	}
	else ri.camera_position= mrpt::math::TPoint3Df(0,0,0);

	// Get the model transformation:
	GLfloat  mat_mod[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,mat_mod);
	ri.model_matrix = Eigen::Matrix<float,4,4,Eigen::ColMajor>(mat_mod);

	// PROJ * MODEL
	ri.full_matrix = ri.proj_matrix * ri.model_matrix;
#else
	MRPT_UNUSED_PARAM(ri);
#endif
}

/*---------------------------------------------------------------
					renderTextBitmap
  ---------------------------------------------------------------*/
void	gl_utils::renderTextBitmap( const char *str, void *fontStyle )
{
#if MRPT_HAS_OPENGL_GLUT
	while ( *str ) glutBitmapCharacter( fontStyle ,*(str++) );
#else
	MRPT_UNUSED_PARAM(str); MRPT_UNUSED_PARAM(fontStyle);
#endif
}


void *aux_mrptfont2glutfont(const TOpenGLFont font)
{
#if MRPT_HAS_OPENGL_GLUT
	switch (font)
	{
	default:
	case MRPT_GLUT_BITMAP_TIMES_ROMAN_10: return GLUT_BITMAP_TIMES_ROMAN_10; break;
	case MRPT_GLUT_BITMAP_TIMES_ROMAN_24: return GLUT_BITMAP_TIMES_ROMAN_24; break;

	case MRPT_GLUT_BITMAP_HELVETICA_10: return GLUT_BITMAP_HELVETICA_10; break;
	case MRPT_GLUT_BITMAP_HELVETICA_12: return GLUT_BITMAP_HELVETICA_12; break;
	case MRPT_GLUT_BITMAP_HELVETICA_18: return GLUT_BITMAP_HELVETICA_18; break;
	}
#else
	MRPT_UNUSED_PARAM(font);
	return NULL;
#endif
}

/** Return the exact width in pixels for a given string, as will be rendered by renderTextBitmap().
  * \sa renderTextBitmap
  */
int gl_utils::textBitmapWidth(
	const std::string &str,
	mrpt::opengl::TOpenGLFont    font)
{
#if MRPT_HAS_OPENGL_GLUT
	if (str.empty()) return 0;
	return glutBitmapLength(aux_mrptfont2glutfont(font), (const unsigned char*)str.c_str() );
#else
	MRPT_UNUSED_PARAM(str); MRPT_UNUSED_PARAM(font);
	return 10;
#endif
}

/*---------------------------------------------------------------
					renderTextBitmap
  ---------------------------------------------------------------*/
void CRenderizable::renderTextBitmap(
	int screen_x,
	int screen_y,
	const std::string &str,
	float  color_r,
	float  color_g,
	float  color_b,
	TOpenGLFont font
	)
{
#if MRPT_HAS_OPENGL_GLUT
    glDisable(GL_DEPTH_TEST);

	// If (x,y) are negative, wrap to the opposite side:
	if (screen_x<0 || screen_y<0)
	{
		// Size of the viewport:
		GLint	win_dims[4];  // [2]:width ,[3]:height
		glGetIntegerv( GL_VIEWPORT, win_dims );

		if (screen_x<0) screen_x += win_dims[2];
		if (screen_y<0) screen_y += win_dims[3];
	}

	// Draw text:
    glColor3f(color_r,color_g,color_b);

    // From: http://www.mesa3d.org/brianp/sig97/gotchas.htm
	GLfloat fx, fy;

	/* Push current matrix mode and viewport attributes */
	glPushAttrib( GL_TRANSFORM_BIT | GL_VIEWPORT_BIT );

	/* Setup projection parameters */
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	//glDepthRange( z, z );
	glViewport( (int) screen_x - 1, (int) screen_y - 1, 2, 2 );

	/* set the raster (window) position */
	fx = screen_x - (int) screen_x;
	fy = screen_y - (int) screen_y;
	//glRasterPos4f( fx, fy, 0.0, w );
	glRasterPos3f( fx, fy, 0.0 );

	/* restore matrices, viewport and matrix mode */
	glPopMatrix();
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();

	glPopAttrib();

	// Select font:
	void *glut_font_sel = aux_mrptfont2glutfont(font);

	for (size_t i=0;i<str.size();i++)
		glutBitmapCharacter( glut_font_sel ,str[i] );

    glEnable(GL_DEPTH_TEST);
#else
	MRPT_UNUSED_PARAM(screen_x); MRPT_UNUSED_PARAM(screen_y);
	MRPT_UNUSED_PARAM(str); MRPT_UNUSED_PARAM(color_r); MRPT_UNUSED_PARAM(color_g);
	MRPT_UNUSED_PARAM(color_b); MRPT_UNUSED_PARAM(font);
#endif
}


void gl_utils::renderMessageBox(
	const float msg_x, const float msg_y,
	const float msg_w, const float msg_h,
	const std::string &text,
	float text_scale,
	const mrpt::utils::TColor &back_col,
	const mrpt::utils::TColor &border_col,
	const mrpt::utils::TColor &text_col,
	const float border_width,
	const std::string & text_font,
	mrpt::opengl::TOpenGLFontStyle text_style,
	const double text_spacing,
	const double text_kerning
	)
{
#if MRPT_HAS_OPENGL_GLUT
	const int nLines = 1 + std::count(text.begin(),text.end(), '\n');

	GLint	win_dims[4];
	glGetIntegerv( GL_VIEWPORT, win_dims );
	const int w = win_dims[2];
	const int h = win_dims[3];

	const int min_wh = std::min(w,h);
	const float vw_w = w/float(min_wh);
	const float vw_h = h/float(min_wh);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glLoadIdentity();
	glOrtho(0,vw_w,0,vw_h,-1,1);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// The center of the message box:
	const float msg_x0 = vw_w * msg_x;
	const float msg_y0 = vw_h * msg_y;

	const float msg_x1 = vw_w * (msg_x+msg_w);
	const float msg_y1 = vw_h * (msg_y+msg_h);

	const float msg_real_w = msg_x1-msg_x0;
	const float msg_real_h = msg_y1-msg_y0;

	const float msg_cx = .5*(msg_x0+msg_x1);
	const float msg_cy = .5*(msg_y0+msg_y1);

	// Background:
	glColor4ub(back_col.R,back_col.G,back_col.B,back_col.A);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f( msg_x0,msg_y0 );
	glVertex2f( msg_x1,msg_y0 );
	glVertex2f( msg_x1,msg_y1 );
	glVertex2f( msg_x0,msg_y1 );
	glEnd();

	// Border:
	glColor4ub(border_col.R,border_col.G,border_col.B,border_col.A);
	glLineWidth(border_width);
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINE_LOOP);
	glVertex2f( msg_x0,msg_y0 );
	glVertex2f( msg_x1,msg_y0 );
	glVertex2f( msg_x1,msg_y1 );
	glVertex2f( msg_x0,msg_y1 );
	glEnd();
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines


	// Draw text (centered):
	gl_utils::glSetFont(text_font);
	mrpt::utils::TPixelCoordf txtSize = gl_utils::glGetExtends(text,text_scale,text_spacing,text_kerning);

	// Adjust text size if it doesn't fit into the box:
	if (txtSize.x>msg_real_w)
	{
		const float K = 0.99f * msg_real_w/txtSize.x;
		text_scale *= K;
		txtSize.x *=K; txtSize.y *=K;
	}
	if (txtSize.y>msg_real_h)
	{
		const float K = 0.99f * msg_real_h/txtSize.y;
		text_scale *= K;
		txtSize.x *=K; txtSize.y *=K;
	}

	const float text_w = txtSize.x;
	const float text_h = (nLines>1 ? -(nLines-1)*txtSize.y/float(nLines) : txtSize.y);
	const float text_x0 = msg_cx-.5f*text_w;
	const float text_y0 = msg_cy-.5f*text_h;

	glTranslatef(text_x0,text_y0,0);
	glColor4ub(text_col.R,text_col.G,text_col.B,text_col.A);
	gl_utils::glDrawText(text,text_scale,text_style,text_spacing,text_kerning);

	// Restore gl flags:
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
#else
	MRPT_UNUSED_PARAM(msg_x); MRPT_UNUSED_PARAM(msg_y);
	MRPT_UNUSED_PARAM(msg_w); MRPT_UNUSED_PARAM(msg_h);
	MRPT_UNUSED_PARAM(text); MRPT_UNUSED_PARAM(text_scale);
	MRPT_UNUSED_PARAM(back_col); MRPT_UNUSED_PARAM(border_col);
	MRPT_UNUSED_PARAM(text_col); MRPT_UNUSED_PARAM(border_width);
	MRPT_UNUSED_PARAM(text_font); MRPT_UNUSED_PARAM(text_style);
	MRPT_UNUSED_PARAM(text_spacing); MRPT_UNUSED_PARAM(text_kerning);
#endif
}


//  ===============  START OF CODE FROM "libcvd -> gltext.cpp" ===============
//    License: LGPL
#if MRPT_HAS_OPENGL_GLUT
namespace Internal
{
	struct Point
	{
		float x,y;
	};

	struct Font {
		typedef unsigned short Index;

		struct Char {
			Index vertexOffset;
			Index triangleOffset;
			Index outlineOffset;
			GLsizei numTriangles;
			GLsizei numOutlines;
			float advance;
		};

		Point * vertices;
		Index * triangles;
		Index * outlines;
		Char * characters;
		string glyphs;

		const Char * findChar( const char c ) const {
			size_t ind = glyphs.find(c);
			if(ind == string::npos)
				return NULL;
			return characters + ind;
		}

		float getAdvance( const char c ) const {
			const Char * ch = findChar(c);
			if(!ch)
				return 0;
			return ch->advance;
		}

		void fill( const char c ) const {
			const Char * ch = findChar(c);
			if(!ch || !ch->numTriangles)
				return;
			glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
			glDisable(GL_LIGHTING);  // Disable lights when drawing lines
			glDrawElements(GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT, triangles + ch->triangleOffset);
			glEnable(GL_LIGHTING);
		}

		void outline( const char c ) const {
			const Char * ch = findChar(c);
			if(!ch || !ch->numOutlines)
				return;
			glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
			glDisable(GL_LIGHTING);  // Disable lights when drawing lines
			glDrawElements(GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT, outlines + ch->outlineOffset);
			glEnable(GL_LIGHTING);
		}

		void draw( const char c ) const {
			const Char * ch = findChar(c);
			if(!ch || !ch->numTriangles || !ch->numOutlines)
				return;
			glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
			glDisable(GL_LIGHTING);  // Disable lights when drawing lines
			glDrawElements(GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT, triangles + ch->triangleOffset);
			glDrawElements(GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT, outlines + ch->outlineOffset);
			glEnable(GL_LIGHTING);  // Disable lights when drawing lines
		}
	};

	// the fonts defined in these headers are derived from Bitstream Vera fonts. See http://www.gnome.org/fonts/ for license and details
	#include "glfont_sans.h"
	#include "glfont_mono.h"
	#include "glfont_serif.h"

	struct FontData {

		typedef map<string,Font *> FontMap;

		FontData() {
			fonts["sans"] = &sans_font;
			fonts["mono"] = &mono_font;
			fonts["serif"] = &serif_font;
			gl_utils::glSetFont("sans");
		}
		inline Font * currentFont(){
			return fonts[currentFontName];
		}

		string currentFontName;
		FontMap fonts;
	};

	static struct FontData data;
} // namespace Internal
#endif

void gl_utils::glSetFont( const std::string & fontname ){
#if MRPT_HAS_OPENGL_GLUT
    if(Internal::data.fonts.count(fontname) > 0)
        Internal::data.currentFontName = fontname;
#else
	MRPT_UNUSED_PARAM(fontname);
#endif
}

const std::string & gl_utils::glGetFont(){
#if MRPT_HAS_OPENGL_GLUT
    return Internal::data.currentFontName;
#else
    THROW_EXCEPTION("MRPT built without OpenGL")
#endif
}

mrpt::utils::TPixelCoordf gl_utils::glDrawText(const std::string& text, const double textScale, enum TOpenGLFontStyle style, double spacing, double kerning){
#if MRPT_HAS_OPENGL_GLUT
	glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    if(style == NICE) {
        glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1);
    }
    glEnableClientState(GL_VERTEX_ARRAY);

    // figure out which operation to do on the Char (yes, this is a pointer to member function :)
    void (Internal::Font::* operation)(const char c) const;
    operation=NULL;
    switch(style){
        case FILL: operation = &Internal::Font::fill;
            break;
        case OUTLINE: operation = &Internal::Font::outline;
            break;
        case NICE: operation = &Internal::Font::draw;
            break;
        default: THROW_EXCEPTION("Invalid text style value.");
    }

    // Scale of the text:
    glScaled(textScale,textScale,textScale);

    int lines = 0;
    double max_total = 0;
    double total=0;
    const Internal::Font * font = Internal::data.currentFont();
    const Internal::Font::Char * space = font->findChar(' ');
    const double tab_width = 8 * ((space)?(space->advance):1);
    for (size_t i=0; i<text.length(); ++i) {
        char c = text[i];
        if (c == '\n') {
            glTranslated(-total,-spacing, 0);
            max_total = std::max(max_total, total);
            total = 0;
            ++lines;
            continue;
        }
        if(c == '\t'){
            const float advance = tab_width - std::fmod(total, tab_width);
            total += advance;
            glTranslated(advance, 0, 0);
            continue;
        }
        const Internal::Font::Char * ch = font->findChar(c);
        if(!ch){
            c = toupper(c);
            ch = font->findChar(c);
            if(!ch) {
                c = '?';
                ch = font->findChar(c);
            }
        }
        if(!ch)
            continue;
        (font->*operation)(c);

        double w = ch->advance + kerning;
        glTranslated(w, 0, 0);
        total += w;
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    if(style == NICE){
        glPopAttrib();
    }
    glPopMatrix();

    max_total = std::max(total, max_total);

    return mrpt::utils::TPixelCoordf(textScale*max_total, textScale*(lines+1)*spacing);
#else
	MRPT_UNUSED_PARAM(text); MRPT_UNUSED_PARAM(textScale); MRPT_UNUSED_PARAM(style);
	MRPT_UNUSED_PARAM(spacing); MRPT_UNUSED_PARAM(kerning);
	THROW_EXCEPTION("MRPT built without OpenGL")
#endif
}

mrpt::utils::TPixelCoordf gl_utils::glGetExtends(const std::string & text,  const double textScale, double spacing, double kerning)
{
#if MRPT_HAS_OPENGL_GLUT
    int lines = 0;
    double max_total = 0;
    double total=0;
    const Internal::Font * font = Internal::data.currentFont();
    for (size_t i=0; i<text.length(); ++i) {
        char c = text[i];
        if (c == '\n') {
            max_total = std::max(max_total, total);
            total = 0;
            ++lines;
            continue;
        }
        const Internal::Font::Char * ch = font->findChar(c);
        if(!ch){
            c = toupper(c);
            ch = font->findChar(c);
            if(!ch) {
                c = '?';
                ch = font->findChar(c);
            }
        }
        if(!ch)
            continue;
        total += ch->advance + kerning;
    }
    max_total = std::max(total, max_total);
    return mrpt::utils::TPixelCoordf(textScale*max_total, textScale*(lines+1)*spacing);
#else
	MRPT_UNUSED_PARAM(text); MRPT_UNUSED_PARAM(textScale);
	MRPT_UNUSED_PARAM(spacing); MRPT_UNUSED_PARAM(kerning);
	THROW_EXCEPTION("MRPT built without OpenGL")
#endif
}
//  ===============  END OF CODE FROM "libcvd -> gltext.cpp" ===============
