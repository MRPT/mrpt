#ifndef CVD_GLES1_HELPERS_H
#define CVD_GLES1_HELPERS_H

#include <iostream>
#include <map>
#include <utility>

#include <cvd/image_ref.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/rgb8.h>
#include <cvd/rgba.h>
#include <cvd/config.h>
// #include <OpenGLES/ES1/gl.h>
#include <cvd/internal/gles1_types.h>

#ifdef CVD_HAVE_TOON
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>
#include <TooN/se2.h>
#include <TooN/so2.h>
#endif

namespace CVD
{

    /// add a translation specified by an ImageRef
    /// @param v the translation ImageRef
    /// @ingroup gGL
    inline void glTranslate( const ImageRef & v )
    {
        glTranslatef(v.x, v.y, 0);
    }

	/// add a translation specified from the first three coordinates of a vector
	/// @param v the translation vector
	/// @ingroup gGL
	template <int N, typename P, typename A> inline void glTranslate( const TooN::Vector<N, P, A> & v)
	{
		glTranslatef(v[0], v[1], v[2]);
	}

	/// add a translation specified from the first two coordinates of a 2-vector
	/// z is set to zero here
	/// @param v the translation vector
	/// @ingroup gGL
	template <typename P, typename A> inline void glTranslate( const TooN::Vector<2, P, A> & v)
	{
		glTranslatef(v[0], v[1], 0);
	}

	/// add a translation specified from the first coordinate of a 1-vector
	/// Y and Z are zero here
	/// @param v the translation vector
	/// @ingroup gGL
	template <typename P, typename A> inline void glTranslate( const TooN::Vector<1, P, A> & v)
	{
		glTranslatef(v[0], 0, 0);
	}

	/// multiply a TooN matrix onto the current matrix stack. Works for matrizes
	/// of size n >= 4 and uses the upper left 4x4 submatrix. The matrix is also
	/// transposed to account for GL's column major format.
	/// @param m the transformation matrix
	/// @ingroup gGL
	template <int N, class P, class A> inline void glMultMatrix( const TooN::Matrix<N,N,P,A> & m )
	{
		GLfloat glm[16];
		glm[0] = m[0][0]; glm[1] = m[1][0]; glm[2] = m[2][0]; glm[3] = m[3][0];
		glm[4] = m[0][1]; glm[5] = m[1][1]; glm[6] = m[2][1]; glm[7] = m[3][1];
		glm[8] = m[0][2]; glm[9] = m[1][2]; glm[10] = m[2][2]; glm[11] = m[3][2];
		glm[12] = m[0][3]; glm[13] = m[1][3]; glm[14] = m[2][3]; glm[15] = m[3][3];
		glMultMatrixf(glm);
	}

	/// multiply a TooN 3x3 matrix onto the current matrix stack. The GL matrix
	/// last column and row are set to 0 with the lower right element to 1.
	/// The matrix is also transposed to account for GL's column major format.
	/// @param m the transformation matrix
	/// @ingroup gGL
	template <class P, class A> inline void glMultMatrix( const TooN::Matrix<3,3,P,A> & m )
	{
		GLfloat glm[16];
		glm[0] = m[0][0]; glm[1] = m[1][0]; glm[2] = m[2][0]; glm[3] = 0;
		glm[4] = m[0][1]; glm[5] = m[1][1]; glm[6] = m[2][1]; glm[7] = 0;
		glm[8] = m[0][2]; glm[9] = m[1][2]; glm[10] = m[2][2]; glm[11] = 0;
		glm[12] = 0; glm[13] = 0; glm[14] = 0; glm[15] = 1;
		glMultMatrixf(glm);
	}

	/// multiply a TooN 2x2 matrix onto the current matrix stack. The TooN matrix
	/// will only occupy the upper left hand block, the remainder will be from the
	/// identity matrix. The matrix is also transposed to account for GL's column major format.
	/// @param m the transformation matrix
	/// @ingroup gGL
	template <class P, class A> inline void glMultMatrix( const TooN::Matrix<2,2,P,A> & m )
	{
		GLfloat glm[16];
		glm[0] = m[0][0]; glm[1] = m[1][0]; glm[2] = 0; glm[3] = 0;
		glm[4] = m[0][1]; glm[5] = m[1][1]; glm[6] = 0; glm[7] = 0;
		glm[8] = 0; glm[9] = 0; glm[10] = 1; glm[11] = 0;
		glm[12] = 0; glm[13] = 0; glm[14] = 0; glm[15] = 1;
		glMultMatrixf(glm);
	}

	/// multiplies a SO3 onto the current matrix stack
	/// @param so3 the SO3
	/// @ingroup gGL
	template <typename P>
	inline void glMultMatrix( const TooN::SO3<P> & so3 )
	{
		glMultMatrix( so3.get_matrix());
	}

	/// multiplies a SE3 onto the current matrix stack. This multiplies
	/// the SO3 and the translation in order.
	/// @param se3 the SE3
	/// @ingroup gGL
	template <typename P>
	inline void glMultMatrix( const TooN::SE3<P> & se3 )
	{
		glTranslate( se3.get_translation());
		glMultMatrix( se3.get_rotation());
	}

	/// multiplies a SO2 onto the current matrix stack
	/// @param so2 the SO2
	/// @ingroup gGL
	template <typename P>
	inline void glMultMatrix( const TooN::SO2<P> & so2 )
	{
		glMultMatrix( so2.get_matrix());
	}

	/// multiplies a SE2 onto the current matrix stack. This multiplies
	/// the SO2 and the translation in order.
	/// @param se3 the SE2
	/// @ingroup gGL
	template <typename P>
	inline void glMultMatrix( const TooN::SE2<P> & se2 )
	{
		glTranslate( se2.get_translation());
		glMultMatrix( se2.get_rotation());
	}

	/// Sets up an ortho projection suitable for drawing onto individual pixels of a
	/// gl window (or video image.) glVertex2f(0.0,0.0) will be the top left pixel and
	/// glVertex2f(xsize-1.0, ysize-1.0) will be the bottom right pixel. Depth is set
	/// from -1 to 1.
        /// n.b. You first need to set up the matrix environment yourself,
	/// e.g. glMatrixMode(GL_PROJECTION); glLoadIdentity();
	/// @param size ImageRef containing the size of the GL window.
	template <typename P> inline void glOrtho( const CVD::ImageRef & size, const P nearPlane = -1.0, const P farPlane = 1.0)
	{
	    ::glOrthof( -0.375, size.x - 0.375, size.y - 0.375, -0.375, nearPlane, farPlane );
	}

	/// Sets up an ortho projection from a simple Vector<6>
        /// n.b. You first need to set up the matrix environment yourself,
	/// e.g. glMatrixMode(GL_PROJECTION); glLoadIdentity();
	/// @param param 6-vector containing the parameters of the projection
	template <typename P, typename A> inline void glOrtho( const TooN::Vector<6, P, A> & param)
	{
		::glOrthof( param[0], param[1], param[2], param[3], param[4], param[5]);
	}

	/// sets a gl frustum from the linear camera parameters, image size and near and far plane.
	/// The camera will be in OpenGL style with camera center in the origin and the viewing direction
	/// down the negative z axis, with y pointing upwards and x pointing to the left and the image plane
	/// at z=-1.
	/// Images coordinates need to be rotated around the x axis to make sense here, because typically
	/// the camera is described as y going down (pixel lines) and image plane at z=1.
	/// @param params vector containing fu, fv, pu, pv as in the linear part of camera parameters
	/// @param width width of the image plane in pixels, here the viewport for example
	/// @param height height of the image plane in pixels, here the viewport for example
	/// @param near near clipping plane
	/// @param far far clipping plane
	/// @ingroup gGL
	template <typename P, typename A>
	inline void glFrustum( const TooN::Vector<4, P, A> & params, const float width, const float height, const float nearPlane = 0.1, const float farPlane = 100)
	{
		GLfloat left, right, bottom, top;
		left = -nearPlane * params[2] / params[0];
		top = nearPlane * params[3] / params[1];
		right = nearPlane * ( width - params[2] ) / params[0];
		bottom = - nearPlane * ( height - params[3] ) / params[1];
		::glFrustumf( left, right, bottom, top, nearPlane, farPlane );
	}

	/// sets a gl frustum taking the first 4 parameters from the camera model. see @see glFrustum for
	/// details on the created frustum.
	/// @param camera camera supplying the parameters for the frustum
	/// @param width width of the image plane in pixels, here the viewport for example
	/// @param height height of the image plane in pixels, here the viewport for example
	/// @param near near clipping plane
	/// @param far far clipping plane
	/// @ingroup gGL
	template <class CAMERA> inline void glFrustum( const CAMERA & camera, const float width, const float height, const float nearPlane = 0.1, const float farPlane = 100)
	{
		glFrustum( camera.get_parameters().template slice<0,4>(), width, height, nearPlane, farPlane);
	}

	/// Sets up an ortho projection from a simple Vector<6>
	/// n.b. You first need to set up the matrix environment yourself,
	/// e.g. glMatrixMode(GL_PROJECTION); glLoadIdentity();
	/// @param param 6-vector containing the parameters of the projection
	template <typename P, typename A> inline void glFrustum( const TooN::Vector<6, P, A> & param)
	{
		::glFrustumf( param[0], param[1], param[2], param[3], param[4], param[5]);
	}
    
	/// Set the new colour to the red, green and blue components given in the Vector
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param v The new colour
	///  @ingroup gGL
	template <typename P, typename A> inline void glColor(const TooN::Vector<3, P, A> & v)
	{
		glColor4f(v[0], v[1], v[2], 1);
	}

	/// Set the new colour to the red, green, blue and alpha components given in the Vector
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param v The new colour
	///@ingroup gGL
	template <typename P, typename A> inline void glColor(const TooN::Vector<4, P, A>& v)
	{
		glColor4f(v[0], v[1], v[2], v[3]);
	}
    
	/// Set the new clear colour to the red, green, blue and alpha components given in the Vector
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param v The new colour
	///@ingroup gGL
	template <typename P, typename A> inline void glClearColor(const TooN::Vector<4, P, A> & v)
	{
		::glClearColor((GLclampf)v[0], (GLclampf)v[1], (GLclampf)v[2], (GLclampf)v[3]);
	}

	/// Set the new clear colour to the red, green, blue components given in the Vector
	/// alpha is set to 1
	/// @param v The new colour
	///@ingroup gGL
	template <typename P, typename A> inline void glClearColor(const TooN::Vector<3, P, A>& v)
	{
		::glClearColor((GLclampf)v[0], (GLclampf)v[1], (GLclampf)v[2], 1);
	}

	/// glColor version for dynamic TooN::Vector, will test for 3 or 4 components
	/// @param v The new colour
	/// @ingroup gGL
	template <typename P, typename A> inline void glColor(const TooN::Vector<-1, P, A> & v)
	{
		switch(v.size()){
		case 3: glColor4f(v[0], v[1], v[2], 1);
			break;
		case 4: glColor4f(v[0], v[1], v[2], v[3]);
			break;
		}
	}

	/// Set the new colour to the red, green, blue components given
	/// (where 0 represents zero intensity and 255 full intensity)
	/// @param c The new colour
	///@ingroup gGL
	inline void glColor(const CVD::Rgb<byte>& c)
	{
		glColor4ub(c.red, c.green, c.blue, 255);
	}

 	/// Set the new colour to the red, green and blue components given
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param c The new colour
	///@ingroup gGL
	inline void glColor(const CVD::Rgb<float>& c)
	{
		glColor4f(c.red, c.green, c.blue, 1);
	}

 	/// Set the new colour to the red, green and blue components given
	/// (where 0.0 represents zero intensity and 255 full intensity). The Rgb8::dummy member is ignored
	/// @param c The new colour
	///@ingroup gGL
	inline void glColor3(const CVD::Rgb8& c)
	{
		glColor4ub(c.red, c.green, c.blue, 255);
	}

 	/// Set the new colour to the red, green, blue and alpha components given
	/// (where 0.0 represents zero intensity and 255 full intensity)
	/// @param c The new colour
	///@ingroup gGL
	inline void glColor4(const CVD::Rgb8& c)
	{
		glColor4ub(c.red, c.green, c.blue, c.dummy);
	}

 	/// Set the new colour to the red, green, blue and alpha components given
	/// (where 0 represents zero intensity and 255 full intensity)
	/// @param c The new colour
	///@ingroup gGL
	inline void glColor(const CVD::Rgba<unsigned char>& c)
	{
		glColor4ub(c.red, c.green, c.blue, c.alpha);
	}

 	/// Set the new colour to the red, green, blue and alpha components given
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param c The new colour
	///@ingroup gGL
	inline void glColor(const CVD::Rgba<float>& c)
	{
		glColor4f(c.red, c.green, c.blue, c.alpha);
	}

 	/// Read the current image from the colour buffer specified by glReadBuffer
	/// @param i The image to write the image data into. This must already be initialised to be an BasicImage (or Image) of the right size.
	/// @param origin The window co-ordinate of the first pixel to be read from the frame buffer
	///@ingroup gGL
	template<class C> inline void glReadPixels(BasicImage<C>& i, const ImageRef origin=ImageRef(0,0))
	{
		::glReadPixels(origin.x, origin.y, i.size().x, i.size().y, gl::data<C>::format, gl::data<C>::type, i.data());
	}

 	/// Read the current image from the colour buffer specified by glReadBuffer
	/// @param size   The size of the area to read.
	/// @param origin The window co-ordinate of the first pixel to be read from the frame buffer
	///@ingroup gGL
	template<class C> inline Image<C> glReadPixels(ImageRef size, const ImageRef origin=ImageRef(0,0))
	{
		Image<C> i(size);
		::glReadPixels(origin.x, origin.y, i.size().x, i.size().y, gl::data<C>::format, gl::data<C>::type, i.data());
		return i;
	}

	/// Sets an image as a texture sub region.
	/// note the reordering of the various parameters to make better use of default parameters
	/// @param i the image to set as texture
	/// @ingroup gGL
	template<class C> inline void glTexBasicImage2D( const BasicImage<C> &i, GLint xoffset = 0, GLint yoffset = 0, GLenum target = GL_TEXTURE_2D, GLint level = 0)
	{
		::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		::glTexBasicImage2D(target, level, xoffset, yoffset, i.size().x, i.size().y, gl::data<C>::format, gl::data<C>::type, i.data());
	}

	/// Sets an image as a texture.
	/// note the reordering of the various parameters to make better use of default parameters
	/// @param i the image to set as texture
	/// @ingroup gGL
	template<class C> inline void glTexImage2D( const BasicImage<C> &i, GLint border = 0, GLenum target = GL_TEXTURE_2D, GLint level = 0)
	{
		::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		::glTexImage2D(target, level, gl::data<C>::format, i.size().x, i.size().y, border, gl::data<C>::format, gl::data<C>::type, i.data());
	}

    /// Prints the current errors on the gl error stack
    ///@ingroup gGL
    inline void glPrintErrors(void){
        GLenum code;
        while((code = glGetError()) != GL_NO_ERROR){
            std::cout << "GL:" << code << std::endl;
        }
    }

    /// @defgroup gGLText OpenGL text rendering
    /// @ingroup gGL
    /// @{

    /// sets the font to use for future font rendering commands. currently sans, serif and mono are available.
    /// @param fontname string containing font name
    void glSetFont( const std::string & fontname );

    /// returns the name of the currently active font
    const std::string & glGetFont();

    /// different style for font rendering
    enum TEXT_STYLE {
        FILL = 0,       ///< renders glyphs as filled polygons
        OUTLINE = 1,    ///< renders glyphs as outlines with GL_LINES
        NICE = 2        ///< renders glyphs filled with antialiased outlines
    };

    /// renders a string in GL using the current settings.
    /// Font coordinates are +X along the line and +Y along the up direction of glyphs.
    /// The origin is at the top baseline at the left of the first character. Characters have a maximum size of 1.
    /// linefeed is interpreted as a new line and the start is offset in -Y direction by @ref spacing . Individual characters
    /// are separated by @ref kerning + plus their individual with.
    /// @param text string to be rendered, unknown characters are replaced with '?'
    /// @param style rendering style
    /// @param spacing distance between individual text lines
    /// @param kerning distance between characters
    std::pair<double, double> glDrawText(const std::string & text, enum TEXT_STYLE style = NICE, double spacing = 1.5, double kerning = 0.1);

    /// returns the size of the bounding box of a text to be rendered, similar to @ref glDrawText but without any visual output
    std::pair<double, double> glGetExtends(const std::string & text, double spacing = 1.5, double kerning = 0.1);

    ///@}

};

#endif
