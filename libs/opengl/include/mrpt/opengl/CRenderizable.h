/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef opengl_CRenderizable_H
#define opengl_CRenderizable_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/opengl/link_pragmas.h>

namespace mrpt
{
	namespace poses { class CPose3D; class CPoint3D; class CPoint2D; }
	namespace utils { class CStringList; }

	namespace opengl
	{
		class COpenGLViewport;
		class CSetOfObjects;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRenderizable, mrpt::utils::CSerializable, OPENGL_IMPEXP )

		/** The base class of 3D objects that can be directly rendered through OpenGL.
		  *  In this class there are a set of common properties to all 3D objects, mainly:
		  *		- A name (m_name): A name that can be optionally asigned to objects for easing its reference.
		  *		- 6D coordinates (x,y,z,yaw,pitch,roll), relative to the "current" reference framework. By default, any object is referenced to global scene coordinates.
		  *		- A RGB color: This field will be used in simple elements (points, lines, text,...) but is ignored in more complex objects that carry their own color information (triangle sets,...)
		  *  See the main class opengl::COpenGLScene
		  *  \sa opengl::COpenGLScene, mrpt::opengl
		  */
		class OPENGL_IMPEXP CRenderizable : public mrpt::utils::CSerializable
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CRenderizable )

			friend class mrpt::opengl::COpenGLViewport;
			friend class mrpt::opengl::CSetOfObjects;

		protected:
			std::string				m_name;
			bool					m_show_name;
			double					m_color_R,m_color_G,m_color_B,m_color_A;    //!< Color components in the range [0,1]
			double					m_x,m_y,m_z;								//!< Translation relative to parent coordinate origin.
			double					m_yaw,m_pitch,m_roll;						//!< Rotation relative to parent coordinate origin, in **DEGREES**.
			float					m_scale_x, m_scale_y, m_scale_z;			//!< Scale components to apply to the object (default=1)
			bool					m_visible; //!< Is the object visible? (default=true)

 		public:
			/** @name Changes the appearance of the object to render
			    @{ */

			void setName(const std::string &n) { m_name=n; }	//!< Changes the name of the object
			std::string getName() const { return m_name; }		//!< Returns the name of the object

			inline bool isVisible() const /** Is the object visible? \sa setVisibility */  { return m_visible; }
			inline void setVisibility(bool visible=true) /** Set object visibility (default=true) \sa isVisible */  { m_visible=visible; }

			void enableShowName(bool showName=true) { m_show_name=showName; }	//!< Enables or disables showing the name of the object as a label when rendering

			CRenderizable& setPose( const mrpt::poses::CPose3D &o );	//!< Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to this)
			CRenderizable& setPose( const mrpt::math::TPose3D &o );	//!< Set the 3D pose from a  mrpt::math::TPose3D object (return a ref to this)
			CRenderizable& setPose( const mrpt::poses::CPoint3D &o );	//!< Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to this)
			CRenderizable& setPose( const mrpt::poses::CPoint2D &o );	//!< Set the 3D pose from a mrpt::poses::CPose3D object (return a ref to this)

			mrpt::math::TPose3D getPose() const;	//!< Returns the 3D pose of the object

			/** Changes the location of the object, keeping untouched the orientation  \return a ref to this */
			inline CRenderizable& setLocation(double x,double y,double z) { m_x=x; m_y=y; m_z=z; return *this; }

			/** Changes the location of the object, keeping untouched the orientation  \return a ref to this  */
			inline CRenderizable& setLocation(const mrpt::math::TPoint3D &p ) { m_x=p.x; m_y=p.y; m_z=p.z; return *this;  }

			double getPoseX() const { return m_x; } //!< Translation relative to parent coordinate origin.
			double getPoseY() const { return m_y; } //!< Translation relative to parent coordinate origin.
			double getPoseZ() const { return m_z; } //!< Translation relative to parent coordinate origin.
			double getPoseYaw() const { return m_yaw; } //!< Rotation relative to parent coordinate origin, in **DEGREES**.
			double getPosePitch() const { return m_pitch; } //!< Rotation relative to parent coordinate origin, in **DEGREES**.
			double getPoseRoll() const { return m_roll; } //!< Rotation relative to parent coordinate origin, in **DEGREES**.

			double getColorR() const { return m_color_R; } //!< Color components in the range [0,1]
			double getColorG() const { return m_color_G; } //!< Color components in the range [0,1]
			double getColorB() const { return m_color_B; } //!< Color components in the range [0,1]
			double getColorA() const { return m_color_A; } //!< Color components in the range [0,1]

			virtual CRenderizable&  setColorR(const double r)	{m_color_R=r; return *this;}	//!<Color components in the range [0,1] \return a ref to this
			virtual CRenderizable&  setColorG(const double g)	{m_color_G=g; return *this;}	//!<Color components in the range [0,1] \return a ref to this
			virtual CRenderizable&  setColorB(const double b)	{m_color_B=b; return *this;}	//!<Color components in the range [0,1] \return a ref to this
			virtual CRenderizable&  setColorA(const double a)	{m_color_A=a; return *this;}	//!<Color components in the range [0,1] \return a ref to this

			inline CRenderizable& setScale(float s)  { m_scale_x=m_scale_y=m_scale_z = s; return *this; } //!< Scale to apply to the object, in all three axes (default=1)  \return a ref to this
			inline CRenderizable& setScale(float sx,float sy,float sz)  { m_scale_x=sx; m_scale_y=sy; m_scale_z = sz; return *this; } //!< Scale to apply to the object in each axis (default=1)  \return a ref to this
			inline float getScaleX() const { return m_scale_x; }  //!< Get the current scaling factor in one axis
			inline float getScaleY() const { return m_scale_y; }  //!< Get the current scaling factor in one axis
			inline float getScaleZ() const { return m_scale_z; }  //!< Get the current scaling factor in one axis


			inline mrpt::utils::TColorf getColor() const { return mrpt::utils::TColorf(m_color_R,m_color_G,m_color_B,m_color_A); }  //!< Returns the object color property as a TColorf
			virtual CRenderizable& setColor( const mrpt::utils::TColorf &c);  //!< Changes the default object color \return a ref to this

			/** Set the color components of this object (R,G,B,Alpha, in the range 0-1)  \return a ref to this */
			virtual CRenderizable& setColor( double R, double G, double B, double A=1);

			/** @} */


			/** Default constructor:  */
			CRenderizable();
			virtual ~CRenderizable() { }

			/** Interface for the stlplus smart pointer class. */
			inline CRenderizable * clone() const
			{
				return static_cast<CRenderizable*>( this->duplicate() );
			}

			/** Implements the rendering of 3D objects in each class derived from CRenderizable.
			  */
			virtual void  render() const = 0;


			/** Simulation of ray-trace, given a pose. Returns true if the ray effectively collisions with the object (returning the distance to the origin of the ray in "dist"), or false in other case. "dist" variable yields undefined behaviour when false is returned
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

			static void	renderTextBitmap( const char *str, void *fontStyle );

		protected:
			/** Checks glGetError and throws an exception if an error situation is found */
			static void checkOpenGLError();
			/** Can be used by derived classes to draw a triangle with a normal vector computed automatically - to be called within a glBegin()-glEnd() block. */
			static void renderTriangleWithNormal( const mrpt::math::TPoint3D &p1,const mrpt::math::TPoint3D &p2,const mrpt::math::TPoint3D &p3 );

			void  writeToStreamRender(utils::CStream &out) const;
			void  readFromStreamRender(utils::CStream &in);

			/** Returns the lowest, free texture name.  */
			static unsigned int getNewTextureNumber();
			static void releaseTextureName(unsigned int i);


			/** Information about the rendering process being issued. \sa See getCurrentRenderingInfo for more details */
			struct OPENGL_IMPEXP TRenderInfo
			{
				int vp_x, vp_y, vp_width, vp_height;    //!< Rendering viewport geometry (in pixels)
				Eigen::Matrix<float,4,4,Eigen::ColMajor>  proj_matrix;  //!< The 4x4 projection matrix
				Eigen::Matrix<float,4,4,Eigen::ColMajor>  model_matrix;  //!< The 4x4 model transformation matrix
				Eigen::Matrix<float,4,4,Eigen::ColMajor>  full_matrix;  //!< PROJ * MODEL

				/** Computes the normalized coordinates (range=[0,1]) on the current rendering viewport of a
				  * point with local coordinates (wrt to the current model matrix) of (x,y,z).
				  *  The output proj_z_depth is the real distance from the eye to the point.
				  */
				void projectPoint(float x,float y,float z, float &proj_x, float &proj_y, float &proj_z_depth) const
				{
					const Eigen::Matrix<float,4,1,Eigen::ColMajor> proj = full_matrix * Eigen::Matrix<float,4,1,Eigen::ColMajor>(x,y,z,1);
					proj_x = proj[2] ? proj[0]/proj[2] : 0;
					proj_y = proj[2] ? proj[1]/proj[2] : 0;
					proj_z_depth = proj[3];
				}

				/** Exactly like projectPoint but the (x,y) projected coordinates are given in pixels instead of normalized coordinates. */
				void projectPointPixels(float x,float y,float z, float &proj_x_px, float &proj_y_px, float &proj_z_depth) const
				{
					projectPoint(x,y,z,proj_x_px,proj_y_px,proj_z_depth);
					proj_x_px*=static_cast<float>(vp_width);
					proj_y_px*=static_cast<float>(vp_height);
				}
			};

			/** Gather useful information on the render parameters.
			  *  It can be called from within the render() method of derived classes, and
			  *   the returned matrices can be used to determine whether a given point (lx,ly,lz)
			  *   in local coordinates wrt the object being rendered falls within the screen or not:
			  * \code
			  *  TRenderInfo ri;
			  *  getCurrentRenderingInfo(ri);
			  *  Eigen::Matrix<float,4,4> M= ri.proj_matrix * ri.model_matrix * HomogeneousMatrix(lx,ly,lz);
			  *  const float rend_x = M(0,3)/M(3,3);
			  *  const float rend_y = M(1,3)/M(3,3);
			  * \endcode
			  *  where (rend_x,rend_y) are both in the range [0,1].
			  */
			void getCurrentRenderingInfo(TRenderInfo &ri) const;

		};
		/**
		  * Applies a CPose3D transformation to the object. Note that this method doesn't <i>set</i> the pose to the given value, but <i>combines</i> it with the existing one.
		  * \sa setPose
		  */
		OPENGL_IMPEXP CRenderizablePtr & operator<<(CRenderizablePtr &r,const mrpt::poses::CPose3D &p);

	} // end namespace

} // End of namespace


#endif
