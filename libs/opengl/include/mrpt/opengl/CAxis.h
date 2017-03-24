/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CAxis_H
#define opengl_CAxis_H

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt
{
	namespace opengl
	{


		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CAxis, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** Draw a 3D world axis, with coordinate marks at some regular interval
		  *  \sa opengl::COpenGLScene
		  *  
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *  <tr> <td> mrpt::opengl::CAxis </td> <td> \image html preview_CAxis.png </td> </tr>
		  *  </table>
		  *  </div>
		  *  
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CAxis : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CAxis )
		protected:
			float	m_xmin,m_ymin,m_zmin;
			float	m_xmax,m_ymax,m_zmax;
			float	m_frequency;
			float	m_lineWidth;
			bool	m_marks[3]; //!< draw marks for X,Y,Z
			float	m_textScale;
			float	m_textRot[3][3]; // {x,y,z},{yaw,pitch,roll}

		public:
			void setAxisLimits(float xmin,float ymin, float zmin, float xmax,float ymax, float zmax);
			void setFrequency(float f); //!< Changes the frequency of the "ticks"
			float getFrequency() const;
			void setLineWidth(float w);
			float getLineWidth() const;
			void setTextScale(float f); //!< Changes the size of text labels (default:0.25)
			float getTextScale() const;
			void setTextLabelOrientation(int axis, float yaw_deg, float pitch_deg, float roll_deg); //!< axis: {0,1,2}=>{X,Y,Z}
			void getTextLabelOrientation(int axis, float &yaw_deg, float &pitch_deg, float &roll_deg) const; //!< axis: {0,1,2}=>{X,Y,Z}

			void enableTickMarks(bool v=true);
			void enableTickMarks(bool show_x, bool show_y, bool show_z);

			/** Class factory  */
			static CAxisPtr Create(
				float xmin,float ymin, float zmin,
				float xmax, float ymax,  float zmax,
				float frecuency = 1, float lineWidth = 3, bool marks=false);

			/** Render */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;
	private:
			/** Constructor */
			CAxis(
				float xmin=-1.0f,float ymin=-1.0f, float zmin=-1.0f,
				float xmax=1.0f, float ymax=1.0f,  float zmax=1.0f,
				float frecuency = 0.25f, float lineWidth = 3.0f, bool marks=false);

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CAxis() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CAxis, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace
} // End of namespace

#endif
