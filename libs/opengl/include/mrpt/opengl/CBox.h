/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CBox_H
#define opengl_CBox_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt	{
namespace opengl	{

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CBox,CRenderizableDisplayList, OPENGL_IMPEXP)
	
	/** A solid or wireframe box in 3D, defined by 6 rectangular faces parallel to the planes X, Y and Z (note that the object can be translated and rotated afterwards as any other CRenderizable object using the "object pose" in the base class).
	  *  Three drawing modes are possible:
	  *	- Wireframe: setWireframe(true). Used color is the CRenderizable color
	  *	- Solid box: setWireframe(false). Used color is the CRenderizable color
	  *	- Solid box with border: setWireframe(false) + enableBoxBorder(true). Solid color is the CRenderizable color, border line can be set with setBoxBorderColor().
	  *  
	  * \sa opengl::COpenGLScene,opengl::CRenderizable
	  *  
	  *  <div align="center">
	  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
	  *   <tr> <td> mrpt::opengl::CBox </td> <td> \image html preview_CBox.png </td> </tr>
	  *  </table>
	  *  </div>
	  *  
	  * \ingroup mrpt_opengl_grp
	  */
	class OPENGL_IMPEXP CBox :public CRenderizableDisplayList	{
		DEFINE_SERIALIZABLE(CBox)

	protected:
		mrpt::math::TPoint3D  	m_corner_min,m_corner_max;		//!< Corners coordinates
		bool 					m_wireframe;	//!< true: wireframe, false: solid
		float					m_lineWidth; 	//!< For wireframe only.
		bool			m_draw_border;		//!< Draw line borders to solid box with the given linewidth (default: true)
		mrpt::utils::TColor     m_solidborder_color;    //!< Color of the solid box borders.
		
	public:
		/** Constructor returning a smart pointer to the newly created object. */
		static CBoxPtr Create(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2, bool  is_wireframe = false, float lineWidth = 1.0 );
		
		/** Render
		  * \sa mrpt::opengl::CRenderizable
		  */
		void render_dl() const MRPT_OVERRIDE;

		/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
		void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;
		
		/**
		  * Ray tracing.
		  * \sa mrpt::opengl::CRenderizable
		  */
		bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;
		
		inline void setLineWidth(float width) { m_lineWidth = width; CRenderizableDisplayList::notifyChange(); }
		inline float getLineWidth() const { return m_lineWidth; }
		
		inline void setWireframe(bool is_wireframe=true) { m_wireframe = is_wireframe; CRenderizableDisplayList::notifyChange(); }
		inline bool isWireframe() const { return m_wireframe; }

		inline void enableBoxBorder(bool drawBorder=true) {m_draw_border = drawBorder; CRenderizableDisplayList::notifyChange(); }
		inline bool isBoxBorderEnabled() const { return m_draw_border; }

		inline void setBoxBorderColor(const mrpt::utils::TColor &c) { m_solidborder_color=c; CRenderizableDisplayList::notifyChange(); }
		inline mrpt::utils::TColor getBoxBorderColor() const { return m_solidborder_color; }

		/** Set the position and size of the box, from two corners in 3D */
		void setBoxCorners(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2);
		void getBoxCorners(mrpt::math::TPoint3D &corner1, mrpt::math::TPoint3D &corner2) const { corner1= m_corner_min; corner2 = m_corner_max; }
		

	private:
		/** Basic empty constructor. Set all parameters to default. */ 
		CBox();
		
		/** Constructor with all the parameters  */ 
		CBox(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2, bool  is_wireframe = false, float lineWidth = 1.0);
		
		/** Destructor  */ 
		virtual ~CBox() { }
		
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CBox,CRenderizableDisplayList, OPENGL_IMPEXP)
}
}
#endif
