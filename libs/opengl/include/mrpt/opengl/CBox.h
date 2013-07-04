/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
		static CBoxPtr Create(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2, bool  is_wireframe = false, float lineWidth = 1.0 )	
		{ 
			return CBoxPtr(new CBox(corner1,corner2,is_wireframe,lineWidth)); 
		}
		
		/** Render
		  * \sa mrpt::opengl::CRenderizable
		  */
		void render_dl() const;

		/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
		virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;
		
		/**
		  * Ray tracing.
		  * \sa mrpt::opengl::CRenderizable
		  */
		virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;
		
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
}
}
#endif
