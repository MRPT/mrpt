/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header


#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h> // round()

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

float  mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL = 0.10f;
size_t mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE            = 1e6;


IMPLEMENTS_SERIALIZABLE( CPointCloud, CRenderizable, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
CPointCloud::CPointCloud( ) :
    m_colorFromDepth(CPointCloud::colNone),
	m_xs(),m_ys(),m_zs(),
	m_pointSize(1),
	m_pointSmooth(false),
	m_last_rendered_count(0),
	m_last_rendered_count_ongoing(0),
	m_min(0),
	m_max(0),
	m_max_m_min(0),
	m_max_m_min_inv(0),
	m_minmax_valid(false),
	m_colorFromDepth_min(0,0,0),
	m_colorFromDepth_max(0,0,1)
{
	markAllPointsAsNew();
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CPointCloud::render() const
{
#if MRPT_HAS_OPENGL_GLUT

    ASSERT_(m_xs.size() == m_ys.size());
    ASSERT_(m_xs.size() == m_zs.size());

	octree_assure_uptodate(); // Rebuild octree if needed
	m_last_rendered_count_ongoing = 0;

	// Info needed by octree renderer:
	gl_utils::TRenderInfo ri;
	gl_utils::getCurrentRenderingInfo(ri);

	if ( m_colorFromDepth )
	{
		if (!m_minmax_valid)
		{
			m_minmax_valid = true;
			if (!m_zs.empty())
				mrpt::math::minimum_maximum(
					m_colorFromDepth == CPointCloud::colZ ? m_zs : (m_colorFromDepth == CPointCloud::colY ? m_ys : m_xs),
					m_min, m_max);
			else m_max=m_min=0;
		}

		m_max_m_min = m_max - m_min;
		if (std::abs(m_max_m_min)<1e-4)
				m_max_m_min=-1;
		else	m_min = m_max - m_max_m_min * 1.01f;
		m_max_m_min_inv  = 1.0/m_max_m_min;
	}

	if ( m_color.A != 255 )
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glDisable(GL_BLEND);
	}



	// Slopes of color interpolation:
	m_col_slop.R = m_colorFromDepth_max.R - m_colorFromDepth_min.R;
	m_col_slop.G  = m_colorFromDepth_max.G - m_colorFromDepth_min.G;
	m_col_slop.B  = m_colorFromDepth_max.B - m_colorFromDepth_min.B;

	m_col_slop_inv.R = m_col_slop.R!=0 ? 1.0f/m_col_slop.R : 0;
	m_col_slop_inv.G = m_col_slop.G!=0 ? 1.0f/m_col_slop.G : 0;
	m_col_slop_inv.B = m_col_slop.B!=0 ? 1.0f/m_col_slop.B : 0;

    glPointSize( m_pointSize );
    if (m_pointSmooth)
			glEnable ( GL_POINT_SMOOTH );
	else 	glDisable( GL_POINT_SMOOTH );


	// Disable lighting for point clouds:
	glDisable(GL_LIGHTING);

    glBegin( GL_POINTS );
    glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A); // The default if m_colorFromDepth=false
	octree_render(ri); // Render all points recursively:
    glEnd();

	glEnable(GL_LIGHTING);

	if ( m_color.A != 255 )
		glDisable(GL_BLEND);

	if (m_pointSmooth)
		glDisable( GL_POINT_SMOOTH );


	m_last_rendered_count = m_last_rendered_count_ongoing;

	checkOpenGLError();
#endif
}

inline void CPointCloud::internal_render_one_point(size_t i) const
{
#if MRPT_HAS_OPENGL_GLUT
	if ( m_colorFromDepth!=colNone && m_max_m_min>0 )
	{
		const float depthCol = (m_colorFromDepth==colX ? m_xs[i] : (m_colorFromDepth==colY ? m_ys[i] : m_zs[i]));

		float	f = (depthCol - m_min) * m_max_m_min_inv;
		f=std::max(0.0f,min(1.0f,f));

		glColor4f(
			m_colorFromDepth_min.R + f*m_col_slop_inv.R,
			m_colorFromDepth_min.G + f*m_col_slop_inv.G,
			m_colorFromDepth_min.B + f*m_col_slop_inv.B,
			m_color.A * (1.0f/255.f) );
	}
	glVertex3f( m_xs[i],m_ys[i],m_zs[i] );
#else
	MRPT_UNUSED_PARAM(i);
#endif
}


/** Render a subset of points (required by octree renderer) */
void  CPointCloud::render_subset(const bool all, const std::vector<size_t>& idxs, const float render_area_sqpixels ) const
{
#if MRPT_HAS_OPENGL_GLUT

	const size_t N = (all ? m_xs.size() : idxs.size());
	const size_t decimation = mrpt::utils::round( std::max(1.0f, static_cast<float>(N / (mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL * render_area_sqpixels)) ) );

	m_last_rendered_count_ongoing += N/decimation;

	if (all)
	{
		for (size_t i=0;i<N;i++)
			internal_render_one_point(i);
	}
	else
	{
		const size_t Np = idxs.size();
		for (size_t i=0;i<Np;i+=decimation)
			internal_render_one_point(idxs[i]);
	}
#else
	MRPT_UNUSED_PARAM(all); MRPT_UNUSED_PARAM(idxs); MRPT_UNUSED_PARAM(render_area_sqpixels);
#endif
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPointCloud::writeToStream(mrpt::utils::CStream &out,int *version) const
{

	if (version)
		*version = 4;
	else
	{
		writeToStreamRender(out);
		// Changed from bool to enum/int32_t in version 3.
		out << static_cast<int32_t>(m_colorFromDepth);
		out << m_xs << m_ys << m_zs;

		// Added in version 1.
		out << m_pointSize;

		// New in version 2:
		out << m_colorFromDepth_min.R << m_colorFromDepth_min.G << m_colorFromDepth_min.B;
		out << m_colorFromDepth_max.R << m_colorFromDepth_max.G << m_colorFromDepth_max.B;

		// New in version 4:
		out << m_pointSmooth;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPointCloud::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		{
			readFromStreamRender(in);
			if (version>=3)
			{
				int32_t axis;
				in >> axis;
				m_colorFromDepth = Axis(axis);
			}
			else
			{
				bool colorFromZ;
				in >> colorFromZ;
				m_colorFromDepth = colorFromZ ? CPointCloud::colZ : CPointCloud::colNone;
			}
			in >> m_xs >> m_ys >> m_zs;

			if (version>=1)
				 in >> m_pointSize;
			else m_pointSize = 1;

			if (version>=2)
			{
				in >> m_colorFromDepth_min.R >> m_colorFromDepth_min.G >> m_colorFromDepth_min.B;
				in  >> m_colorFromDepth_max.R >> m_colorFromDepth_max.G >> m_colorFromDepth_max.B;
			}
			else
			{
				m_colorFromDepth_min = TColorf(0,0,0);
				m_colorFromDepth_max.R = m_color.R*255.f;
				m_colorFromDepth_max.G = m_color.G*255.f;
				m_colorFromDepth_max.B = m_color.B*255.f;
			}

			if (version>=4)
					in >> m_pointSmooth;
			else 	m_pointSmooth = false;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

	markAllPointsAsNew();
}

/*---------------------------------------------------------------
						clear
---------------------------------------------------------------*/
void CPointCloud::clear()
{
	m_xs.clear();
	m_ys.clear();
	m_zs.clear();
	markAllPointsAsNew();
}

/*---------------------------------------------------------------
						insertPoint
---------------------------------------------------------------*/
void CPointCloud::insertPoint( float x,float y, float z )
{
	m_xs.push_back(x);
	m_ys.push_back(y);
	m_zs.push_back(z);

	m_minmax_valid = false;

	// JL: TODO note: Well, this can be clearly done much more efficiently but...I don't have time! :-(
	markAllPointsAsNew();
}

/** Write an individual point (checks for "i" in the valid range only in Debug). */
void CPointCloud::setPoint(size_t i, const float x,const float y, const float z)
{
#ifdef _DEBUG
	ASSERT_BELOW_(i,size())
#endif
	m_xs[i] = x;
	m_ys[i] = y;
	m_zs[i] = z;

	m_minmax_valid = false;

	// JL: TODO note: Well, this can be clearly done much more efficiently but...I don't have time! :-(
	markAllPointsAsNew();
}

/*---------------------------------------------------------------
					setGradientColors
---------------------------------------------------------------*/
void  CPointCloud::setGradientColors( const mrpt::utils::TColorf &colorMin, const mrpt::utils::TColorf &colorMax )
{
	m_colorFromDepth_min = colorMin;
	m_colorFromDepth_max = colorMax;
}

// Do needed internal work if all points are new (octree rebuilt,...)
void CPointCloud::markAllPointsAsNew()
{
	m_minmax_valid = false;
	octree_mark_as_outdated();
}

/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
void CPointCloud::PLY_import_set_vertex_count(const size_t N)
{
	this->resize(N);
}

/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CPointCloud::PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color)
{
	MRPT_UNUSED_PARAM(pt_color);
	this->setPoint(idx,pt.x,pt.y,pt.z);
}

/** In a base class, return the number of vertices */
size_t CPointCloud::PLY_export_get_vertex_count() const
{
	return this->size();
}

/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point.
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CPointCloud::PLY_export_get_vertex(
	const size_t idx,
	mrpt::math::TPoint3Df &pt,
	bool &pt_has_color,
	mrpt::utils::TColorf &pt_color) const
{
	MRPT_UNUSED_PARAM(pt_color);
	pt_has_color=false;

	pt.x = m_xs[idx];
	pt.y = m_ys[idx];
	pt.z = m_zs[idx];
}
