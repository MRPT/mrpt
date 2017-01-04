/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header


#include <mrpt/opengl/CSetOfTriangles.h>
#include "opengl_internals.h"
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CSetOfTriangles, CRenderizableDisplayList, mrpt::opengl )


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CSetOfTriangles::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT

	if (m_enableTransparency)
	{
		//glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
    else
    {
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
    }

	vector<TTriangle>::const_iterator	it;

	glEnable(GL_NORMALIZE); // Normalize normals
	glBegin(GL_TRIANGLES);

	for (it=m_triangles.begin();it!=m_triangles.end();++it)
	{
        // Compute the normal vector:
        // ---------------------------------
        float	ax= it->x[1] - it->x[0];
        float	ay= it->y[1] - it->y[0];
        float	az= it->z[1] - it->z[0];

        float	bx= it->x[2] - it->x[0];
        float	by= it->y[2] - it->y[0];
        float	bz= it->z[2] - it->z[0];

        glNormal3f(ay*bz-az*by,-ax*bz+az*bx,ax*by-ay*bx);

		glColor4f( it->r[0],it->g[0],it->b[0],it->a[0] );
		glVertex3f(it->x[0],it->y[0],it->z[0]);

		glColor4f( it->r[1],it->g[1],it->b[1],it->a[1] );
		glVertex3f(it->x[1],it->y[1],it->z[1]);

		glColor4f( it->r[2],it->g[2],it->b[2],it->a[2] );
		glVertex3f(it->x[2],it->y[2],it->z[2]);
	}

	glEnd();
	glDisable(GL_NORMALIZE);

	glDisable(GL_BLEND);
#endif
}

static void triangle_writeToStream(mrpt::utils::CStream &o, const CSetOfTriangles::TTriangle &t)
{
	o.WriteBufferFixEndianness(t.x,3);
	o.WriteBufferFixEndianness(t.y,3);
	o.WriteBufferFixEndianness(t.z,3);

	o.WriteBufferFixEndianness(t.r,3);
	o.WriteBufferFixEndianness(t.g,3);
	o.WriteBufferFixEndianness(t.b,3);
	o.WriteBufferFixEndianness(t.a,3);
}
static void triangle_readFromStream(mrpt::utils::CStream &i, CSetOfTriangles::TTriangle &t)
{
	i.ReadBufferFixEndianness(t.x,3);
	i.ReadBufferFixEndianness(t.y,3);
	i.ReadBufferFixEndianness(t.z,3);

	i.ReadBufferFixEndianness(t.r,3);
	i.ReadBufferFixEndianness(t.g,3);
	i.ReadBufferFixEndianness(t.b,3);
	i.ReadBufferFixEndianness(t.a,3);
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfTriangles::writeToStream(mrpt::utils::CStream &out,int *version) const
{

	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		uint32_t	n = (uint32_t)m_triangles.size();
		out << n;
		for (size_t i=0;i<n;i++)
			triangle_writeToStream(out,m_triangles[i]);

		// Version 1:
		out << m_enableTransparency;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfTriangles::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			uint32_t	n;
			in >> n;
			m_triangles.assign(n,TTriangle());
			for (size_t i=0;i<n;i++)
				triangle_readFromStream(in,m_triangles[i]);

			if (version>=1)
					in >> m_enableTransparency;
			else	m_enableTransparency = true;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	polygonsUpToDate=false;
	CRenderizableDisplayList::notifyChange();
}

bool CSetOfTriangles::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	if (!polygonsUpToDate) updatePolygons();
	return mrpt::math::traceRay(tmpPolygons,o-this->m_pose,dist);
}

//Helper function. Given two 2D points (y1,z1) and (y2,z2), returns three coefficients A, B and C so that both points
//verify Ay+Bz+C=0
//returns true if the coefficients have actually been calculated
/*
inline bool lineCoefs(const float &y1,const float &z1,const float &y2,const float &z2,float coefs[3])	{
	if ((y1==y2)&(z1==z2)) return false;	//Both points are the same
	if (y1==y2)	{
		//Equation is y-y1=0
		coefs[0]=1;
		coefs[1]=0;
		coefs[2]=-y1;
		return true;
	}	else	{
		//Equation is:
		// z1 - z2        /z2 - z1       \ .
		// -------y + z + |-------y1 - z1| = 0
		// y2 - y1        \y2 - y1       /
		coefs[0]=(z1-z2)/(y2-y1);
		coefs[1]=1;
		coefs[2]=((z2-z1)/(y2-y1))*y1-z1;
		return true;
	}
}
*/
/*
bool CSetOfTriangles::traceRayTriangle(const mrpt::poses::CPose3D &transf,double &dist,const float xb[3],const float yb[3],const float zb[3])	{
	//Computation of the actual coordinates in the beam's system.
	float x[3];
	float y[3];
	float z[3];
	for (int i=0;i<3;i++) transf.composePoint(xb[i],yb[i],zb[i],x[i],y[i],z[i]);

	//If the triangle is parallel to the beam, no collision is posible.
	//The triangle is parallel to the beam if the projection of the triangle to the YZ plane results in a line.
	float lCoefs[3];
	if (!lineCoefs(y[0],z[0],y[1],z[1],lCoefs)) return false;
	else if (lCoefs[0]*y[2]+lCoefs[1]*z[2]+lCoefs[2]==0) return false;
	//Basic sign check
	if (x[0]<0&&x[1]<0&&x[2]<0) return false;
	if (y[0]<0&&y[1]<0&&y[2]<0) return false;
	if (z[0]<0&&z[1]<0&&z[2]<0) return false;
	if (y[0]>0&&y[1]>0&&y[2]>0) return false;
	if (z[0]>0&&z[1]>0&&z[2]>0) return false;
	//Let M be the following matrix:
	//  /p1\ /x1 y1 z1\ .
	//M=|p2|=|x2 y2 z2|
	//  \p3/ \x3 y3 z3/
	//If M has rank 3, then (p1,p2,p3) conform a plane which does not contain the origin (0,0,0).
	//If M has rank 2, then (p1,p2,p3) may conform either a line or a plane which contains the origin.
	//If M has a lesser rank, then (p1,p2,p3) do not conform a plane.
	//Let N be the following matrix:
	//N=/p2-p1\ = /x1 y1 z1\ .
	//  \p3-p1/   \x3 y3 z3/
	//Given that the rank of M is 2, if the rank of N is still 2 then (p1,p2,p3) conform a plane; either, a line.
	float mat[9];
	for (int i=0;i<3;i++)	{
		mat[3*i]=x[i];
		mat[3*i+1]=y[i];
		mat[3*i+2]=z[i];
	}
	CMatrixTemplateNumeric<float> M=CMatrixTemplateNumeric<float>(3,3,mat);
	float d2=0;
	float mat2[6];
	switch (M.rank())	{
		case 3:
			//M's rank is 3, so the triangle is inside a plane which doesn't pass through (0,0,0).
			//This plane's equation is Ax+By+Cz+1=0. Since the point we're searching for verifies y=0 and z=0, we
			//only need to compute A (x=-1/A). We do this using Cramer's method.
			for (int i=0;i<9;i+=3) mat[i]=1;
			d2=(CMatrixTemplateNumeric<float>(3,3,mat)).det();
			if (d2==0) return false;
			else dist=M.det()/d2;
			break;
		case 2:
			//if N's rank is 2, the triangle is inside a plane containing (0,0,0).
			//Otherwise, (p1,p2,p3) don't conform a plane.
			for (int i=0;i<2;i++)	{
				mat2[3*i]=x[i+1]-x[0];
				mat2[3*i+1]=y[i+1]-y[0];
				mat2[3*i+2]=z[i+1]-z[0];
			}
			if (CMatrixTemplateNumeric<float>(2,3,mat2).rank()==2) dist=0;
			else return false;
			break;
		default:
			return false;
	}
	if (dist<0) return false;
	//We've already determined the collision point between the beam and the plane, but we need to check if this
	//point is actually inside the triangle. We do this by projecting the scene into a <x=constant> plane, so
	//that the triangle is defined by three 2D lines and the beam is the point (y,z)=(0,0).

	//For each pair of points, we compute the line that they conform, and then we check if the other point's
	//sign in that line's equation equals that of the origin (that is, both points are on the same side of the line).
	//If this holds for each one of the three possible combinations, then the point is inside the triangle.
	//Furthermore, if any of the three equations verify f(0,0)=0, then the point is in the verge of the line, which
	//is considered as being inside. Note, whichever is the case, that f(0,0)=lCoefs[2].

	//lineCoefs already contains the coefficients for the first line.
	if (lCoefs[2]==0) return true;
	else if (((lCoefs[0]*y[2]+lCoefs[1]*z[2]+lCoefs[2])>0)!=(lCoefs[2]>0)) return false;
	lineCoefs(y[0],z[0],y[2],z[2],lCoefs);
	if (lCoefs[2]==0) return true;
	else if (((lCoefs[0]*y[1]+lCoefs[1]*z[1]+lCoefs[2])>0)!=(lCoefs[2]>0)) return false;
	lineCoefs(y[1],z[1],y[2],z[2],lCoefs);
	if (lCoefs[2]==0) return true;
	else return ((lCoefs[0]*y[0]+lCoefs[1]*z[0]+lCoefs[2])>0)==(lCoefs[2]>0);
}
*/

CRenderizable& CSetOfTriangles::setColor_u8(const mrpt::utils::TColor &c)	{
	CRenderizableDisplayList::notifyChange();
	m_color=c;
	mrpt::utils::TColorf col(c);
	for (std::vector<TTriangle>::iterator it=m_triangles.begin();it!=m_triangles.end();++it) for (size_t i=0;i<3;i++)	{
		it->r[i]=col.R;
		it->g[i]=col.G;
		it->b[i]=col.B;
		it->a[i]=col.A;
	}
	return *this;
}

CRenderizable& CSetOfTriangles::setColorR_u8(const uint8_t r)	{
	CRenderizableDisplayList::notifyChange();
	m_color.R=r;
	const float col = r/255.f;
	for (std::vector<TTriangle>::iterator it=m_triangles.begin();it!=m_triangles.end();++it) for (size_t i=0;i<3;i++) it->r[i]=col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorG_u8(const uint8_t g)	{
	CRenderizableDisplayList::notifyChange();
	m_color.G=g;
	const float col = g/255.f;
	for (std::vector<TTriangle>::iterator it=m_triangles.begin();it!=m_triangles.end();++it) for (size_t i=0;i<3;i++) it->g[i]=col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorB_u8(const uint8_t b)	{
	CRenderizableDisplayList::notifyChange();
	m_color.B=b;
	const float col = b/255.f;
	for (std::vector<TTriangle>::iterator it=m_triangles.begin();it!=m_triangles.end();++it) for (size_t i=0;i<3;i++) it->b[i]=col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorA_u8(const uint8_t a)	{
	CRenderizableDisplayList::notifyChange();
	m_color.A=a;
	const float col = a/255.f;
	for (std::vector<TTriangle>::iterator it=m_triangles.begin();it!=m_triangles.end();++it) for (size_t i=0;i<3;i++) it->a[i]=col;
	return *this;
}

void CSetOfTriangles::getPolygons(std::vector<mrpt::math::TPolygon3D> &polys) const	{
	if (!polygonsUpToDate) updatePolygons();
	size_t N=tmpPolygons.size();
	for (size_t i=0;i<N;i++) polys[i]=tmpPolygons[i].poly;
}

void CSetOfTriangles::updatePolygons() const	{
	TPolygon3D tmp(3);
	size_t N=m_triangles.size();
	tmpPolygons.resize(N);
	for (size_t i=0;i<N;i++) for (size_t j=0;j<3;j++)	{
		const TTriangle &t=m_triangles[i];
		tmp[j].x=t.x[j];
		tmp[j].y=t.y[j];
		tmp[j].z=t.z[j];
		tmpPolygons[i]=tmp;
	}
	polygonsUpToDate=true;
	CRenderizableDisplayList::notifyChange();
}

void CSetOfTriangles::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = mrpt::math::TPoint3D(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	for (size_t i=0;i<m_triangles.size();i++)
	{
		const TTriangle &t=m_triangles[i];

		keep_min(bb_min.x, t.x[0]);  keep_max(bb_max.x, t.x[0]);
		keep_min(bb_min.y, t.y[0]);  keep_max(bb_max.y, t.y[0]);
		keep_min(bb_min.z, t.z[0]);  keep_max(bb_max.z, t.z[0]);

		keep_min(bb_min.x, t.x[1]);  keep_max(bb_max.x, t.x[1]);
		keep_min(bb_min.y, t.y[1]);  keep_max(bb_max.y, t.y[1]);
		keep_min(bb_min.z, t.z[1]);  keep_max(bb_max.z, t.z[1]);

		keep_min(bb_min.x, t.x[2]);  keep_max(bb_max.x, t.x[2]);
		keep_min(bb_min.y, t.y[2]);  keep_max(bb_max.y, t.y[2]);
		keep_min(bb_min.z, t.z[2]);  keep_max(bb_max.z, t.z[2]);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CSetOfTriangles::insertTriangles(const CSetOfTrianglesPtr &p)	{
	reserve(m_triangles.size()+p->m_triangles.size());
	m_triangles.insert(m_triangles.end(),p->m_triangles.begin(),p->m_triangles.end());
	polygonsUpToDate=false;
	CRenderizableDisplayList::notifyChange();
}
