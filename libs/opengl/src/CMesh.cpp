/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/poses/CPose3D.h>

#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CSetOfTriangles.h>

#include <mrpt/utils/color_maps.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CMesh, CRenderizableDisplayList, mrpt::opengl )

void CMesh::updateTriangles() const	{
	CRenderizableDisplayList::notifyChange();

	float cR[3],cG[3],cB[3];
	const size_t cols=Z.getColCount();
	const size_t rows=Z.getRowCount();
	if (m_colorFromZ) updateColorsMatrix();
	else	{
		cR[0]=cR[1]=cR[2]=m_color.R/255.f;
		cG[0]=cG[1]=cG[2]=m_color.G/255.f;
		cB[0]=cB[1]=cB[2]=m_color.B/255.f;
	}
	ASSERT_(cols>0&&rows>0);
	ASSERT_(xMax>xMin&&yMax>yMin);
	bool useMask=false;
	if (mask.getColCount()!=0&&mask.getRowCount()!=0)	{
		ASSERT_(mask.getColCount()==cols&&mask.getRowCount()==rows);
		useMask=true;
	}
	//const float sCellX=(xMax-xMin)/(cols-1);
	//const float sCellY=(yMax-yMin)/(rows-1);
	const float sCellX=(xMax-xMin)/(rows-1);
	const float sCellY=(yMax-yMin)/(cols-1);
	actualMesh.empty();
	CSetOfTriangles::TTriangle tri;
	for (size_t i=0;i<rows-1;i++) for (size_t j=0;j<cols-1;j++)	{
		if (useMask&&(!mask(i,j)||!mask(i+1,j+1))) continue;
		tri.x[0]=xMin+i*sCellX;
		tri.y[0]=yMin+j*sCellY;
		tri.z[0]=Z(i,j);
		tri.x[2]=tri.x[0]+sCellX;
		tri.y[2]=tri.y[0]+sCellY;
		tri.z[2]=Z(i+1,j+1);
		if (!useMask||mask(i+1,j))	{
			tri.x[1]=tri.x[2];
			tri.y[1]=tri.y[0];
			tri.z[1]=Z(i+1,j);
			if (m_colorFromZ)	{
				colormap(m_colorMap,C(i,j),tri.r[0],tri.g[0],tri.b[0]);
				colormap(m_colorMap,C(i+1,j),tri.r[1],tri.g[1],tri.b[1]);
				colormap(m_colorMap,C(i+1,j+1),tri.r[2],tri.g[2],tri.b[2]);
			}	else	{
				tri.r[0]=tri.r[1]=tri.r[2]=m_color.R/255.f;
				tri.g[0]=tri.g[1]=tri.g[2]=m_color.G/255.f;
				tri.b[0]=tri.b[1]=tri.b[2]=m_color.B/255.f;
			}
			actualMesh.push_back(tri);
		}
		if (!useMask||mask(i,j+1))	{
			tri.x[1]=tri.x[0];
			tri.y[1]=tri.y[2];
			tri.z[1]=Z(i,j+1);
			if (m_colorFromZ)	{
				colormap(m_colorMap,C(i,j),tri.r[0],tri.g[0],tri.b[0]);
				colormap(m_colorMap,C(i,j+1),tri.r[1],tri.g[1],tri.b[1]);
				colormap(m_colorMap,C(i+1,j+1),tri.r[2],tri.g[2],tri.b[2]);
			}	else	{
				tri.r[0]=tri.r[1]=tri.r[2]=m_color.R/255.f;
				tri.g[0]=tri.g[1]=tri.g[2]=m_color.G/255.f;
				tri.b[0]=tri.b[1]=tri.b[2]=m_color.B/255.f;
			}
			actualMesh.push_back(tri);
		}
	}
	trianglesUpToDate=true;
	polygonsUpToDate=false;
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CMesh::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	if (m_enableTransparency)	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	}	else	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);
	if (!trianglesUpToDate) updateTriangles();
	if (!m_isWireFrame) glBegin(GL_TRIANGLES);
	for (size_t i=0;i<actualMesh.size();i++)	{
		const CSetOfTriangles::TTriangle &t=actualMesh[i];
		float ax=t.x[1]-t.x[0];
		float bx=t.x[2]-t.x[0];
		float ay=t.y[1]-t.y[0];
		float by=t.y[2]-t.y[0];
		float az=t.z[1]-t.z[0];
		float bz=t.z[2]-t.z[0];
		glNormal3f(ay*bz-az*by,az*bx-ax*bz,ax*by-ay*bx);
		if (m_isWireFrame) glBegin(GL_LINE_LOOP);
		for (int i=0;i<3;i++)	{
			glColor4f(t.r[i],t.g[i],t.b[i],t.a[i]);
			glVertex3f(t.x[i],t.y[i],t.z[i]);
		}
		if (m_isWireFrame) glEnd();
	}
	if (!m_isWireFrame) glEnd();
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
#endif
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void  CMesh::assignImage(
	const CImage& img )
{
	MRPT_START

	// Make a copy:
	m_textureImage = img;
	m_enableTransparency = false;

	CRenderizableDisplayList::notifyChange();

	MRPT_END
}
/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CMesh::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);

		// Version 0:
		out << m_textureImage;
		out << xMin << xMax << yMin << yMax;
		out << Z << U << V << mask;  // We don't need to serialize C, it's computed
		out << m_enableTransparency;
		out << m_colorFromZ;
		// new in v1
		out << m_isWireFrame;
		out << int16_t(m_colorMap);
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CMesh::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);

			in >> m_textureImage;

			in >> xMin;
			in >> xMax;
			in >> yMin;
			in >> yMax;

			in >> Z >> U >> V >> mask;
			in >> m_enableTransparency;
			in >> m_colorFromZ;

			if (version>=1)
			{
				in >> m_isWireFrame;
				int16_t	i;
				in >> i;
				m_colorMap =  TColormap(i);
			}
			else	m_isWireFrame = false;

			m_modified_Z = true;
		}
		trianglesUpToDate=false;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	trianglesUpToDate=false;
}


void CMesh::updateColorsMatrix() const
{
	if (!m_modified_Z) return;

	CRenderizableDisplayList::notifyChange();

	const size_t cols = Z.getColCount();
	const size_t rows = Z.getRowCount();

	C.setSize(rows,cols);

	// Compute the "smoothed" height matrix:
	// ------------------------------------------
	//CMatrixFloat	MEANS(rows,cols);
	//const int W = 20;
	//for (size_t i=0;i<rows;i++)
	//{
	//	for (size_t j=0;j<cols;j++)
	//	{
	//		int j0 = max(0,int(j)-W);
	//		int j1 = min(int(cols-1),int(j)+W);

	//		int i0 = max(0,int(i)-W);
	//		int i1 = min(int(rows-1),int(i)+W);

	//		double S = 0;
	//		int    N = 0;
	//		for (int ii=i0;ii<=i1;ii++)
	//		{
	//			for (int jj=j0;jj<=j1;jj++)
	//			{
	//				S+=Z(ii,jj);
	//				N++;
	//			}
	//		}

	//		if (N)
	//			MEANS(i,j) = S / N;
	//	}
	//}

	// Color is proportional to difference between height of a cell and
	//  the mean of the nearby cells MEANS:
	C = Z; //- MEANS(i,j);

	// Ignore cells with mask==0
	//for (size_t i=0;i<rows;i++)
	//	for (size_t j=0;j<cols;j++)
	//		if (!mask(i,j))
	//			C(i,j) = 0;

	C.normalize(0.01f,0.99f);

	//SAVE_MATRIX(C);

	m_modified_Z = false; // Done
	trianglesUpToDate=false;
}

void CMesh::setZ( const mrpt::math::CMatrixTemplateNumeric<float> &in_Z )
{
	Z=in_Z;
	m_modified_Z = true;
	trianglesUpToDate=false;
	CRenderizableDisplayList::notifyChange();
}

void CMesh::setMask( const mrpt::math::CMatrixTemplateNumeric<float> &in_mask )
{
	mask = in_mask;
	trianglesUpToDate=false;
	CRenderizableDisplayList::notifyChange();
}

void CMesh::setUV( const mrpt::math::CMatrixTemplateNumeric<float> &in_U, const mrpt::math::CMatrixTemplateNumeric<float> &in_V)
{
	U=in_U;
	V=in_V;
	CRenderizableDisplayList::notifyChange();
}

bool CMesh::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	if (!trianglesUpToDate||!polygonsUpToDate) updatePolygons();
	return mrpt::math::traceRay(tmpPolys,o-this->m_pose,dist);
}

static math::TPolygon3D tmpPoly(3);
mrpt::math::TPolygonWithPlane createPolygonFromTriangle(const CSetOfTriangles::TTriangle &t)	{
	for (size_t i=0;i<3;i++)	{
		tmpPoly[i].x=t.x[i];
		tmpPoly[i].y=t.y[i];
		tmpPoly[i].z=t.z[i];
	}
	return mrpt::math::TPolygonWithPlane(tmpPoly);
}

void CMesh::updatePolygons() const	{
	if (!trianglesUpToDate) updateTriangles();
	size_t N=actualMesh.size();
	tmpPolys.resize(N);
	transform(actualMesh.begin(),actualMesh.end(),tmpPolys.begin(),createPolygonFromTriangle);
	polygonsUpToDate=true;
	CRenderizableDisplayList::notifyChange();
}
