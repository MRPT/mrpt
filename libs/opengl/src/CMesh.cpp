/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CMesh, CRenderizableDisplayList, mrpt::opengl )

CMesh::CMesh(bool enableTransparency, float xMin, float xMax, float yMin, float yMax) :
	m_textureImage(0, 0),
	m_enableTransparency(enableTransparency),
	m_colorFromZ(false),
	m_isWireFrame(false),
	m_isImage(false),
	Z(0, 0), mask(0, 0), U(0, 0), V(0, 0), C(0, 0), C_r(0, 0), C_g(0, 0), C_b(0, 0),
	m_colorMap(mrpt::utils::cmHOT),
	m_modified_Z(true),
	m_modified_Image(false),
	xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax),
	trianglesUpToDate(false)
{
	m_color.A = 255;
	m_color.R = 0;
	m_color.G = 0;
	m_color.B = 150;
}

CMesh::~CMesh()
{
}


CMeshPtr CMesh::Create(bool enableTransparency, float xMin, float xMax , float yMin , float yMax  )
{
	return CMeshPtr( new CMesh( enableTransparency, xMin ,xMax , yMin ,yMax ) );
}
void CMesh::updateTriangles() const	{
	CRenderizableDisplayList::notifyChange();

// Remember:
//mutable std::vector<std::pair<CSetOfTriangles::TTriangle,TTriangleVertexIndices> > actualMesh;	//!< List of triangles in the mesh
//mutable std::vector<std::pair<mrpt::math::TPoint3D,size_t> > vertex_normals; //!< The accumulated normals & counts for each vertex, so normals can be averaged.

	const size_t cols=Z.getColCount();
	const size_t rows=Z.getRowCount();

	actualMesh.clear();
	if (cols == 0 && rows == 0)
		return; // empty mesh

	ASSERT_(cols>0 && rows>0)
	ASSERT_(xMax>xMin&&yMax>yMin)

	// we have 1 more row & col of vertices than of triangles:
	vertex_normals.assign((1+cols)*(1+rows), std::pair<TPoint3D,size_t>(TPoint3D(0,0,0),0) );

	float cR[3], cG[3], cB[3], cA[3];
	cA[0] = cA[1] = cA[2] = m_color.A / 255.f;

	if ((m_colorFromZ) || (m_isImage)) {
		updateColorsMatrix();
	} else {
		cR[0]=cR[1]=cR[2]=m_color.R/255.f;
		cG[0]=cG[1]=cG[2]=m_color.G/255.f;
		cB[0]=cB[1]=cB[2]=m_color.B/255.f;
	}


	bool useMask=false;
	if (mask.getColCount()!=0&&mask.getRowCount()!=0)	{
		ASSERT_(mask.getColCount()==cols&&mask.getRowCount()==rows);
		useMask=true;
	}
	const float sCellX=(xMax-xMin)/(rows-1);
	const float sCellY=(yMax-yMin)/(cols-1);

	CSetOfTriangles::TTriangle tri;
	for (size_t iX=0;iX<rows-1;iX++) for (size_t iY=0;iY<cols-1;iY++)	{
		if (useMask&&(!mask(iX,iY)||!mask(iX+1,iY+1))) continue;
		tri.x[0]=xMin+iX*sCellX;
		tri.y[0]=yMin+iY*sCellY;
		tri.z[0]=Z(iX,iY);
		tri.x[2]=tri.x[0]+sCellX;
		tri.y[2]=tri.y[0]+sCellY;
		tri.z[2]=Z(iX+1,iY+1);

		// Vertex indices:
		TTriangleVertexIndices tvi;
		tvi.vind[0] = iX+rows*iY;
		tvi.vind[2] = (iX+1)+rows*(iY+1);

		// Each quadrangle has up to 2 triangles:
		//  [0]
		//   |
		//   |
		//  [1]--[2]
		// Order: 0,1,2
		if (!useMask||mask(iX+1,iY))
		{
			tri.x[1]=tri.x[2];
			tri.y[1]=tri.y[0];
			tri.z[1]=Z(iX+1,iY);
			for (int i=0;i<3;i++) tri.a[i] = cA[i];  // Assign alpha channel

			if (m_colorFromZ)	{
				colormap(m_colorMap,C(iX,iY),tri.r[0],tri.g[0],tri.b[0]);
				colormap(m_colorMap,C(iX+1,iY),tri.r[1],tri.g[1],tri.b[1]);
				colormap(m_colorMap,C(iX+1,iY+1),tri.r[2],tri.g[2],tri.b[2]);
			}
			else if (m_isImage)	{
				if (m_textureImage.isColor())
				{
					tri.r[0]=tri.r[1]=tri.r[2]=C_r(iX,iY);
					tri.g[0]=tri.g[1]=tri.g[2]=C_g(iX,iY);
					tri.b[0]=tri.b[1]=tri.b[2]=C_b(iX,iY);
				}
				else
				{
					tri.r[0]=tri.r[1]=tri.r[2]=C(iX,iY);
					tri.g[0]=tri.g[1]=tri.g[2]=C(iX,iY);
					tri.b[0]=tri.b[1]=tri.b[2]=C(iX,iY);
				}
			}
			else {
				tri.r[0]=tri.r[1]=tri.r[2]=m_color.R/255.f;
				tri.g[0]=tri.g[1]=tri.g[2]=m_color.G/255.f;
				tri.b[0]=tri.b[1]=tri.b[2]=m_color.B/255.f;
			}

			// Compute normal of this triangle, and add it up to the 3 neighboring vertices:
			// A = P1 - P0, B = P2 - P0
			float ax=tri.x[1]-tri.x[0];
			float bx=tri.x[2]-tri.x[0];
			float ay=tri.y[1]-tri.y[0];
			float by=tri.y[2]-tri.y[0];
			float az=tri.z[1]-tri.z[0];
			float bz=tri.z[2]-tri.z[0];
			const TPoint3D this_normal(ay*bz-az*by,az*bx-ax*bz,ax*by-ay*bx);

			// Vertex indices:
			tvi.vind[1] = iX+1+rows*iY;

			// Add triangle:
			actualMesh.push_back( std::pair<CSetOfTriangles::TTriangle,TTriangleVertexIndices>(tri,tvi) );

			// For averaging normals:
			for (int k=0;k<3;k++) {
				vertex_normals[ tvi.vind[k] ].first  += this_normal;
				vertex_normals[ tvi.vind[k] ].second ++;
			}
		}
		// 2:
		//  [0]--[1->2]
		//     \  |
		//       \|
		//       [2->1]
		// Order: 0,2,1
		if (!useMask||mask(iX,iY+1))
		{
			tri.x[1]=tri.x[2];
			tri.y[1]=tri.y[2];
			tri.z[1]=tri.z[2];

			tri.x[2]=tri.x[0];
			//tri.y[2]=tri.y[1];
			tri.z[2]=Z(iX,iY+1);
			if (m_colorFromZ)	{
				colormap(m_colorMap,C(iX,iY),tri.r[0],tri.g[0],tri.b[0]);
				colormap(m_colorMap,C(iX+1,iY+1),tri.r[1],tri.g[1],tri.b[1]);
				colormap(m_colorMap,C(iX,iY+1),tri.r[2],tri.g[2],tri.b[2]);
			}
			else if (m_isImage)	{
				if (m_textureImage.isColor())
				{
					tri.r[0]=tri.r[1]=tri.r[2]=C_r(iX,iY);
					tri.g[0]=tri.g[1]=tri.g[2]=C_g(iX,iY);
					tri.b[0]=tri.b[1]=tri.b[2]=C_b(iX,iY);
				}
				else
				{
					tri.r[0]=tri.r[1]=tri.r[2]=C(iX,iY);
					tri.g[0]=tri.g[1]=tri.g[2]=C(iX,iY);
					tri.b[0]=tri.b[1]=tri.b[2]=C(iX,iY);
				}
			}
			else {
				tri.r[0]=tri.r[1]=tri.r[2]=m_color.R/255.f;
				tri.g[0]=tri.g[1]=tri.g[2]=m_color.G/255.f;
				tri.b[0]=tri.b[1]=tri.b[2]=m_color.B/255.f;
			}

			// Compute normal of this triangle, and add it up to the 3 neighboring vertices:
			// A = P1 - P0, B = P2 - P0
			float ax=tri.x[1]-tri.x[0];
			float bx=tri.x[2]-tri.x[0];
			float ay=tri.y[1]-tri.y[0];
			float by=tri.y[2]-tri.y[0];
			float az=tri.z[1]-tri.z[0];
			float bz=tri.z[2]-tri.z[0];
			const TPoint3D this_normal(ay*bz-az*by,az*bx-ax*bz,ax*by-ay*bx);

			// Vertex indices:
			tvi.vind[1] = tvi.vind[2];
			tvi.vind[2] = iX+rows*(iY+1);

			// Add triangle:
			actualMesh.push_back( std::pair<CSetOfTriangles::TTriangle,TTriangleVertexIndices>(tri,tvi) );

			// For averaging normals:
			for (int k=0;k<3;k++) {
				vertex_normals[ tvi.vind[k] ].first  += this_normal;
				vertex_normals[ tvi.vind[k] ].second ++;
			}
		}
	}

	// Average normals:
	for (size_t i=0;i<vertex_normals.size();i++)
	{
		const size_t N = vertex_normals[i].second;
		if (N>0) vertex_normals[i].first *= 1.0/N;
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
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	}	else	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}
	glEnable(GL_NORMALIZE);  // So the GPU normalizes the normals instead of doing it in the CPU
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);
	if (!trianglesUpToDate) updateTriangles();
	if (!m_isWireFrame) glBegin(GL_TRIANGLES);
	for (size_t i=0;i<actualMesh.size();i++)	{
		const CSetOfTriangles::TTriangle &t=actualMesh[i].first;
		const TTriangleVertexIndices &tvi=actualMesh[i].second;

		if (m_isWireFrame) {
			glDisable(GL_LIGHTING);  // Disable lights when drawing lines
			glBegin(GL_LINE_LOOP);
		}
		for (int i=0;i<3;i++)	{
			const mrpt::math::TPoint3D &n = vertex_normals[tvi.vind[i]].first;
			glNormal3f(n.x,n.y,n.z);
			glColor4f(t.r[i],t.g[i],t.b[i],t.a[i]);
			glVertex3f(t.x[i],t.y[i],t.z[i]);
		}
		if (m_isWireFrame)
		{
			glEnd();
			glEnable(GL_LIGHTING);
		}
	}
	if (!m_isWireFrame) glEnd();
	glDisable(GL_BLEND);
	glDisable(GL_NORMALIZE);
	glEnable(GL_DEPTH_TEST);
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

	// Delete content in Z
	Z.setSize( img.getHeight(), img.getWidth());
	Z.assign(0);


	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	trianglesUpToDate=false;


	CRenderizableDisplayList::notifyChange();

	MRPT_END
}

/*---------------------------------------------------------------
							assign Image and Z
  ---------------------------------------------------------------*/
void  CMesh::assignImageAndZ( const CImage& img, const mrpt::math::CMatrixTemplateNumeric<float> &in_Z)
{
	MRPT_START

	ASSERT_((img.getWidth() == static_cast<size_t>(in_Z.cols()))&&(img.getHeight() == static_cast<size_t>(in_Z.rows())))

	Z = in_Z;

	// Make a copy:
	m_textureImage = img;


	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	trianglesUpToDate = false;


	CRenderizableDisplayList::notifyChange();

	MRPT_END
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CMesh::writeToStream(mrpt::utils::CStream &out,int *version) const
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
void  CMesh::readFromStream(mrpt::utils::CStream &in,int version)
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
	CRenderizableDisplayList::notifyChange();
}


void CMesh::updateColorsMatrix() const
{	
	if ((!m_modified_Z)&&(!m_modified_Image)) return;

	CRenderizableDisplayList::notifyChange();

	if (m_isImage)
	{	
		const size_t cols = m_textureImage.getWidth();
		const size_t rows = m_textureImage.getHeight();

		if ((cols != Z.getColCount())||(rows != Z.getRowCount()))
			printf("\nTexture Image and Z sizes have to be equal");

		else if (m_textureImage.isColor())
		{
			C_r.setSize(rows, cols);
			C_g.setSize(rows, cols);
			C_b.setSize(rows, cols);
			m_textureImage.getAsRGBMatrices(C_r, C_g, C_b);
		}
		else
		{
			C.setSize(rows, cols);
			m_textureImage.getAsMatrix(C);
		}
	}
	else
	{
		const size_t cols = Z.getColCount();
		const size_t rows = Z.getRowCount();
		C.setSize(rows,cols);

		//Color is proportional to height:
		C = Z;

		//If mask is empty -> Normalize the whole mesh
		if (mask.empty())
			C.normalize(0.01f,0.99f);

		//Else -> Normalize color ignoring masked-out cells:
		else
		{
			float val_max = -std::numeric_limits<float>::max(), val_min =  std::numeric_limits<float>::max();
			bool any_valid =false;

			for (size_t c=0;c<cols;c++) 
				for (size_t r=0;r<rows;r++)
				{
					if (!mask(r,c)) continue;
					any_valid = true;
					const float val = C(r,c);
					mrpt::utils::keep_max(val_max,val);
					mrpt::utils::keep_min(val_min,val);
				}

			if (any_valid)
			{
				float minMaxDelta = val_max - val_min;
				if (minMaxDelta==0) minMaxDelta = 1;
				const float minMaxDelta_ = 1.0f/minMaxDelta;
				C.array() = (C.array()-val_min)*minMaxDelta_;
			}
		}
	}

	m_modified_Image = false;
	m_modified_Z = false;
	trianglesUpToDate=false;
}

void CMesh::setZ( const mrpt::math::CMatrixTemplateNumeric<float> &in_Z )
{
	Z=in_Z;
	m_modified_Z = true;
	trianglesUpToDate=false;

	//Delete previously loaded images
	m_isImage = false;

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
mrpt::math::TPolygonWithPlane createPolygonFromTriangle(const std::pair<CSetOfTriangles::TTriangle,CMesh::TTriangleVertexIndices> &p)	{
	const CSetOfTriangles::TTriangle &t = p.first;
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


void CMesh::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = xMin;
	bb_min.y = yMin;
	bb_min.z = Z.minCoeff();

	bb_max.x = xMax;
	bb_max.y = yMax;
	bb_max.z = Z.maxCoeff();

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
