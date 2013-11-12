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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/poses/CPose3D.h>

#include <mrpt/opengl/CMeshFast.h>
#include <mrpt/opengl/CSetOfTriangles.h>

#include <mrpt/utils/color_maps.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CMeshFast, CRenderizableDisplayList, mrpt::opengl )

void CMeshFast::updatePoints() const	{
	CRenderizableDisplayList::notifyChange();

	const size_t cols = Z.getColCount();
	const size_t rows = Z.getRowCount();

	if ((m_colorFromZ)||(m_isImage))
		updateColorsMatrix();

	ASSERT_((cols>0)&&(rows>0))
	ASSERT_((xMax>xMin)&&(yMax>yMin))

	X.setSize(rows,cols);
	Y.setSize(rows,cols);
	const float sCellX=(xMax-xMin)/(rows-1);
	const float sCellY=(yMax-yMin)/(cols-1);

	for (size_t iX=0;iX<rows;iX++)
		for (size_t iY=0;iY<cols;iY++)
		{
			X(iX,iY) = xMin+iX*sCellX;
			Y(iX,iY) = yMin+iY*sCellY;

		}

	pointsUpToDate = true;
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CMeshFast::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT

	if (!pointsUpToDate)
		updatePoints();

    ASSERT_(X.size() == Y.size());
    ASSERT_(X.size() == Z.size());

	if ( m_color.A != 255 )
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

    glPointSize( m_pointSize );

    if (m_pointSmooth)
		glEnable ( GL_POINT_SMOOTH );
	else 	
		glDisable( GL_POINT_SMOOTH );

	// Disable lighting for point clouds:
	glDisable(GL_LIGHTING);

	glBegin( GL_POINTS );
	for (unsigned int i=0; i<X.getRowCount(); i++)
		for (unsigned int j=0; j<X.getColCount(); j++)
		{
			if ( m_isImage && m_textureImage.isColor())
				glColor4f(C_r(i,j), C_g(i,j), C_b(i,j), m_color.A/255.f);
			
			else if (m_isImage)
				glColor4f(C(i,j), C(i,j), C(i,j), m_color.A/255.f);

			else if (m_colorFromZ)
			{
				float rz, gz, bz;
				colormap(m_colorMap, C(i,j), rz, gz, bz);
				glColor4f(rz, gz, bz, m_color.A/255.f);
			}

			else
				glColor4f(m_color.R/255.f, m_color.G/255.f, m_color.B/255.f, m_color.A/255.f);

			glVertex3f(X(i,j), Y(i,j), Z(i,j));
		}

	glEnd();

	glEnable(GL_LIGHTING);

	// Undo flags:
	if ( m_color.A != 255 )
		glDisable(GL_BLEND);

	if (m_pointSmooth)
		glDisable( GL_POINT_SMOOTH );

	checkOpenGLError();
#endif
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void  CMeshFast::assignImage(
	const CImage& img )
{
	MRPT_START

	// Make a copy:
	m_textureImage = img;

	// Delete content in Z
	Z.setSize( img.getHeight(), img.getWidth());
	Z.assign(0);

	//Update flags/states
	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	pointsUpToDate=false;
	

	CRenderizableDisplayList::notifyChange();

	MRPT_END
}

/*---------------------------------------------------------------
							assign Image and Z
  ---------------------------------------------------------------*/
void  CMeshFast::assignImageAndZ( const CImage& img, const mrpt::math::CMatrixTemplateNumeric<float> &in_Z)
{
	MRPT_START

	ASSERT_((img.getWidth() == static_cast<size_t>(in_Z.cols()))&&(img.getHeight() == static_cast<size_t>(in_Z.rows())))

	Z = in_Z;
		
	// Make a copy:
	m_textureImage = img;

	//Update flags/states
	m_modified_Image = true;
	m_enableTransparency = false;
	m_colorFromZ = false;
	m_isImage = true;
	pointsUpToDate = false;
	

	CRenderizableDisplayList::notifyChange();

	MRPT_END
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CMeshFast::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);

		out << m_textureImage;
		out << m_isImage;
		out << xMin << xMax << yMin << yMax;
		out << X << Y << Z;  // We don't need to serialize C, it's computed
		out << m_enableTransparency;
		out << m_colorFromZ;
		out << int16_t(m_colorMap);
		out << m_pointSize; 
		out << m_pointSmooth;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CMeshFast::readFromStream(CStream &in,int version)
{
	switch(version)
	{
		case 0:
			{
			readFromStreamRender(in);

			in >> m_textureImage;
			in >> m_isImage;

			in >> xMin;
			in >> xMax;
			in >> yMin;
			in >> yMax;

			in >> X >> Y >> Z;
			in >> m_enableTransparency;
			in >> m_colorFromZ;

			int16_t	i;
			in >> i;
			m_colorMap =  TColormap(i);
			in >> m_pointSize;
			in >> m_pointSmooth;
			m_modified_Z = true;
			}

			pointsUpToDate = false;
			break;
		
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}


void CMeshFast::updateColorsMatrix() const
{
	if ((!m_modified_Z)&&(!m_modified_Image)) return;

	CRenderizableDisplayList::notifyChange();

	if (m_isImage)
	{
		const size_t cols = m_textureImage.getWidth();
		const size_t rows = m_textureImage.getHeight();
		
		if ((cols != Z.getColCount())||(rows != Z.getRowCount()))
		{
			printf("\nTexture Image and Z sizes have to be equal");

		}
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

		// Color is proportional to difference between height of a cell and
		//  the mean of the nearby cells MEANS:
		C = Z;
		C.normalize(0.01f,0.99f);
	}


	m_modified_Image = false;
	m_modified_Z = false; // Done
	pointsUpToDate = false;
}

void CMeshFast::setZ( const mrpt::math::CMatrixTemplateNumeric<float> &in_Z )
{
	Z = in_Z;
	m_modified_Z = true;
	pointsUpToDate = false;

	//Delete previously loaded images
	m_isImage = false;

	CRenderizableDisplayList::notifyChange();
}


void CMeshFast::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
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
