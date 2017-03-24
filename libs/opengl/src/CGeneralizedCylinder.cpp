/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
#include <mrpt/opengl/CGeneralizedCylinder.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_matrices.h>  // for extract*()
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CGeneralizedCylinder,CRenderizableDisplayList,mrpt::opengl)

CGeneralizedCylinderPtr CGeneralizedCylinder::Create(const std::vector<TPoint3D> &axis,const std::vector<TPoint3D> &generatrix)	
{
	return CGeneralizedCylinderPtr(new CGeneralizedCylinder(axis,generatrix));
}
void CGeneralizedCylinder::TQuadrilateral::calculateNormal()	{
	double ax=points[1].x-points[0].x;
	double ay=points[1].y-points[0].y;
	double az=points[1].z-points[0].z;
	double bx=points[2].x-points[0].x;
	double by=points[2].y-points[0].y;
	double bz=points[2].z-points[0].z;
	normal[0]=az*by-ay*bz;
	normal[1]=ax*bz-az*bx;
	normal[2]=ay*bx-ax*by;
	double s=0;
	for (size_t i=0;i<3;i++) s+=normal[i]*normal[i];
	s=sqrt(s);
	for (size_t i=0;i<3;i++) normal[i]/=s;
}

#if MRPT_HAS_OPENGL_GLUT
class FQuadrilateralRenderer	{
private:
	const mrpt::utils::TColor &color;
public:
	void operator()(const CGeneralizedCylinder::TQuadrilateral &t) const	{
		glNormal3d(t.normal[0],t.normal[1],t.normal[2]);
		for (int i=0;i<4;i++) glVertex3d(t.points[i].x,t.points[i].y,t.points[i].z);
	}
	FQuadrilateralRenderer(const mrpt::utils::TColor &c):color(c)	{}
	~FQuadrilateralRenderer()	{}
};
#endif

void CGeneralizedCylinder::getMeshIterators(const vector<TQuadrilateral> &m,vector<TQuadrilateral>::const_iterator &begin,vector<TQuadrilateral>::const_iterator &end) const	{
	if (fullyVisible)	{
		begin=m.begin();
		end=m.end();
	}	else	{
		size_t qps=m.size()/getNumberOfSections();	//quadrilaterals per section
		begin=m.begin()+qps*firstSection;
		end=m.begin()+qps*lastSection;
	}
}

void CGeneralizedCylinder::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	if (!meshUpToDate) updateMesh();
	checkOpenGLError();
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_QUADS);
	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
	vector<TQuadrilateral>::const_iterator begin,end;
	getMeshIterators(mesh,begin,end);
	for_each(begin,end,FQuadrilateralRenderer(m_color));
	glEnd();
	if (m_color.A!=1.0) glDisable(GL_BLEND);

#endif
}

inline void createMesh(const CMatrixTemplate<TPoint3D> &pointsMesh,size_t R,size_t C,vector<CGeneralizedCylinder::TQuadrilateral> &mesh)	{
	mesh.reserve(R*C);
	for (size_t i=0;i<R;i++) for (size_t j=0;j<C;j++) mesh.push_back(CGeneralizedCylinder::TQuadrilateral(pointsMesh(i,j),pointsMesh(i,j+1),pointsMesh(i+1,j+1),pointsMesh(i+1,j)));
}

/*void transformMesh(const CPose3D &pose,const CMatrixTemplate<TPoint3D> &in,CMatrixTemplate<TPoint3D> &out)	{
	size_t R=in.getRowCount();
	size_t C=in.getColCount();
	out.setSize(R,C);
	for (size_t i=0;i<R;i++) for (size_t j=0;j<C;j++)	{
		TPoint3D pIn=in.get_unsafe(i,j);
		TPoint3D &pOut=out.get_unsafe(i,j);
		pose.composePoint(pIn.x,pIn.y,pIn.z,pOut.x,pOut.y,pOut.z);
	}
}*/

bool CGeneralizedCylinder::traceRay(const CPose3D &o,double &dist) const	{
	if (!meshUpToDate||!polysUpToDate) updatePolys();
	return math::traceRay(polys,o-this->m_pose,dist);
}

void CGeneralizedCylinder::updateMesh() const	{
	CRenderizableDisplayList::notifyChange();

	size_t A=axis.size();
	vector<TPoint3D> genX=generatrix;
	if (closed&&genX.size()>2) genX.push_back(genX[0]);
	size_t G=genX.size();
	mesh.clear();
	if (A>1&&G>1)	{
		pointsMesh=CMatrixTemplate<TPoint3D>(A,G);
		for (size_t i=0;i<A;i++) for (size_t j=0;j<G;j++) axis[i].composePoint(genX[j],pointsMesh.get_unsafe(i,j));
		createMesh(pointsMesh,A-1,G-1,mesh);
	}
	meshUpToDate=true;
	polysUpToDate=false;
}

void CGeneralizedCylinder::writeToStream(mrpt::utils::CStream &out,int *version) const	{
	if (version) *version=1;
	else	{
		writeToStreamRender(out);
		out<<axis<<generatrix;	//In version 0, axis was a vector<TPoint3D>. In version 1, it is a vector<CPose3D>.
	}
}

void CGeneralizedCylinder::readFromStream(mrpt::utils::CStream &in,int version)	{
	switch (version)	{
		case 0:	{
			readFromStreamRender(in);
			vector<TPoint3D> a;
			in>>a>>generatrix;
			generatePoses(a,axis);
			meshUpToDate=false;
			polysUpToDate=false;
			break;
		}	case 1:
			readFromStreamRender(in);
			//version 0
			in>>axis>>generatrix;
			meshUpToDate=false;
			polysUpToDate=false;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}

void generatePolygon(CPolyhedronPtr &poly,const vector<TPoint3D> &profile,const CPose3D &pose)	{
	math::TPolygon3D p(profile.size());
	for (size_t i=0;i<profile.size();i++) pose.composePoint(profile[i].x,profile[i].y,profile[i].z,p[i].x,p[i].y,p[i].z);
	vector<math::TPolygon3D> convexPolys;
	if (!math::splitInConvexComponents(p,convexPolys)) convexPolys.push_back(p);
	poly=CPolyhedron::Create(convexPolys);
}

void CGeneralizedCylinder::getOrigin(CPolyhedronPtr &poly) const	{
	if (!meshUpToDate) updateMesh();
	if (axis.size()<2||generatrix.size()<3) throw std::logic_error("Not enough points.");
	size_t i=fullyVisible?0:firstSection;
	generatePolygon(poly,generatrix,axis[i]);
	poly->setPose(this->m_pose);
	poly->setColor(getColor());
}

void CGeneralizedCylinder::getEnd(CPolyhedronPtr &poly) const	{
	if (!meshUpToDate) updateMesh();
	if (axis.size()<2||generatrix.size()<3) throw std::logic_error("Not enough points.");
	size_t i=(fullyVisible?axis.size():lastSection)-1;
	generatePolygon(poly,generatrix,axis[i]);
	poly->setPose(this->m_pose);
	poly->setColor(getColor());
}

void CGeneralizedCylinder::generateSetOfPolygons(std::vector<TPolygon3D> &res) const	{
	if (!meshUpToDate||!polysUpToDate) updatePolys();
	size_t N=polys.size();
	res.resize(N);
	for (size_t i=0;i<N;i++) res[i]=polys[i].poly;
}

void CGeneralizedCylinder::getClosedSection(size_t index1,size_t index2,mrpt::opengl::CPolyhedronPtr &poly) const	{
	if (index1>index2) swap(index1,index2);
	if (index2>=axis.size()-1) throw std::logic_error("Out of range");
	CMatrixTemplate<TPoint3D> ROIpoints;
	if (!meshUpToDate) updateMesh();
	pointsMesh.extractRows(index1,index2+1,ROIpoints);
	//At this point, ROIpoints contains a matrix of TPoints in which the number of rows equals (index2-index1)+2 and there is a column
	//for each vertex in the generatrix.
	if (!closed)	{
		vector<TPoint3D> vec;
		ROIpoints.extractCol(0,vec);
		ROIpoints.appendCol(vec);
	}
	vector<TPoint3D> vertices;
	ROIpoints.getAsVector(vertices);
	size_t nr=ROIpoints.getRowCount()-1;
	size_t nc=ROIpoints.getColCount()-1;
	vector<vector<uint32_t> > faces;
	faces.reserve(nr*nc+2);
	vector<uint32_t> tmp(4);
	for (size_t i=0;i<nr;i++) for (size_t j=0;j<nc;j++)	{
		size_t base=(nc+1)*i+j;
		tmp[0]=base;
		tmp[1]=base+1;
		tmp[2]=base+nc+2;
		tmp[3]=base+nc+1;
		faces.push_back(tmp);
	}
	tmp.resize(nr+1);
	for (size_t i=0;i<nr+1;i++) tmp[i]=i*(nc+1);
	faces.push_back(tmp);
	for (size_t i=0;i<nr+1;i++) tmp[i]=i*(nc+2)-1;
	poly=CPolyhedron::Create(vertices,faces);
}

void CGeneralizedCylinder::removeVisibleSectionAtStart()	{
	CRenderizableDisplayList::notifyChange();
	if (fullyVisible)	{
		if (!getNumberOfSections()) throw std::logic_error("No more sections");
		fullyVisible=false;
		firstSection=1;
		lastSection=getNumberOfSections();
	}	else if (firstSection>=lastSection) throw std::logic_error("No more sections");
	else firstSection++;
}
void CGeneralizedCylinder::removeVisibleSectionAtEnd()	{
	CRenderizableDisplayList::notifyChange();
	if (fullyVisible)	{
		if (!getNumberOfSections()) throw std::logic_error("No more sections");
		fullyVisible=false;
		firstSection=0;
		lastSection=getNumberOfSections()-1;
	}	else if (firstSection>=lastSection) throw std::logic_error("No more sections");
	else lastSection--;
}

void CGeneralizedCylinder::updatePolys() const	{
	CRenderizableDisplayList::notifyChange();

	if (!meshUpToDate) updateMesh();
	size_t N=mesh.size();
	polys.resize(N);
	TPolygon3D tmp(4);
	for (size_t i=0;i<N;i++)	{
		for (size_t j=0;j<4;j++) tmp[j]=mesh[i].points[j];
		polys[i]=tmp;
	}
	polysUpToDate=true;
}

void CGeneralizedCylinder::generatePoses(const vector<TPoint3D> &pIn,mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t &pOut)	{
	size_t N=pIn.size();
	if (N==0)	{
		pOut.resize(0);
		return;
	}
	vector<double> yaws;
	yaws.reserve(N);
	vector<TPoint3D>::const_iterator it1=pIn.begin(),it2;
	for (;;) if ((it2=it1+1)==pIn.end()) break;
	else	{
		yaws.push_back(atan2(it2->y-it1->y,it2->x-it1->x));
		it1=it2;
	}
	yaws.push_back(*yaws.rbegin());
	pOut.resize(N);
	for (size_t i=0;i<N;i++)	{
		const TPoint3D &p=pIn[i];
		pOut[i]=CPose3D(p.x,p.y,p.z,yaws[i],0,0);
	}
}

bool CGeneralizedCylinder::getFirstSectionPose(CPose3D &p)	{
	if (axis.size()<=0) return false;
	p=axis[0];
	return true;
}

bool CGeneralizedCylinder::getLastSectionPose(CPose3D &p)	{
	if (axis.size()<=0) return false;
	p=*axis.rbegin();
	return true;
}

bool CGeneralizedCylinder::getFirstVisibleSectionPose(CPose3D &p)	{
	if (fullyVisible) return getFirstSectionPose(p);
	if (getVisibleSections()<=0) return false;
	p=axis[firstSection];
	return true;
}

bool CGeneralizedCylinder::getLastVisibleSectionPose(CPose3D &p)	{
	if (fullyVisible) return getLastSectionPose(p);
	if (getVisibleSections()<=0) return false;
	p=axis[lastSection];
	return true;
}

void CGeneralizedCylinder::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = TPoint3D(0,0,0);
	bb_max = TPoint3D(0,0,0);

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
