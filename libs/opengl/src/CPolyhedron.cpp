/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/math/CMatrixTemplate.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h> // dotProduct()
#include <mrpt/random.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPolyhedron,CRenderizableDisplayList,mrpt::opengl)

//Auxiliary data and code
template<class T> class FCreatePolygonFromFace	{
public:
	const vector<TPoint3D> &verts;
	FCreatePolygonFromFace(const vector<TPoint3D> &v):verts(v)	{}
	TPolygon3D p;
	T operator()(const CPolyhedron::TPolyhedronFace &f)	{
		p=TPolygon3D(f.vertices.size());
		for (size_t i=0;i<f.vertices.size();i++) p[i]=verts[f.vertices[i]];
		return p;
	}
};

bool getVerticesAndFaces(const vector<math::TPolygon3D> &polys,vector<TPoint3D> &vertices,vector<CPolyhedron::TPolyhedronFace> &faces)	{
	vertices.reserve(4*polys.size());
	faces.reserve(polys.size());
	for (std::vector<math::TPolygon3D>::const_iterator it=polys.begin();it!=polys.end();++it)	{
		size_t N=it->size();
		if (N<3) return false;
		CPolyhedron::TPolyhedronFace f;
		f.vertices.resize(N);
		for (size_t i=0;i<N;i++)	{
			vector<TPoint3D>::iterator it2=find(vertices.begin(),vertices.end(),(*it)[i]);
			if (it2==vertices.end())	{
				f.vertices[i]=vertices.size();
				vertices.push_back((*it)[i]);
			}	else f.vertices[i]=it2-vertices.begin();
		}
		faces.push_back(f);
	}
	return true;
}

enum JohnsonBodyPart	{
	INF_NO_BODY=-2,SUP_NO_BODY=-1,UPWARDS_PYRAMID=0,DOWNWARDS_PYRAMID=1,UPWARDS_CUPOLA=2,DOWNWARDS_CUPOLA=3,ROTATED_UPWARDS_CUPOLA=4,ROTATED_DOWNWARDS_CUPOLA=5,PRISM=6,ANTIPRISM=7,UPWARDS_ROTUNDA=8,DOWNWARDS_ROTUNDA=9,ROTATED_UPWARDS_ROTUNDA=10,ROTATED_DOWNWARDS_ROTUNDA=11
};
bool analyzeJohnsonPartsString(const std::string &components,uint32_t numBaseEdges,vector<JohnsonBodyPart> &parts)	{
	size_t N=components.length();
	size_t i=0;
	bool rot=false;
	while (i<N)	{
		switch (components[i])	{
			case 'A':case 'a':
				if (parts.size()==0) parts.push_back(INF_NO_BODY);
				parts.push_back(ANTIPRISM);
				break;
			case 'C':case 'c':
				if (numBaseEdges&1) return false;
				if (i==N-1) return false;
				i++;
				if (components[i]=='+')	{
					if (i!=N-1) return false;
					if (parts.size()==0) parts.push_back(INF_NO_BODY);
					parts.push_back(rot?ROTATED_UPWARDS_CUPOLA:UPWARDS_CUPOLA);
					rot=false;
				}	else if (components[i]=='-')	{
					if (parts.size()>0) return false;
					parts.push_back(rot?ROTATED_DOWNWARDS_CUPOLA:DOWNWARDS_CUPOLA);
					rot=false;
				}	else return false;
				break;
			case 'R':case 'r':
				if (numBaseEdges!=10) return false;
				if (i==N-1) return false;
				i++;
				if (components[i]=='+')	{
					if (i!=N-1) return false;
					if (parts.size()==0) parts.push_back(INF_NO_BODY);
					parts.push_back(rot?ROTATED_UPWARDS_ROTUNDA:UPWARDS_ROTUNDA);
					rot=false;
				}	else if (components[i]=='-')	{
					if (parts.size()>0) return false;
					parts.push_back(rot?ROTATED_DOWNWARDS_ROTUNDA:DOWNWARDS_ROTUNDA);
					rot=false;
				}	else return false;
				break;
			case 'G':case 'g':
				if (i==N-1) return false;
				i++;
				if (components[i]=='C'||components[i]=='R')	{
					rot=true;
					continue;
				}	else return false;
			case 'P':case 'p':
				if (i==N-1) return false;
				i++;
				switch (components[i])	{
					case '+':
						if (numBaseEdges>5) return false;
						if (i!=N-1) return false;
						if (parts.size()==0) parts.push_back(INF_NO_BODY);
						parts.push_back(UPWARDS_PYRAMID);
						break;
					case '-':
						if (numBaseEdges>5) return false;
						if (i!=1) return false;
						parts.push_back(DOWNWARDS_PYRAMID);
						break;
					case 'R':case 'r':
						if (parts.size()>0&&(*parts.rbegin()==PRISM)) return false;
						if (parts.size()==0) parts.push_back(INF_NO_BODY);
						parts.push_back(PRISM);
						break;
					default:
						return false;
				}
				break;
			default:
				return false;
		}
		i++;
	}
	if (parts.size()==0) return false;
	JohnsonBodyPart p=*parts.rbegin();
	if (p!=UPWARDS_PYRAMID&&p!=UPWARDS_CUPOLA&&p!=ROTATED_UPWARDS_CUPOLA&&p!=UPWARDS_ROTUNDA&&p!=ROTATED_UPWARDS_ROTUNDA) parts.push_back(SUP_NO_BODY);
	return true;
}
inline size_t additionalVertices(JohnsonBodyPart j,uint32_t numBaseEdges)	{
	if (j==INF_NO_BODY||j==SUP_NO_BODY) return 0;
	else if (j<UPWARDS_CUPOLA) return 1;	//j is a pyramid
	else if (j<PRISM) return numBaseEdges>>1;	//j is a cupola
	else if (j<UPWARDS_ROTUNDA) return numBaseEdges;	//j is a prism or antiprism
	else return 10;	//j is a rotunda
}
void insertCupola(size_t numBaseEdges,double angleShift,double baseRadius,double edgeLength,bool isRotated,bool isUpwards,size_t base,vector<TPoint3D> &verts,vector<CPolyhedron::TPolyhedronFace> &faces)	{
	size_t edges2=numBaseEdges>>1;
	double minorRadius=baseRadius*sin(M_PI/numBaseEdges)/sin(M_PI/edges2);
	//"Proper base"'s apothem=base radius*cos(2*pi/(2*numBaseEdges))
	//"Small base"'s apothem=small radius*cos(2*pi/2*(numBaseEdges/2))
	//Cupola's height is so that (Da^2+Height^2=Edge length^2, where Da is the difference between both apothems.
	double h=sqrt(square(edgeLength)-square(baseRadius*cos(M_PI/numBaseEdges)-minorRadius*cos(M_PI/edges2)));
	double height=verts[base].z+(isUpwards?h:-h);
	angleShift+=M_PI/edges2+(isRotated?-M_PI/numBaseEdges:M_PI/numBaseEdges);
	size_t minorBase=verts.size();
	for (size_t i=0;i<edges2;i++)	{
		double ang=angleShift+2*M_PI*i/edges2;
		verts.push_back(TPoint3D(minorRadius*cos(ang),minorRadius*sin(ang),height));
	}
	CPolyhedron::TPolyhedronFace tri,quad,cBase;
	tri.vertices.resize(3);
	quad.vertices.resize(4);
	cBase.vertices.resize(edges2);
	size_t iq=isRotated?1:2,it=0;
	for (size_t i=0;i<edges2;i++)	{
		cBase.vertices[i]=it+minorBase;
		size_t iiq=(iq+1)%numBaseEdges+base;
		size_t iiiq=(iiq+1)%numBaseEdges+base;
		size_t iit=(it+1)%edges2+minorBase;
		quad.vertices[0]=it+minorBase;
		quad.vertices[1]=iit;
		quad.vertices[2]=iiq;
		quad.vertices[3]=iq+base;
		tri.vertices[0]=iit;
		tri.vertices[1]=iiq;
		tri.vertices[2]=iiiq;
		iq=(iq+2)%numBaseEdges;
		it=(it+1)%edges2;
		faces.push_back(tri);
		faces.push_back(quad);
	}
	if (edges2>=3) faces.push_back(cBase);
}
void insertRotunda(double angleShift,double baseRadius,bool isRotated,bool isUpwards,size_t base,vector<TPoint3D> &verts,vector<CPolyhedron::TPolyhedronFace> &faces)	{
	double R1=baseRadius*sqrt((5.0-sqrt(5.0))/10.0);
	double R2=baseRadius*sqrt((5.0+sqrt(5.0))/10.0);
	double baseHeight=verts[base].z;
	TPoint3D p1[5],p2[5];
	angleShift+=M_PI/10;
	if (isRotated) angleShift+=M_PI/5;
	for (size_t i=0;i<5;i++)	{
		double a=(i+i+1)*M_PI/5+angleShift;
		double b=(i+i)*M_PI/5+angleShift;
		double ca=cos(a),sa=sin(a),cb=cos(b),sb=sin(b);
		p1[i].x=R1*ca;
		p1[i].y=R1*sa;
		p1[i].z=baseHeight+(isUpwards?R2:-R2);
		p2[i].x=R2*cb;
		p2[i].y=R2*sb;
		p2[i].z=baseHeight+(isUpwards?R1:-R1);
	}
	size_t newBase=verts.size();
	for (size_t i=0;i<5;i++) verts.push_back(p1[i]);
	for (size_t i=0;i<5;i++) verts.push_back(p2[i]);
	CPolyhedron::TPolyhedronFace f,g;
	f.vertices.resize(3);
	g.vertices.resize(5);
	size_t baseStart=isRotated?2:1;
	for (size_t i=0;i<5;i++)	{
		size_t ii=(i+1)%5;
		f.vertices[0]=newBase+i;
		f.vertices[1]=newBase+ii;
		f.vertices[2]=newBase+ii+5;
		faces.push_back(f);
		f.vertices[0]=newBase+i+5;
		f.vertices[1]=((i+i+baseStart)%10)+base;
		f.vertices[2]=((i+i+9+baseStart)%10)+base;
		faces.push_back(f);
		g.vertices[0]=newBase+(ii%5)+5;
		g.vertices[1]=newBase+i;
		g.vertices[2]=newBase+i+5;
		g.vertices[3]=(i+i+baseStart)%10+base;
		g.vertices[4]=(i+i+baseStart+1)%10+base;
		faces.push_back(g);
	}
	for (size_t i=0;i<5;i++) g.vertices[i]=i+newBase;
	faces.push_back(g);
	return;
}
inline size_t additionalFaces(JohnsonBodyPart j,uint32_t numBaseEdges)	{
	if (j==INF_NO_BODY||j==SUP_NO_BODY) return 1;	//j is a base
	else if (j<UPWARDS_CUPOLA) return numBaseEdges;	//j is a pyramid
	else if (j<PRISM) return numBaseEdges+ ((numBaseEdges>=6)?1:0);	//j is a cupola
	else if (j==PRISM) return numBaseEdges;
	else if (j==ANTIPRISM) return numBaseEdges<<1;
	else return 16;	//j is a rotunda
}

inline bool faceContainsEdge(const CPolyhedron::TPolyhedronFace &f,const CPolyhedron::TPolyhedronEdge &e)	{
	char hm=0;
	for (vector<uint32_t>::const_iterator it=f.vertices.begin();it!=f.vertices.end();++it) if (*it==e.v1||*it==e.v2) hm++;
	return hm==2;
}

bool getPlanesIntersection(const vector<const TPlane *> &planes,TPoint3D &pnt)	{
	if (planes.size()<3) return false;
	char o=0;
	TPlane pl=*planes[0];
	TLine3D l;
	TObject3D obj;
	for (size_t i=1;i<planes.size();i++) switch (o)	{
		case 0:
			if (!intersect(pl,*planes[i],obj)) return false;
			if (obj.getPlane(pl)) o=0;
			else if (obj.getLine(l)) o=1;
			else if (obj.getPoint(pnt)) o=2;
			else return false;
			break;
		case 1:
			if (!intersect(l,*planes[i],obj)) return false;
			if (obj.getLine(l)) o=1;
			else if (obj.getPoint(pnt)) o=2;
			else return false;
			break;
		case 2:
			if (!planes[i]->contains(pnt)) return false;
			break;
		default:
			return false;
	}
	return o==2;
}

bool searchForFace(const vector<CPolyhedron::TPolyhedronFace> &fs,uint32_t v1,uint32_t v2,uint32_t v3)	{
	for (vector<CPolyhedron::TPolyhedronFace>::const_iterator it=fs.begin();it!=fs.end();++it)	{
		const vector<uint32_t> &f=it->vertices;
		size_t hmf=0;
		for (vector<uint32_t>::const_iterator it2=f.begin();it2!=f.end();++it2)	{
			if (*it2==v1) hmf|=1;
			else if (*it2==v2) hmf|=2;
			else if (*it2==v3) hmf|=4;
		}
		if (hmf==7) return true;
	}
	return false;
}
bool searchForEdge(const vector<CPolyhedron::TPolyhedronEdge> &es,uint32_t v1,uint32_t v2,size_t &where)	{
	for (where=0;where<es.size();where++)	{
		const CPolyhedron::TPolyhedronEdge &e=es[where];
		if (e.v1==v1&&e.v2==v2) return true;
		else if (e.v1==v2&&e.v2==v1) return false;
	}
	throw std::logic_error("Internal error. Edge not found");
}
bool searchForEdge(const vector<CPolyhedron::TPolyhedronFace>::const_iterator &begin,const vector<CPolyhedron::TPolyhedronFace>::const_iterator &end,uint32_t v1,uint32_t v2)	{
	for (vector<CPolyhedron::TPolyhedronFace>::const_iterator it=begin;it!=end;++it)	{
		const vector<uint32_t> &f=it->vertices;
		char res=0;
		for (vector<uint32_t>::const_iterator it2=f.begin();it2!=f.end();++it2) if (*it2==v1) res|=1;
		else if (*it2==v2) res|=2;
		if (res==3) return true;
	}
	return false;
}
double getHeight(const TPolygon3D &p,const TPoint3D &c)	{
	size_t N=p.size();
	if (N>5||N<3) throw std::logic_error("Faces must have exactly 3, 4 or 5 vertices.");
	double r=mrpt::math::distance(p[0],c);
	double l=mrpt::math::distance(p[0],p[1]);
	for (size_t i=1;i<N;i++) if (abs(mrpt::math::distance(p[i],c)-r)>=mrpt::math::geometryEpsilon) throw std::logic_error("There is a non-regular polygon.");
	else if (abs(mrpt::math::distance(p[i],p[(i+1)%N])-l)>=mrpt::math::geometryEpsilon) throw std::logic_error("There is a non-regular polygon.");
	return sqrt(square(l)-square(r));
}
struct SegmentVector	{
	double d[3];
	double mod;
	inline double &operator[](size_t i) {
		return d[i];
	}
	inline double operator[](size_t i) const	{
		return d[i];
	}
};
//End of auxiliary data and code

double CPolyhedron::TPolyhedronEdge::length(const vector<TPoint3D> &vertices) const	{
	return mrpt::math::distance(vertices[v1],vertices[v2]);
}

double CPolyhedron::TPolyhedronFace::area(const vector<TPoint3D> &vs) const	{
	//Calculate as fan of triangles.
	size_t N=vertices.size();
	vector<SegmentVector> d(N-1);
	for (size_t i=1;i<N;i++)	{
		d[i-1].mod=0;
		for (size_t j=0;j<3;j++)	{
			d[i-1][j]=vs[vertices[i]][j]-vs[vertices[0]][j];
			d[i-1].mod+=square(d[i-1][j]);
		}
		d[i-1].mod=sqrt(d[i-1].mod);
	}
	double res=0;
	for (size_t i=1;i<N-1;i++) res+=sqrt(square(d[i-1].mod*d[i].mod)-square(dotProduct<3,double>(d[i-1],d[i])));
	return res/2;
}

void CPolyhedron::TPolyhedronFace::getCenter(const vector<TPoint3D> &vrts,TPoint3D &p) const	{
	p.x=p.y=p.z=0.0;
	for (vector<uint32_t>::const_iterator it=vertices.begin();it!=vertices.end();++it)	{
		p.x+=vrts[*it].x;
		p.y+=vrts[*it].y;
		p.z+=vrts[*it].z;
	}
	size_t N=vertices.size();
	p.x/=N;
	p.y/=N;
	p.z/=N;
}

CPolyhedronPtr CPolyhedron::Create(const std::vector<math::TPolygon3D> &polys)	{
	vector<TPoint3D> vertices(0);
	vector<TPolyhedronFace> faces;
	if (getVerticesAndFaces(polys,vertices,faces)) return Create(vertices,faces);
	else return CreateEmpty();
}

CPolyhedronPtr CPolyhedron::CreateCubicPrism(double x1,double x2,double y1,double y2,double z1,double z2)	{
	vector<TPoint3D> verts;
	vector<TPolyhedronFace> faces;
	for (int i=0;i<8;i++) verts.push_back(TPoint3D((i&1)?x2:x1,(i&2)?y2:y1,(i&4)?z2:z1));
	static uint32_t faceVertices[]={0,1,5,4, 2,3,7,6, 0,2,6,4, 1,3,7,5, 0,1,3,2, 4,5,7,6};
	TPolyhedronFace f;
	for (uint32_t *p=reinterpret_cast<uint32_t *>(&faceVertices);p<24+reinterpret_cast<uint32_t *>(&faceVertices);p+=4)	{
		f.vertices.insert(f.vertices.begin(),p,p+4);
		faces.push_back(f);
		f.vertices.clear();
	}
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreatePyramid(const vector<TPoint2D> &baseVertices,double height)	{
	uint32_t n=baseVertices.size();
	if (baseVertices.size()<3) throw std::logic_error("Not enought vertices");
	vector<TPoint3D> verts;
	vector<TPolyhedronFace> faces;
	verts.push_back(TPoint3D(0,0,height));
	for (vector<TPoint2D>::const_iterator it=baseVertices.begin();it!=baseVertices.end();++it) verts.push_back(TPoint3D(it->x,it->y,0));
	TPolyhedronFace f,g;
	f.vertices.push_back(0);
	f.vertices.push_back(n);
	f.vertices.push_back(1);
	g.vertices.push_back(1);
	faces.push_back(f);
	for (uint32_t i=2;i<=n;i++)	{
		f.vertices.erase(f.vertices.begin()+1);
		f.vertices.push_back(i);
		faces.push_back(f);
		g.vertices.push_back(i);
	}
	faces.push_back(g);
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateDoublePyramid(const vector<TPoint2D> &baseVertices,double height1,double height2)	{
	uint32_t N=baseVertices.size();
	if (N<3) throw std::logic_error("Not enought vertices");
	vector<TPoint3D> verts;
	verts.reserve(N+2);
	vector<TPolyhedronFace> faces;
	faces.reserve(N<<1);
	verts.push_back(TPoint3D(0,0,height1));
	for (vector<TPoint2D>::const_iterator it=baseVertices.begin();it!=baseVertices.end();++it) verts.push_back(TPoint3D(it->x,it->y,0));
	verts.push_back(TPoint3D(0,0,-height2));
	TPolyhedronFace f,g;
	f.vertices.resize(3);
	g.vertices.resize(3);
	f.vertices[0]=0;
	g.vertices[0]=N+1;
	for (uint32_t i=1;i<N;i++)	{
		f.vertices[1]=g.vertices[1]=i;
		f.vertices[2]=g.vertices[2]=i+1;
		faces.push_back(f);
		faces.push_back(g);
	}
	f.vertices[1]=g.vertices[1]=1;
	faces.push_back(f);
	faces.push_back(g);
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateTruncatedPyramid(const vector<TPoint2D> &baseVertices,double height,double ratio)	{
	uint32_t n=baseVertices.size();
	if (n<3) throw std::logic_error("Not enough vertices");
	vector<TPoint3D> verts(n+n);
	vector<TPolyhedronFace> faces(n+2);
	TPolyhedronFace f,g,h;
	f.vertices.resize(4);
	g.vertices.resize(n);
	h.vertices.resize(n);
	for (uint32_t i=0;i<n;i++)	{
		verts[i]=TPoint3D(baseVertices[i].x,baseVertices[i].y,0);
		verts[i+n]=TPoint3D(baseVertices[i].x*ratio,baseVertices[i].y*ratio,height);
		uint32_t ii=(i+1)%n;
		f.vertices[0]=i;
		f.vertices[1]=ii;
		f.vertices[2]=ii+n;
		f.vertices[3]=i+n;
		faces[i]=f;
		g.vertices[i]=i;
		h.vertices[i]=i+n;
	}
	faces[n]=g;
	faces[n+1]=h;
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateCustomAntiprism(const vector<TPoint2D> &bottomBase,const vector<TPoint2D> &topBase,const double height)	{
	uint32_t n=bottomBase.size();
	if (n<3) throw std::logic_error("Not enough vertices");
	if (n!=topBase.size()) throw std::logic_error("Bases' number of vertices do not match");
	vector<TPoint3D> verts(n+n);
	vector<TPolyhedronFace> faces(n+n+2);
	TPolyhedronFace f,g,h;
	f.vertices.resize(3);
	g.vertices.resize(n);
	h.vertices.resize(n);
	for (uint32_t i=0;i<n;i++)	{
		verts[i]=TPoint3D(bottomBase[i].x,bottomBase[i].y,0);
		verts[n+i]=TPoint3D(topBase[i].x,topBase[i].y,height);
		uint32_t ii=(i+1)%n;
		f.vertices[0]=i;
		f.vertices[1]=ii;
		f.vertices[2]=i+n;
		faces[i]=f;
		f.vertices[0]=i+n;
		f.vertices[1]=ii+n;
		f.vertices[2]=ii;
		faces[n+i]=f;
		g.vertices[i]=i;
		h.vertices[i]=n+i;
	}
	faces[n+n]=g;
	faces[n+n+1]=h;
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateParallelepiped(const TPoint3D &base,const TPoint3D &v1,const TPoint3D &v2,const TPoint3D &v3)	{
	vector<TPoint3D> verts(8);
	vector<TPolyhedronFace> faces(6);
	for (uint32_t i=0;i<8;i++)	{
		verts[i]=base;
		if (i&1) verts[i]=verts[i]+v1;
		if (i&2) verts[i]=verts[i]+v2;
		if (i&4) verts[i]=verts[i]+v3;
	}
	TPolyhedronFace f;
	f.vertices.resize(4);
	f.vertices[0]=0;
	f.vertices[1]=1;
	f.vertices[2]=3;
	f.vertices[3]=2;
	faces[0]=f;
	//f.vertices[0]=0;
	//f.vertices[1]=1;
	f.vertices[2]=5;
	f.vertices[3]=4;
	faces[1]=f;
	//f.vertices[0]=0;
	f.vertices[1]=2;
	f.vertices[2]=6;
	//f.vertices[3]=4;
	faces[2]=f;
	for (uint32_t i=0;i<3;i++)	{
		uint32_t valueAdd=4>>i;
		faces[i+3].vertices.resize(4);
		for (uint32_t j=0;j<4;j++) faces[i+3].vertices[j]=faces[i].vertices[j]+valueAdd;
	}
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateBifrustum(const vector<TPoint2D> &baseVertices,double height1,double ratio1,double height2,double ratio2)	{
	//TODO: check special case in which ratio=0.
	size_t N=baseVertices.size();
	vector<TPoint3D> verts(3*N);
	size_t N2=N+N;
	for (size_t i=0;i<N;i++)	{
		double x=baseVertices[i].x;
		double y=baseVertices[i].y;
		verts[i].x=x;
		verts[i].y=y;
		verts[i].z=0;
		verts[i+N].x=x*ratio1;
		verts[i+N].y=y*ratio1;
		verts[i+N].z=height1;
		verts[i+N2].x=x*ratio2;
		verts[i+N2].y=y*ratio2;
		verts[i+N2].z=-height2;	//This is not an error. This way, two positive heights produce an actual bifrustum.
	}
	vector<TPolyhedronFace> faces(N2+2);
	TPolyhedronFace f,g,h;
	f.vertices.resize(4);
	g.vertices.resize(N);
	h.vertices.resize(N);
	for (size_t i=0;i<N;i++)	{
		size_t i2=(i+1)%N;
		f.vertices[0]=i;
		f.vertices[1]=i2;
		f.vertices[2]=i2+N;
		f.vertices[3]=i+N;
		faces[i]=f;
		f.vertices[2]=i2+N2;
		f.vertices[3]=i+N2;
		faces[i+N]=f;
		g.vertices[i]=i+N;
		h.vertices[i]=i+N2;
	}
	faces[N2]=g;
	faces[N2+1]=h;
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateTrapezohedron(uint32_t numBaseEdges,double baseRadius,double basesDistance)	{
	if (numBaseEdges<3) throw std::logic_error("Not enough vertices");
	if (basesDistance==0||baseRadius==0) return CreateEmpty();
	size_t numBaseEdges2=numBaseEdges<<1;
	vector<TPoint3D> verts(numBaseEdges2+2);
	double space=2*M_PI/numBaseEdges;
	double shift=space/2;
	double height1=basesDistance/2;
	double cospii=cos(M_PI/numBaseEdges);
	double height2=height1*(cospii+1)/(1-cospii);	//Apex height, calculated so each face conforms a plane
	for (size_t i=0;i<numBaseEdges;i++)	{
		double ang=space*i;
		double ang2=ang+shift;
		size_t ii=i+numBaseEdges;
		verts[i].x=baseRadius*cos(ang);
		verts[i].y=baseRadius*sin(ang);
		verts[i].z=-height1;
		verts[ii].x=baseRadius*cos(ang2);
		verts[ii].y=baseRadius*sin(ang2);
		verts[ii].z=height1;
	}
	verts[numBaseEdges2].x=0;
	verts[numBaseEdges2].y=0;
	verts[numBaseEdges2].z=-height2;
	verts[numBaseEdges2+1].x=0;
	verts[numBaseEdges2+1].y=0;
	verts[numBaseEdges2+1].z=height2;
	vector<TPolyhedronFace> faces(numBaseEdges2);
	TPolyhedronFace f,g;
	f.vertices.resize(4);
	g.vertices.resize(4);
	f.vertices[3]=numBaseEdges2;
	g.vertices[3]=numBaseEdges2+1;
	for (size_t i=0;i<numBaseEdges;i++)	{
		size_t ii=(i+1)%numBaseEdges;
		size_t i2=i+numBaseEdges;
		f.vertices[0]=i;
		f.vertices[1]=i2;
		f.vertices[2]=ii;
		g.vertices[0]=i2;
		g.vertices[1]=ii;
		g.vertices[2]=ii+numBaseEdges;
		faces[i]=f;
		faces[i+numBaseEdges]=g;
	}
	return CreateNoCheck(verts,faces);
}

CPolyhedronPtr CPolyhedron::CreateJohnsonSolidWithConstantBase(uint32_t numBaseEdges,double baseRadius,const std::string &components,size_t shifts)	{
	if (baseRadius==0) return CreateEmpty();
	if (numBaseEdges<3) throw std::logic_error("Not enough vertices");
	vector<JohnsonBodyPart> parts;
	if (!analyzeJohnsonPartsString(components,numBaseEdges,parts)) throw std::logic_error("Invalid string");
	//Some common values are computed
	size_t nParts=parts.size();
	double edgeLength=2*baseRadius*sin(M_PI/numBaseEdges);
	double antiPrismHeight=sqrt(square(edgeLength)-square(baseRadius)*(2-2*cos(M_PI/numBaseEdges)));
	//Vertices' and faces' vectors are computed
	size_t nVerts=numBaseEdges*(nParts-1)+additionalVertices(parts[0],numBaseEdges)+additionalVertices(*parts.rbegin(),numBaseEdges);
	size_t nFaces=0;
	for (size_t i=0;i<nParts;i++) nFaces+=additionalFaces(parts[i],numBaseEdges);
	vector<TPoint3D> verts;
	verts.reserve(nVerts);
	vector<TPolyhedronFace> faces;
	faces.reserve(nFaces);
	//Each base's position is computed. Also, the height is set so that the polyhedron is vertically centered in z=0.
	double h,mHeight=0;
	vector<pair<double,size_t> > basePositionInfo(nParts-1);
	for (size_t i=0;i<nParts-1;i++)	{
		if (parts[i]==PRISM) h=edgeLength;
		else if (parts[i]==ANTIPRISM)	{
			h=antiPrismHeight;
			shifts++;
		}	else h=0;
		basePositionInfo[i]=make_pair(mHeight+=h,shifts);
	}
	mHeight/=2;
	double semi=M_PI/numBaseEdges;
	//All the bases are generated and inserted into the vertices' vector.
	for (vector<pair<double,size_t> >::const_iterator it=basePositionInfo.begin();it!=basePositionInfo.end();++it) generateShiftedBase(numBaseEdges,baseRadius,it->first-mHeight,semi*it->second,verts);
	size_t initialBase=0,endBase=0;
	TPolyhedronFace face;
	face.vertices.reserve(numBaseEdges);
	//Each body is inserted.
	for (size_t i=0;i<nParts;i++)	{
		switch (parts[i])	{
			case INF_NO_BODY:
				//Inferior base.
				face.vertices.resize(numBaseEdges);
				for (size_t i=0;i<numBaseEdges;i++) face.vertices[i]=endBase+i;
				faces.push_back(face);
			break;
			case SUP_NO_BODY:
				//Superior base.
				face.vertices.resize(numBaseEdges);
				for (size_t i=0;i<numBaseEdges;i++) face.vertices[i]=initialBase+i;
				faces.push_back(face);
			break;
			case UPWARDS_PYRAMID:	{
				//Upwards-pointing pyramid. There must be 5 or less vertices.
				double apexHeight=baseRadius*sqrt(4*square(sin(M_PI/numBaseEdges))-1);
				face.vertices.resize(3);
				face.vertices[0]=verts.size();
				face.vertices[1]=initialBase+numBaseEdges-1;
				face.vertices[2]=initialBase;
				do	{
					faces.push_back(face);
					face.vertices[1]=face.vertices[2];
					face.vertices[2]++;
				}	while (face.vertices[2]<initialBase+numBaseEdges);
				verts.push_back(TPoint3D(0,0,verts[initialBase].z+apexHeight));
				break;
			}	case DOWNWARDS_PYRAMID:	{
				//Downwards-pointing pyramid. There must be 5 or less vertices.
				double apexHeight=baseRadius*sqrt(4*square(sin(M_PI/numBaseEdges))-1);
				face.vertices.resize(3);
				face.vertices[0]=verts.size();
				face.vertices[1]=endBase+numBaseEdges-1;
				face.vertices[2]=endBase;
				do	{
					faces.push_back(face);
					face.vertices[1]=face.vertices[2];
					face.vertices[2]++;
				}	while (face.vertices[2]<endBase+numBaseEdges);
				verts.push_back(TPoint3D(0,0,verts[endBase].z-apexHeight));
				break;
			}	case UPWARDS_CUPOLA:
				//Upwards-pointing cupola. There must be an even amount of vertices.
				insertCupola(numBaseEdges,basePositionInfo.rbegin()->second*semi,baseRadius,edgeLength,false,true,initialBase,verts,faces);
			break;
			case DOWNWARDS_CUPOLA:
				//Downwards-pointing cupola. There must be an even amount of vertices.
				insertCupola(numBaseEdges,basePositionInfo[0].second*semi,baseRadius,edgeLength,false,false,endBase,verts,faces);
			break;
			case ROTATED_UPWARDS_CUPOLA:
				//Upwards-pointing, slightly rotated, cupola. There must be an even amount of vertices.
				insertCupola(numBaseEdges,basePositionInfo.rbegin()->second*semi,baseRadius,edgeLength,true,true,initialBase,verts,faces);
			break;
			case ROTATED_DOWNWARDS_CUPOLA:
				//Downwards-pointing, slightly rotated, cupola. There must be an even amount of vertices.
				insertCupola(numBaseEdges,basePositionInfo[0].second*semi,baseRadius,edgeLength,true,false,endBase,verts,faces);
			break;
			case PRISM:
				//Archimedean prism.
				face.vertices.resize(4);
				for (size_t i=0;i<numBaseEdges;i++)	{
					size_t ii=(i+1)%numBaseEdges;
					face.vertices[0]=initialBase+i;
					face.vertices[1]=endBase+i;
					face.vertices[2]=endBase+ii;
					face.vertices[3]=initialBase+ii;
					faces.push_back(face);
				}
			break;
			case ANTIPRISM:	{
				//Archimedean antiprism.
				face.vertices.resize(3);
				face.vertices[0]=initialBase;
				face.vertices[1]=endBase;
				face.vertices[2]=initialBase+1;
				bool nextIsEnd=true;
				size_t nextEnd=1;
				size_t nextInitial=2;
				for (size_t i=0;i<numBaseEdges<<1;i++)	{
					faces.push_back(face);
					face.vertices[0]=face.vertices[1];
					face.vertices[1]=face.vertices[2];
					if (nextIsEnd)	{
						face.vertices[2]=endBase+nextEnd;
						nextEnd=(nextEnd+1)%numBaseEdges;
					}	else	{
						face.vertices[2]=initialBase+nextInitial;
						nextInitial=(nextInitial+1)%numBaseEdges;
					}
					nextIsEnd=!nextIsEnd;
				}
				break;
			}	case UPWARDS_ROTUNDA:
				//Upwards-pointing pentagonal rotunda. Only for bases of exactly 10 vertices.
				insertRotunda(basePositionInfo.rbegin()->second*semi,baseRadius,false,true,initialBase,verts,faces);
			break;
			case DOWNWARDS_ROTUNDA:
				//Downwards-pointing pentagonal rotunda. Only for bases of exactly 10 vertices.
				insertRotunda(basePositionInfo[0].second*semi,baseRadius,false,false,endBase,verts,faces);
			break;
			case ROTATED_UPWARDS_ROTUNDA:
				//Upwards-pointing, slightly rotated, pentagonal rotunda. Only for bases of exactly 10 vertices.
				insertRotunda(basePositionInfo.rbegin()->second*semi,baseRadius,true,true,initialBase,verts,faces);
			break;
			case ROTATED_DOWNWARDS_ROTUNDA:
				//Downwards-pointing, slightly rotated, pentagonal rotunda. Only for bases of exactly 10 vertices.
				insertRotunda(basePositionInfo[0].second*semi,baseRadius,true,false,endBase,verts,faces);
			break;
			default:
				throw std::logic_error("Internal error");
		}
		initialBase=endBase;
		endBase+=numBaseEdges;
	}
	return CreateNoCheck(verts,faces);
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/

void CPolyhedron::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	if (mWireframe)	{
		glDisable(GL_LIGHTING);  // Disable lights when drawing lines

		glLineWidth(mLineWidth);
		checkOpenGLError();
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
		glBegin(GL_LINES);
		for (vector<TPolyhedronEdge>::const_iterator it=mEdges.begin();it!=mEdges.end();++it)	{
			TPoint3D p=mVertices[it->v1];
			glVertex3f(p.x,p.y,p.z);
			p=mVertices[it->v2];
			glVertex3f(p.x,p.y,p.z);
		}
		glEnd();
		glEnable(GL_LIGHTING);  // Disable lights when drawing lines
	}	else	{
		checkOpenGLError();
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
		for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it)	{
			glBegin(GL_POLYGON);
			glNormal3f(it->normal[0],it->normal[1],it->normal[2]);
			for (vector<uint32_t>::const_iterator it2=it->vertices.begin();it2!=it->vertices.end();++it2)	{
				const TPoint3D &p=mVertices[*it2];
				glVertex3f(p.x,p.y,p.z);
			}
			glEnd();
		}
		glDisable(GL_BLEND);

	}
#endif
}

bool CPolyhedron::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	if (!polygonsUpToDate) updatePolygons();
	return math::traceRay(tempPolygons,o-this->m_pose,dist);
}

void CPolyhedron::getEdgesLength(std::vector<double> &lengths) const	{
	lengths.resize(mEdges.size());
	std::vector<double>::iterator it2=lengths.begin();
	for (std::vector<TPolyhedronEdge>::const_iterator it=mEdges.begin();it!=mEdges.end();++it,++it2) *it2=it->length(mVertices);
}

void CPolyhedron::getFacesArea(std::vector<double> &areas) const	{
	areas.resize(mFaces.size());
	std::vector<double>::iterator it2=areas.begin();
	for (std::vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it,++it2) *it2=it->area(mVertices);
}

double CPolyhedron::getVolume() const	{
	//TODO. Calculate as set of pyramids whose apices are situated in the center of the polyhedron (will work only with convex polyhedrons).
	//Pyramid volume=V=1/3*base area*height. Height=abs((A-V)·N), where A is the apex, V is any other vertex, N is the base's normal vector and (·) is the scalar product.
	TPoint3D center;
	getCenter(center);
	double res=0;
	if (!polygonsUpToDate) updatePolygons();
	vector<TPolygonWithPlane>::const_iterator itP=tempPolygons.begin();
	vector<double> areas(mFaces.size());
	getFacesArea(areas);
	vector<double>::const_iterator itA=areas.begin();
	for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it,++itP,++itA) res+=abs(itP->plane.distance(center))*(*itA);
	return res/3;
}

void CPolyhedron::getSetOfPolygons(std::vector<math::TPolygon3D> &vec) const	{
	if (!polygonsUpToDate) updatePolygons();
	size_t N=tempPolygons.size();
	vec.resize(N);
	for (size_t i=0;i<N;i++) vec[i]=tempPolygons[i].poly;
}

void CPolyhedron::getSetOfPolygonsAbsolute(std::vector<math::TPolygon3D> &vec) const	{
	vec.resize(mFaces.size());
	size_t N=mVertices.size();
	vector<TPoint3D> nVerts;
	nVerts.resize(N);
	CPose3D pose=this->m_pose;
	for (size_t i=0;i<N;i++) pose.composePoint(mVertices[i],nVerts[i]);
	transform(mFaces.begin(),mFaces.end(),vec.begin(),FCreatePolygonFromFace<TPolygon3D>(nVerts));
}

void CPolyhedron::makeConvexPolygons()	{
	vector<TPolygon3D> polys,polysTMP,polys2;
	getSetOfPolygons(polys);
	polys2.reserve(polys.size());
	for (vector<TPolygon3D>::const_iterator it=polys.begin();it!=polys.end();++it) if (mrpt::math::splitInConvexComponents(*it,polysTMP)) polys2.insert(polys2.end(),polysTMP.begin(),polysTMP.end());
	else polys2.push_back(*it);
	mVertices.clear();
	mEdges.clear();
	mFaces.clear();
	getVerticesAndFaces(polys2,mVertices,mFaces);
	for (vector<TPolyhedronFace>::iterator it=mFaces.begin();it!=mFaces.end();++it)	{
		if (!setNormal(*it,false)) throw std::logic_error("Bad face specification");
		addEdges(*it);
	}
}

void CPolyhedron::getCenter(TPoint3D &center) const	{
	size_t N=mVertices.size();
	if (N==0) throw new std::logic_error("There are no vertices");
	center.x=center.y=center.z=0;
	for (vector<TPoint3D>::const_iterator it=mVertices.begin();it!=mVertices.end();++it)	{
		center.x+=it->x;
		center.y+=it->y;
		center.z+=it->z;
	}
	center.x/=N;
	center.y/=N;
	center.z/=N;
}

CPolyhedronPtr CPolyhedron::CreateRandomPolyhedron(double radius)	{
	switch (mrpt::random::randomGenerator.drawUniform32bit()%34)	{
		case 0:return CreateTetrahedron(radius);
		case 1:return CreateHexahedron(radius);
		case 2:return CreateOctahedron(radius);
		case 3:return CreateDodecahedron(radius);
		case 4:return CreateIcosahedron(radius);
		case 5:return CreateTruncatedTetrahedron(radius);
		case 6:return CreateTruncatedHexahedron(radius);
		case 7:return CreateTruncatedOctahedron(radius);
		case 8:return CreateTruncatedDodecahedron(radius);
		case 9:return CreateTruncatedIcosahedron(radius);
		case 10:return CreateCuboctahedron(radius);
		case 11:return CreateRhombicuboctahedron(radius);
		case 12:return CreateIcosidodecahedron(radius);
		case 13:return CreateRhombicosidodecahedron(radius);
		case 14:return CreateTriakisTetrahedron(radius);
		case 15:return CreateTriakisOctahedron(radius);
		case 16:return CreateTetrakisHexahedron(radius);
		case 17:return CreateTriakisIcosahedron(radius);
		case 18:return CreatePentakisDodecahedron(radius);
		case 19:return CreateRhombicDodecahedron(radius);
		case 20:return CreateDeltoidalIcositetrahedron(radius);
		case 21:return CreateRhombicTriacontahedron(radius);
		case 22:return CreateDeltoidalHexecontahedron(radius);
		case 23:return CreateArchimedeanRegularPrism((mrpt::random::randomGenerator.drawUniform32bit()%10)+3,radius);
		case 24:return CreateArchimedeanRegularAntiprism((mrpt::random::randomGenerator.drawUniform32bit()%10)+3,radius);
		case 25:return CreateJohnsonSolidWithConstantBase(((mrpt::random::randomGenerator.drawUniform32bit()%4)<<1)+4,radius,"C+");
		case 26:return CreateJohnsonSolidWithConstantBase(((mrpt::random::randomGenerator.drawUniform32bit()%4)<<1)+4,radius,"C-C+");
		case 27:return CreateJohnsonSolidWithConstantBase(((mrpt::random::randomGenerator.drawUniform32bit()%4)<<1)+4,radius,"C-PRC+");
		case 28:return CreateJohnsonSolidWithConstantBase(((mrpt::random::randomGenerator.drawUniform32bit()%4)<<1)+4,radius,"C-AC+");
		case 29:return CreateJohnsonSolidWithConstantBase(10,radius,"R+");
		case 30:return CreateJohnsonSolidWithConstantBase(10,radius,"R-PRR+");
		case 31:return CreateJohnsonSolidWithConstantBase(10,radius,"R-AR+");
		case 32:return CreateCatalanTrapezohedron((mrpt::random::randomGenerator.drawUniform32bit()%5)+3,radius);
		case 33:return CreateCatalanDoublePyramid((mrpt::random::randomGenerator.drawUniform32bit()%5)+3,radius);
		default:return CreateEmpty();
	}
}

CPolyhedronPtr CPolyhedron::getDual() const	{
	//This methods builds the dual of a given polyhedron, which is assumed to be centered in (0,0,0), using polar reciprocation.
	//A vertex (x0,y0,z0), which is inside a circunference x^2+y^2+z^2=r^2, its dual face will lie on the x0·x+y0·y+z0·z=r^2 plane.
	//The new vertices can, then, be calculated as the corresponding intersections between three or more planes.
	size_t NV=mFaces.size();
	size_t NE=mEdges.size();
	size_t NF=mVertices.size();
	vector<TPlane> planes(NF);
	for (size_t i=0;i<NF;i++)	{
		const TPoint3D &p=mVertices[i];
		TPlane &pl=planes[i];
		pl.coefs[0]=p.x;
		pl.coefs[1]=p.y;
		pl.coefs[2]=p.z;
		pl.coefs[3]=-square(p.x)-square(p.y)-square(p.z);
	}
	CMatrixTemplate<bool> incidence(NV,NF);
	vector<TPoint3D> vertices(NV);
	for (size_t i=0;i<NV;i++)	{
		for (size_t j=0;j<NF;j++) incidence(i,j)=false;
		vector<const TPlane *> fPls;
		fPls.reserve(mFaces[i].vertices.size());
		for (vector<uint32_t>::const_iterator it=mFaces[i].vertices.begin();it!=mFaces[i].vertices.end();++it)	{
			incidence(i,*it)=true;
			fPls.push_back(&planes[*it]);
		}
		if (!getPlanesIntersection(fPls,vertices[i])) throw std::logic_error("Dual polyhedron cannot be found");
	}
	vector<TPolyhedronFace> faces(NF);
	for (size_t i=0;i<NF;i++) for (size_t j=0;j<NV;j++) if (incidence(j,i)) faces[i].vertices.push_back(j);
	//The following code ensures that the faces' vertex list is in the adequate order.
	CMatrixTemplate<bool> arrayEF(NE,NV);
	for (size_t i=0;i<NE;i++) for (size_t j=0;j<NV;j++) arrayEF(i,j)=faceContainsEdge(mFaces[j],mEdges[i]);
	for (vector<TPolyhedronFace>::iterator it=faces.begin();it!=faces.end();++it)	{
		vector<uint32_t> &face=it->vertices;
		if (face.size()<=3) continue;
		size_t index=0;
		while (index<face.size()-1)	{
			bool err=true;
			while (err)	{
				for (size_t i=0;i<NE;i++) if (arrayEF(i,face[index])&&arrayEF(i,face[index+1]))	{
					err=false;
					break;
				}
				if (err)	{
					size_t val=face[index+1];
					face.erase(face.begin()+index+1);
					face.push_back(val);
				}
			}
			index++;
		}
	}
	return Create(vertices,faces);
}

CPolyhedronPtr CPolyhedron::truncate(double factor) const	{
	if (factor<0) return CreateEmpty();
	if (factor==0) return CreateNoCheck(mVertices,mFaces);
	else if (factor<1)	{
		size_t NE=mEdges.size();
		size_t NV=mVertices.size();
		size_t NF=mFaces.size();
		vector<TPoint3D> vertices(NE<<1);
		vector<TPolyhedronFace> faces(NV+NF);
		for (size_t i=0;i<NE;i++)	{
			const TPoint3D &p1=mVertices[mEdges[i].v1];
			const TPoint3D &p2=mVertices[mEdges[i].v2];
			TPoint3D &v1=vertices[i+i];
			TPoint3D &v2=vertices[i+i+1];
			for (size_t j=0;j<3;j++)	{
				double d=(p2[j]-p1[j])*factor/2;
				v1[j]=p1[j]+d;
				v2[j]=p2[j]-d;
			}
			faces[mEdges[i].v1].vertices.push_back(i+i);
			faces[mEdges[i].v2].vertices.push_back(i+i+1);
		}
		for (size_t i=0;i<NV;i++)	{
			vector<uint32_t> &f=faces[i].vertices;
			size_t sf=f.size();
			if (sf==3) continue;
			for (size_t j=1;j<sf-1;j++)	{
				const TPolyhedronEdge &e1=mEdges[f[j-1]/2];
				for (;;)	{
					const TPolyhedronEdge &e2=mEdges[f[j]/2];
					if (!((e1.v1==i||e1.v2==i)&&(e2.v1==i||e2.v2==i))) THROW_EXCEPTION("En algo te has equivocado, chaval.");
					if (searchForFace(mFaces,i,(e1.v1==i)?e1.v2:e1.v1,(e2.v1==i)?e2.v2:e2.v1)) break;
					uint32_t tmpV=f[j];
					f.erase(f.begin()+j);
					f.push_back(tmpV);
				}
			}
		}
		for (size_t i=0;i<NF;i++)	{
			vector<uint32_t> &f=faces[i+NV].vertices;
			const vector<uint32_t> &cf=mFaces[i].vertices;
			size_t hmV=cf.size();
			f.reserve(hmV<<1);
			for (size_t j=0;j<hmV;j++)	{
				size_t where;
				if (searchForEdge(mEdges,cf[j],cf[(j+1)%hmV],where))	{
					f.push_back(where<<1);
					f.push_back((where<<1)+1);
				}	else	{
					f.push_back((where<<1)+1);
					f.push_back(where<<1);
				}
			}
		}
		return Create(vertices,faces);
	}	else if (factor==1)	{
		size_t NE=mEdges.size();
		size_t NV=mVertices.size();
		size_t NF=mFaces.size();
		vector<TPoint3D> vertices(NE);
		vector<TPolyhedronFace> faces(NV+NF);
		for (size_t i=0;i<NE;i++)	{
			const TPoint3D &p1=mVertices[mEdges[i].v1];
			const TPoint3D &p2=mVertices[mEdges[i].v2];
			TPoint3D &dst=vertices[i];
			for (size_t j=0;j<3;j++) dst[j]=(p1[j]+p2[j])/2;
			faces[mEdges[i].v1].vertices.push_back(i);
			faces[mEdges[i].v2].vertices.push_back(i);
		}
		for (size_t i=0;i<NV;i++)	{
			vector<uint32_t> &f=faces[i].vertices;
			size_t sf=f.size();
			if (sf==3) continue;
			for (size_t j=1;j<sf-1;j++)	{
				const TPolyhedronEdge &e1=mEdges[f[j-1]];
				for (;;)	{
					const TPolyhedronEdge &e2=mEdges[f[j-1]];
					if (!((e1.v1==i||e1.v2==i)&&(e2.v1==1||e2.v2==i))) THROW_EXCEPTION("En algo te has equivocado, chaval.");
					if (searchForFace(mFaces,i,(e1.v1==i)?e1.v2:e1.v1,(e2.v1==i)?e2.v2:e2.v1)) break;
					uint32_t tmpV=f[j];
					f.erase(f.begin()+j);
					f.push_back(tmpV);
				}
			}
		}
		for (size_t i=0;i<NF;i++)	{
			vector<uint32_t> &f=faces[i+NV].vertices;
			const vector<uint32_t> &cf=mFaces[i].vertices;
			size_t hmV=cf.size();
			f.reserve(hmV);
			for (size_t j=0;j<hmV;j++)	{
				size_t where;
				searchForEdge(mEdges,cf[j],cf[(j+1)%hmV],where);
				f.push_back(where);
			}
		}
		return Create(vertices,faces);
	}	else return CreateEmpty();
}

CPolyhedronPtr CPolyhedron::cantellate(double factor) const	{
	if (factor<0) return CreateEmpty();
	else if (factor==0) return CreateNoCheck(mVertices,mFaces);
	size_t NV=mVertices.size();
	size_t NE=mEdges.size();
	size_t NF=mFaces.size();
	vector<TPolygon3D> origFaces(NF);
	getSetOfPolygons(origFaces);
	TPoint3D cnt;
	getCenter(cnt);
	vector<TPoint3D> polyCenters(NF);
	vector<TPoint3D> polyNewCenters(NF);
	size_t NNV=0;
	for (size_t i=0;i<NF;i++)	{
		origFaces[i].getCenter(polyCenters[i]);
		polyCenters[i]-=cnt;
		polyNewCenters[i]=polyCenters[i];
		polyNewCenters[i]*=(1+factor);
		polyNewCenters[i]+=cnt;
		NNV+=origFaces[i].size();
	}
	vector<TPoint3D> vertices(NNV);
	vector<TPolyhedronFace> faces(NF+NV+NE);
	size_t ind=0;
	for (size_t i=0;i<NF;i++)	{
		const TPoint3D &oC=polyCenters[i];
		const TPoint3D &nC=polyNewCenters[i];
		const TPolygon3D &oP=origFaces[i];
		vector<uint32_t> &f=faces[i].vertices;
		size_t oPS=oP.size();
		for (size_t j=0;j<oPS;j++)	{
			vertices[j+ind]=nC+(oP[j]-oC);
			f.push_back(j+ind);
			size_t curr=mFaces[i].vertices[j];
			faces[NF+curr].vertices.push_back(j+ind);
			size_t edge;
			searchForEdge(mEdges,curr,mFaces[i].vertices[(j+oPS-1)%oPS],edge);
			faces[NF+NV+edge].vertices.push_back(j+ind);
			searchForEdge(mEdges,curr,mFaces[i].vertices[(j+1)%oPS],edge);
			faces[NF+NV+edge].vertices.push_back(j+ind);
		}
		ind+=oPS;
	}
	vector<TPolyhedronFace>::const_iterator begin=faces.begin(),edgeBegin=faces.begin()+NF+NV,end=faces.end();
	for (vector<TPolyhedronFace>::iterator it=faces.begin()+NF;it!=faces.begin()+NF+NV;++it)	{
		vector<uint32_t> &f=it->vertices;
		if (f.size()==3) continue;
		for (size_t i=1;i<f.size()-1;i++) for(;;) if (searchForEdge(edgeBegin,end,f[i-1],f[i])) break;
		else	{
			uint32_t tmp=f[i];
			f.erase(f.begin()+i);
			f.push_back(tmp);
		}
	}
	for (vector<TPolyhedronFace>::iterator it=faces.begin()+NF+NV;it!=faces.end();++it)	{
		vector<uint32_t> &f=it->vertices;	//Will always have exactly 4 vertices
		for (size_t i=1;i<3;i++) for (;;) if (searchForEdge(begin,edgeBegin,f[i-1],f[i])) break;
		else	{
			uint32_t tmp=f[i];
			f.erase(f.begin()+i);
			f.push_back(tmp);
		}
	}
	return Create(vertices,faces);
}

CPolyhedronPtr CPolyhedron::augment(double height) const	{
	size_t NV=mVertices.size();
	size_t NF=mFaces.size();
	vector<TPoint3D> vertices(NV+NF);
	std::copy(mVertices.begin(),mVertices.end(),vertices.begin());
	size_t tnf=0;
	for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it) tnf+=it->vertices.size();
	vector<TPolyhedronFace> faces(tnf);
	TPolygon3D tmp;
	TPlane pTmp;
	TPoint3D cTmp;
	size_t iF=0;
	TPoint3D phCenter;
	getCenter(phCenter);
	TPolyhedronFace fTmp;
	fTmp.vertices.resize(3);
	for (size_t i=0;i<NF;i++)	{
		TPoint3D &vertex=vertices[NV+i];
		const vector<uint32_t> &face=mFaces[i].vertices;
		size_t N=face.size();
		tmp.resize(N);
		for (size_t j=0;j<N;j++) tmp[j]=mVertices[face[j]];
		tmp.getBestFittingPlane(pTmp);
		pTmp.unitarize();
		tmp.getCenter(cTmp);
		if (pTmp.evaluatePoint(phCenter)>0)	for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]-height*pTmp.coefs[j];
		else for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]+height*pTmp.coefs[j];
		fTmp.vertices[0]=NV+i;
		for (size_t j=0;j<N;j++)	{
			fTmp.vertices[1]=face[j];
			fTmp.vertices[2]=face[(j+1)%N];
			faces[iF+j]=fTmp;
		}
		iF+=N;
	}
	return CreateNoCheck(vertices,faces);
}

CPolyhedronPtr CPolyhedron::augment(double height,size_t numVertices) const	{
	size_t NV=mVertices.size();
	size_t NF=mFaces.size();
	size_t tnf=0;
	size_t tnv=NV;
	for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it) if (it->vertices.size()==numVertices)	{
		tnf+=numVertices;
		tnv++;
	}	else tnf++;
	if (tnv==NV) return CreateNoCheck(mVertices,mFaces);
	vector<TPoint3D> vertices(tnv);
	std::copy(mVertices.begin(),mVertices.end(),vertices.begin());
	vector<TPolyhedronFace> faces(tnf);
	TPolygon3D tmp(numVertices);
	TPlane pTmp;
	TPoint3D cTmp;
	size_t iF=0;
	size_t iV=NV;
	TPoint3D phCenter;
	getCenter(phCenter);
	TPolyhedronFace fTmp;
	fTmp.vertices.resize(3);
	for (size_t i=0;i<NF;i++)	{
		const vector<uint32_t> &face=mFaces[i].vertices;
		size_t N=face.size();
		if (N!=numVertices)	{
			faces[iF].vertices=face;
			iF++;
			continue;
		}
		TPoint3D &vertex=vertices[iV];
		for (size_t j=0;j<numVertices;j++) tmp[j]=mVertices[face[j]];
		tmp.getBestFittingPlane(pTmp);
		pTmp.unitarize();
		tmp.getCenter(cTmp);
		if (pTmp.evaluatePoint(phCenter)>0) for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]-height*pTmp.coefs[j];
		else for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]+height*pTmp.coefs[j];
		fTmp.vertices[0]=iV;
		for (size_t j=0;j<N;j++)	{
			fTmp.vertices[1]=face[j];
			fTmp.vertices[2]=face[(j+1)%N];
			faces[iF+j]=fTmp;
		}
		iF+=N;
		iV++;
	}
	return CreateNoCheck(vertices,faces);
}

CPolyhedronPtr CPolyhedron::augment(bool direction) const	{
	size_t NV=mVertices.size();
	size_t NF=mFaces.size();
	vector<TPoint3D> vertices(NV+NF);
	std::copy(mVertices.begin(),mVertices.end(),vertices.begin());
	size_t tnf=0;
	for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it) tnf+=it->vertices.size();
	vector<TPolyhedronFace> faces(tnf);
	TPolygon3D tmp;
	TPlane pTmp;
	TPoint3D cTmp;
	size_t iF=0;
	TPoint3D phCenter;
	getCenter(phCenter);
	TPolyhedronFace fTmp;
	fTmp.vertices.resize(3);
	for (size_t i=0;i<NF;i++)	{
		TPoint3D &vertex=vertices[NV+i];
		const vector<uint32_t> &face=mFaces[i].vertices;
		size_t N=face.size();
		tmp.resize(N);
		for (size_t j=0;j<N;j++) tmp[j]=mVertices[face[j]];
		tmp.getCenter(cTmp);
		double height=getHeight(tmp,cTmp);	//throws std::logic_error
		tmp.getBestFittingPlane(pTmp);
		pTmp.unitarize();
		if ((pTmp.evaluatePoint(phCenter)<0)==direction)	for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]-height*pTmp.coefs[j];
		else for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]+height*pTmp.coefs[j];
		fTmp.vertices[0]=NV+i;
		for (size_t j=0;j<N;j++)	{
			fTmp.vertices[1]=face[j];
			fTmp.vertices[2]=face[(j+1)%N];
			faces[iF+j]=fTmp;
		}
		iF+=N;
	}
	return CreateNoCheck(vertices,faces);

}

CPolyhedronPtr CPolyhedron::augment(size_t numVertices,bool direction) const	{
	size_t NV=mVertices.size();
	size_t NF=mFaces.size();
	size_t tnf=0;
	size_t tnv=NV;
	for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it) if (it->vertices.size()==numVertices)	{
		tnf+=numVertices;
		tnv++;
	}	else tnf++;
	if (tnv==NV) return CreateNoCheck(mVertices,mFaces);
	vector<TPoint3D> vertices(tnv);
	std::copy(mVertices.begin(),mVertices.end(),vertices.begin());
	vector<TPolyhedronFace> faces(tnf);
	TPolygon3D tmp(numVertices);
	TPlane pTmp;
	TPoint3D cTmp;
	size_t iF=0;
	size_t iV=NV;
	TPoint3D phCenter;
	getCenter(phCenter);
	TPolyhedronFace fTmp;
	fTmp.vertices.resize(3);
	for (size_t i=0;i<NF;i++)	{
		const vector<uint32_t> &face=mFaces[i].vertices;
		size_t N=face.size();
		if (N!=numVertices)	{
			faces[iF].vertices=face;
			iF++;
			continue;
		}
		TPoint3D &vertex=vertices[iV];
		for (size_t j=0;j<numVertices;j++) tmp[j]=mVertices[face[j]];
		tmp.getBestFittingPlane(pTmp);
		pTmp.unitarize();
		tmp.getCenter(cTmp);
		double height=getHeight(tmp,cTmp);	//throws std::logic_error
		if ((pTmp.evaluatePoint(phCenter)<0)==direction) for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]-height*pTmp.coefs[j];
		else for (size_t j=0;j<3;j++) vertex[j]=cTmp[j]+height*pTmp.coefs[j];
		fTmp.vertices[0]=iV;
		for (size_t j=0;j<N;j++)	{
			fTmp.vertices[1]=face[j];
			fTmp.vertices[2]=face[(j+1)%N];
			faces[iF+j]=fTmp;
		}
		iF+=N;
		iV++;
	}
	return CreateNoCheck(vertices,faces);
}

CPolyhedronPtr CPolyhedron::rotate(double angle) const	{
	vector<TPoint3D> vertices(mVertices);
	double c=cos(angle),s=sin(angle);
	for (vector<TPoint3D>::iterator it=vertices.begin();it!=vertices.end();++it)	{
		double A=it->x;
		double B=it->y;
		it->x=A*c-B*s;
		it->y=B*c+A*s;
	}
	return CreateNoCheck(vertices,mFaces);
}

CPolyhedronPtr CPolyhedron::scale(double factor) const	{
	vector<TPoint3D> vertices(mVertices);
	if (factor<=0) throw std::logic_error("Factor must be a strictly positive number");
	for (vector<TPoint3D>::iterator it=vertices.begin();it!=vertices.end();++it)	{
		it->x*=factor;
		it->y*=factor;
	}
	return CreateNoCheck(vertices,mFaces);
}

vector<TPoint2D> CPolyhedron::generateBase(uint32_t numBaseEdges,double baseRadius)	{
	vector<TPoint2D> base(numBaseEdges);
	for (size_t i=0;i<numBaseEdges;i++)	{
		double ang=2*M_PI*i/numBaseEdges;
		base[i].x=baseRadius*cos(ang);
		base[i].y=baseRadius*sin(ang);
	}
	return base;
}

vector<TPoint2D> CPolyhedron::generateShiftedBase(uint32_t numBaseEdges,double baseRadius)	{
	vector<TPoint2D> base(numBaseEdges);
	double shift=M_PI/numBaseEdges;
	for (size_t i=0;i<numBaseEdges;i++)	{
		double ang=shift+2*M_PI*i/numBaseEdges;
		base[i].x=baseRadius*cos(ang);
		base[i].y=baseRadius*sin(ang);
	}
	return base;
}

void CPolyhedron::generateBase(uint32_t numBaseEdges,double baseRadius,double height,vector<TPoint3D> &vec)	{
	for (size_t i=0;i<numBaseEdges;i++)	{
		double ang=2*M_PI*i/numBaseEdges;
		vec.push_back(TPoint3D(baseRadius*cos(ang),baseRadius*sin(ang),height));
	}
}

void CPolyhedron::generateShiftedBase(uint32_t numBaseEdges,double baseRadius,double height,double shift,vector<TPoint3D> &vec)	{
	for (size_t i=0;i<numBaseEdges;i++)	{
		double ang=2*M_PI*i/numBaseEdges+shift;
		vec.push_back(TPoint3D(baseRadius*cos(ang),baseRadius*sin(ang),height));
	}
}

void CPolyhedron::updatePolygons() const	{
	tempPolygons.resize(mFaces.size());
	transform(mFaces.begin(),mFaces.end(),tempPolygons.begin(),FCreatePolygonFromFace<TPolygonWithPlane>(mVertices));
	polygonsUpToDate=true;
}

bool CPolyhedron::setNormal(TPolyhedronFace &f,bool doCheck)	{
	size_t N=doCheck?f.vertices.size():3;
	TPolygon3D poly(N);
	for (size_t i=0;i<N;i++) poly[i]=mVertices[f.vertices[i]];
	TPlane tmp;
	if (!poly.getPlane(tmp)) return false;
	tmp.getNormalVector(f.normal);
	TPoint3D c;
	getCenter(c);
	if (tmp.evaluatePoint(c)>0) for (size_t i=0;i<3;i++) f.normal[i]=-f.normal[i];
	return true;
}

void CPolyhedron::addEdges(const TPolyhedronFace &f)	{
	TPolyhedronEdge e;
	vector<uint32_t>::const_iterator it=f.vertices.begin();
	e.v1=*it;
	++it;
	while (it!=f.vertices.end())	{
		e.v2=*it;
		if (find(mEdges.begin(),mEdges.end(),e)==mEdges.end()) mEdges.push_back(e);
		e.v1=e.v2;
		++it;
	}
	e.v2=*(f.vertices.begin());
	if (find(mEdges.begin(),mEdges.end(),e)==mEdges.end()) mEdges.push_back(e);
}

bool CPolyhedron::checkConsistence(const vector<TPoint3D> &vertices,const vector<TPolyhedronFace> &faces)	{
	size_t N=vertices.size();
	if (vertices.size()>0) for (vector<TPoint3D>::const_iterator it=vertices.begin();it!=vertices.end()-1;++it) for (vector<TPoint3D>::const_iterator it2=it+1;it2!=vertices.end();++it2) if (*it==*it2) return false;
	for (vector<TPolyhedronFace>::const_iterator it=faces.begin();it!=faces.end();++it)	{
		const vector<uint32_t> &e=it->vertices;
		for (vector<uint32_t>::const_iterator it2=e.begin();it2!=e.end();++it2) if (*it2>=N) return false;
	}
	return true;
}

size_t CPolyhedron::edgesInVertex(size_t vertex) const	{
	size_t res=0;
	for (vector<TPolyhedronEdge>::const_iterator it=mEdges.begin();it!=mEdges.end();++it) if (it->v1==vertex||it->v2==vertex) res++;
	return res;
}

size_t CPolyhedron::facesInVertex(size_t vertex) const	{
	size_t res=0;
	for (vector<TPolyhedronFace>::const_iterator it=mFaces.begin();it!=mFaces.end();++it) if (find(it->vertices.begin(),it->vertices.end(),vertex)!=it->vertices.end()) res++;
	return res;
}

CStream &mrpt::opengl::operator>>(mrpt::utils::CStream& in,CPolyhedron::TPolyhedronEdge &o)	{
	in>>o.v1>>o.v2;
	return in;
}

CStream &mrpt::opengl::operator<<(mrpt::utils::CStream &out,const CPolyhedron::TPolyhedronEdge &o)	{
	out<<o.v1<<o.v2;
	return out;
}

CStream &mrpt::opengl::operator>>(mrpt::utils::CStream& in,CPolyhedron::TPolyhedronFace &o)	{
	in>>o.vertices>>o.normal[0]>>o.normal[1]>>o.normal[2];
	return in;
}

CStream &mrpt::opengl::operator<<(mrpt::utils::CStream& out,const CPolyhedron::TPolyhedronFace &o)	{
	out<<o.vertices<<o.normal[0]<<o.normal[1]<<o.normal[2];
	return out;
}

void CPolyhedron::writeToStream(mrpt::utils::CStream &out,int *version) const	{
	if (version) *version=0;
	else	{
		writeToStreamRender(out);
		//version 0
		out<<mVertices<<mFaces<<mWireframe<<mLineWidth;
	}
}

void CPolyhedron::readFromStream(mrpt::utils::CStream &in,int version)	{
	switch (version)	{
		case 0:
			readFromStreamRender(in);
			in>>mVertices>>mFaces>>mWireframe>>mLineWidth;
			if (!checkConsistence(mVertices,mFaces)) throw std::logic_error("Inconsistent data read from stream");
			for (vector<TPolyhedronFace>::iterator it=mFaces.begin();it!=mFaces.end();++it)	{
				if (!setNormal(*it)) throw std::logic_error("Bad face specification");
				addEdges(*it);
			}
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}


void CPolyhedron::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max.x = 0;
	bb_max.y = 0;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}


/*CPolyhedronPtr CPolyhedron::CreateCuboctahedron(double radius)	{
	if (radius==0) return CreateEmpty();
	vector<TPoint3D> verts;
	vector<TPolyhedronFace> faces;
	double d=radius/sqrt(2.0);
	verts.push_back(TPoint3D(d,0,d));
	verts.push_back(TPoint3D(0,d,d));
	verts.push_back(TPoint3D(0,-d,d));
	verts.push_back(TPoint3D(-d,0,d));
	verts.push_back(TPoint3D(d,d,0));
	verts.push_back(TPoint3D(d,-d,0));
	verts.push_back(TPoint3D(-d,d,0));
	verts.push_back(TPoint3D(-d,-d,0));
	verts.push_back(TPoint3D(d,0,-d));
	verts.push_back(TPoint3D(0,d,-d));
	verts.push_back(TPoint3D(0,-d,-d));
	verts.push_back(TPoint3D(-d,0,-d));
	TPolyhedronFace f;
	static uint32_t faces3[]={0,1,4, 0,2,5, 1,3,6, 2,3,7, 8,9,4, 8,10,5, 9,11,6, 10,11,7};
	static uint32_t faces4[]={0,1,3,2, 8,9,11,10, 0,4,8,5, 1,4,9,6, 2,5,10,7, 3,6,11,7};
	for (uint32_t *p=reinterpret_cast<uint32_t *>(&faces3);p<24+reinterpret_cast<uint32_t *>(&faces3);p+=3)	{
		f.vertices.insert(f.vertices.begin(),p,p+3);
		faces.push_back(f);
		f.vertices.clear();
	}
	for (uint32_t *p=reinterpret_cast<uint32_t *>(&faces4);p<24+reinterpret_cast<uint32_t *>(&faces4);p+=4)	{
		f.vertices.insert(f.vertices.begin(),p,p+4);
		faces.push_back(f);
		f.vertices.clear();
	}
	return CreateNoCheck(verts,faces);
}*/

CPolyhedronPtr CPolyhedron::Create(const vector<TPoint3D> &vertices,const vector<vector<uint32_t> > &faces)	{
	vector<TPolyhedronFace> aux;
	for (vector<vector<uint32_t> >::const_iterator it=faces.begin();it!=faces.end();++it)	{
		TPolyhedronFace f;
		f.vertices=*it;
		aux.push_back(f);
	}
	return Create(vertices,aux);
}
CPolyhedronPtr CPolyhedron::Create(const vector<TPoint3D> &vertices,const vector<TPolyhedronFace> &faces)	{
	return CPolyhedronPtr(new CPolyhedron(vertices,faces,true));
}
CPolyhedronPtr CPolyhedron::CreateTetrahedron(double radius)	{
	CPolyhedronPtr tetra=CreateJohnsonSolidWithConstantBase(3,radius*sqrt(8.0)/3.0,"P+");
	for (vector<TPoint3D>::iterator it=tetra->mVertices.begin();it!=tetra->mVertices.end();++it) it->z-=radius/3;
	return tetra;
}
CPolyhedronPtr CPolyhedron::CreateHexahedron(double radius)	{
	if (radius==0.0) return CreateEmpty();
	double r=radius/sqrt(3.0);
	return CreateCubicPrism(-r,r,-r,r,-r,r);
}
CPolyhedronPtr CPolyhedron::CreateOctahedron(double radius)	{
	return CreateJohnsonSolidWithConstantBase(4,radius,"P-P+");
}
CPolyhedronPtr CPolyhedron::CreateDodecahedron(double radius)	{
	return CreateIcosahedron(radius/sqrt(15-6*sqrt(5.0)))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateIcosahedron(double radius)	{
	double ang=M_PI/5;
	double s2=4*square(sin(ang));
	double prop=sqrt(s2-1)+sqrt(s2-2+2*cos(ang))/2;
	return CreateJohnsonSolidWithConstantBase(5,radius/prop,"P-AP+",1);
}
CPolyhedronPtr CPolyhedron::CreateTruncatedTetrahedron(double radius)	{
	return CreateTetrahedron(radius*sqrt(27.0/11.0))->truncate(2.0/3.0);
}
CPolyhedronPtr CPolyhedron::CreateCuboctahedron(double radius)	{
	return CreateHexahedron(radius*sqrt(1.5))->truncate(1.0);
}
CPolyhedronPtr CPolyhedron::CreateTruncatedHexahedron(double radius)	{
	return CreateHexahedron(radius*sqrt(3.0/(5-sqrt(8.0))))->truncate(2-sqrt(2.0));
}
CPolyhedronPtr CPolyhedron::CreateTruncatedOctahedron(double radius)	{
	return CreateOctahedron(radius*3/sqrt(5.0))->truncate(2.0/3.0);
}
CPolyhedronPtr CPolyhedron::CreateRhombicuboctahedron(double radius,bool type)	{
	return CreateJohnsonSolidWithConstantBase(8,radius/sqrt(1+square(sin(M_PI/8))),type?"C-PRC+":"GC-PRC+",3);
}
CPolyhedronPtr CPolyhedron::CreateIcosidodecahedron(double radius,bool type)	{
	return CreateJohnsonSolidWithConstantBase(10,radius,type?"GR-R+":"R-R+",1);
}
CPolyhedronPtr CPolyhedron::CreateTruncatedDodecahedron(double radius)	{
	return CreateDodecahedron(radius*sqrt(45.0)/sqrt(27+6*sqrt(5.0)))->truncate(1-sqrt(0.2));
}
CPolyhedronPtr CPolyhedron::CreateTruncatedIcosahedron(double radius)	{
	return CreateIcosahedron(radius*sqrt(45.0)/sqrt(25+4*sqrt(5.0)))->truncate(2.0/3.0);
}
CPolyhedronPtr CPolyhedron::CreateRhombicosidodecahedron(double radius)	{
	return CreateIcosahedron(radius*sqrt(10.0/(35.0+9.0*sqrt(5.0))))->cantellate(1.5*(sqrt(5.0)-1));
}
CPolyhedronPtr CPolyhedron::CreatePentagonalRotunda(double radius)	{
	return CreateJohnsonSolidWithConstantBase(10,radius,"R+");
}
CPolyhedronPtr CPolyhedron::CreateTriakisTetrahedron(double radius)	{
	return CreateTruncatedTetrahedron(radius*3/sqrt(33.0))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateRhombicDodecahedron(double radius)	{
	return CreateCuboctahedron(radius/sqrt(2.0))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateTriakisOctahedron(double radius)	{
	return CreateTruncatedHexahedron(radius/sqrt((5-sqrt(8.0))))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateTetrakisHexahedron(double radius)	{
	return CreateTruncatedOctahedron(radius*sqrt(0.6))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateDeltoidalIcositetrahedron(double radius)	{
	return CreateRhombicuboctahedron(radius/sqrt(7-sqrt(32.0)),true)->getDual();
}
CPolyhedronPtr CPolyhedron::CreateRhombicTriacontahedron(double radius)	{
	return CreateIcosidodecahedron(radius*sqrt(2/(5-sqrt(5.0))),true)->getDual();
}
CPolyhedronPtr CPolyhedron::CreateTriakisIcosahedron(double radius)	{
	return CreateTruncatedDodecahedron(radius*sqrt(5/(25-8*sqrt(5.0))))->getDual();
}
CPolyhedronPtr CPolyhedron::CreatePentakisDodecahedron(double radius)	{
	return CreateTruncatedIcosahedron(radius*sqrt(3/(17-6*sqrt(5.0))))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateDeltoidalHexecontahedron(double radius)	{
	return CreateRhombicosidodecahedron(radius*3.0/sqrt(15-2*sqrt(5.0)))->getDual();
}
CPolyhedronPtr CPolyhedron::CreateCubicPrism(const TPoint3D &p1,const TPoint3D &p2)	{
	return CreateCubicPrism(p1.x,p2.x,p1.y,p2.y,p1.z,p2.z);
}
CPolyhedronPtr CPolyhedron::CreateFrustum(const vector<TPoint2D> &baseVertices,double height,double ratio)	{
	return CreateTruncatedPyramid(baseVertices,height,ratio);
}
CPolyhedronPtr CPolyhedron::CreateCustomPrism(const vector<TPoint2D> &baseVertices,double height)	{
	return CreateTruncatedPyramid(baseVertices,height,1.0);
}
CPolyhedronPtr CPolyhedron::CreateRegularAntiprism(uint32_t numBaseEdges,double baseRadius,double height)	{
	return CreateCustomAntiprism(generateBase(numBaseEdges,baseRadius),generateShiftedBase(numBaseEdges,baseRadius),height);
}
CPolyhedronPtr CPolyhedron::CreateRegularPrism(uint32_t numBaseEdges,double baseRadius,double height)	{
	return CreateCustomPrism(generateBase(numBaseEdges,baseRadius),height);
}
CPolyhedronPtr CPolyhedron::CreateRegularPyramid(uint32_t numBaseEdges,double baseRadius,double height)	{
	return CreatePyramid(generateBase(numBaseEdges,baseRadius),height);
}
CPolyhedronPtr CPolyhedron::CreateRegularDoublePyramid(uint32_t numBaseEdges,double baseRadius,double height1,double height2)	{
	return CreateDoublePyramid(generateBase(numBaseEdges,baseRadius),height1,height2);
}
CPolyhedronPtr CPolyhedron::CreateArchimedeanRegularPrism(uint32_t numBaseEdges,double baseRadius)	{
	return CreateJohnsonSolidWithConstantBase(numBaseEdges,baseRadius,"PR");
}
CPolyhedronPtr CPolyhedron::CreateArchimedeanRegularAntiprism(uint32_t numBaseEdges,double baseRadius)	{
	return CreateJohnsonSolidWithConstantBase(numBaseEdges,baseRadius,"A");
}
CPolyhedronPtr CPolyhedron::CreateRegularTruncatedPyramid(uint32_t numBaseEdges,double baseRadius,double height,double ratio)	{
	return CreateTruncatedPyramid(generateBase(numBaseEdges,baseRadius),height,ratio);
}
CPolyhedronPtr CPolyhedron::CreateRegularFrustum(uint32_t numBaseEdges,double baseRadius,double height,double ratio)	{
	return CreateRegularTruncatedPyramid(numBaseEdges,baseRadius,height,ratio);
}
CPolyhedronPtr CPolyhedron::CreateRegularBifrustum(uint32_t numBaseEdges,double baseRadius,double height1,double ratio1,double height2,double ratio2)	{
	return CreateBifrustum(generateBase(numBaseEdges,baseRadius),height1,ratio1,height2,ratio2);
}
CPolyhedronPtr CPolyhedron::CreateCupola(uint32_t numBaseEdges,double edgeLength)	{
	return CreateJohnsonSolidWithConstantBase(numBaseEdges,edgeLength/(2*sin(M_PI/numBaseEdges)),"C+");
}
CPolyhedronPtr CPolyhedron::CreateCatalanTrapezohedron(uint32_t numBaseEdges,double height)	{
	return CreateArchimedeanRegularAntiprism(numBaseEdges,height)->getDual();
}
CPolyhedronPtr CPolyhedron::CreateCatalanDoublePyramid(uint32_t numBaseEdges,double height)	{
	return CreateArchimedeanRegularPrism(numBaseEdges,height)->getDual();
}

CPolyhedronPtr CPolyhedron::CreateNoCheck(const vector<TPoint3D> &vertices,const vector<TPolyhedronFace> &faces)	
{
	return CPolyhedronPtr(new CPolyhedron(vertices,faces,false));
}
CPolyhedronPtr CPolyhedron::CreateEmpty()	
{
	return CPolyhedronPtr(new CPolyhedron());
}
