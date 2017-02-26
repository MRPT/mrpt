/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/threads.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CAngularObservationMesh.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using mrpt::opengl::CAngularObservationMesh;
using namespace stlplus;
using namespace mrpt::system;

const double GRID_R=1.0;
const double GRID_G=1.0;
const double GRID_B=1.0;

//Comment to increase randomness of poses
#define PAIRED_RANDOM_POSES

#define USE_THREADS

inline double MYRAND1(size_t prec=64)	{
	return static_cast<double>(rand()%prec)/static_cast<double>(prec-1);
}

void randomColor(CRenderizablePtr &obj,double alpha)	{
	obj->setColor(MYRAND1(),MYRAND1(),MYRAND1(),alpha);
}

#ifdef USE_THREADS
class PIThreadParam	{
public:
	const pair<CPolyhedronPtr,CPolyhedronPtr> *polys;
	vector<TSegment3D> intersection;
	PIThreadParam(const pair<CPolyhedronPtr,CPolyhedronPtr> &p):polys(&p),intersection()	{}
	PIThreadParam():polys(NULL),intersection()	{}
	inline static PIThreadParam createObject(const pair<CPolyhedronPtr,CPolyhedronPtr> &p)	{
		return PIThreadParam(p);
	}
};

void piThreadFunction(PIThreadParam &p)	{
	vector<TObject3D> ints;
	CPolyhedron::getIntersection(p.polys->first,p.polys->second,ints);
	TObject3D::getSegments(ints,p.intersection);
}

inline TThreadHandle piCreateThread(PIThreadParam &p)	{
	return createThread<PIThreadParam &>(&piThreadFunction,p);
}

class AggregatorFunctor	{
public:
	vector<TSegment3D> &sgms;
	AggregatorFunctor(vector<TSegment3D> &s):sgms(s)	{}
	inline void operator()(const PIThreadParam &p)	{
		sgms.insert(sgms.end(),p.intersection.begin(),p.intersection.end());
	}
};
#endif

CSetOfLinesPtr getIntersections(const vector<pair<CPolyhedronPtr,CPolyhedronPtr> > &v)	{
	vector<TSegment3D> sgms;
	#ifdef USE_THREADS
		vector<PIThreadParam> pars(v.size());
		vector<TThreadHandle> threads(v.size());
		transform(v.begin(),v.end(),pars.begin(),&PIThreadParam::createObject);
		transform(pars.begin(),pars.end(),threads.begin(),&piCreateThread);
		for_each(threads.begin(),threads.end(),&joinThread);
		for_each(pars.begin(),pars.end(),AggregatorFunctor(sgms));
	#else
		vector<TObject3D> ints,TMP;
		for (vector<pair<CPolyhedronPtr,CPolyhedronPtr> >::const_iterator it=v.begin();it!=v.end();++it)	{
			CPolyhedron::getIntersection(it->first,it->second,TMP);
			ints.insert(ints.end(),TMP.begin(),TMP.end());
		}
		TObject3D::getSegments(ints,sgms);
	#endif
	CSetOfLinesPtr lns=CSetOfLines::Create(sgms);
	lns->setLineWidth(9);
	randomColor(lns,1.0);
	return lns;
}

inline double randomAngle(size_t prec=64)	{
	return MYRAND1(prec)*M_PI;
}

inline double randomZ(double space=25,size_t prec=64)	{
	return space*(MYRAND1(prec)-0.5);
}

pair<CPolyhedronPtr,CPolyhedronPtr> addPairOfPolys(CPolyhedronPtr p1,CPolyhedronPtr p2,CSetOfObjectsPtr &objs,double x,double y)	{
	p1->makeConvexPolygons();
	p2->makeConvexPolygons();
#ifdef PAIRED_RANDOM_POSES
	CPose3D pose=CPose3D(x,y,randomZ(),randomAngle(),randomAngle(),randomAngle());
	p1->setPose(pose);
	p2->setPose(pose);
#else
	CPose3D pose1=CPose3D(x,y,randomZ(),randomAngle(),randomAngle(),randomAngle());
	CPose3D pose2=CPose3D(x,y,randomZ(),randomAngle(),randomAngle(),randomAngle());
	p1->setPose(pose1);
	p2->setPose(pose2);
#endif
	randomColor(p1,0.5);
	randomColor(p2,0.5);
	objs->insert(p1);
	objs->insert(p2);
	return make_pair(p1,p2);
}

void display()	{
	CDisplayWindow3D window("Polyhedra Intersection demo",640,480);
	window.resize(640,480);
	COpenGLScenePtr scene1=COpenGLScene::Create();
	opengl::CGridPlaneXYPtr plane1=CGridPlaneXY::Create(-25,25,-25,25,0,1);
	plane1->setColor(GRID_R,GRID_G,GRID_B);
	scene1->insert(plane1);
	scene1->insert(CAxis::Create(-5,-5,-5,5,5,5,2.5,3,true));
	CSetOfObjectsPtr objs=CSetOfObjects::Create();
	vector<pair<CPolyhedronPtr,CPolyhedronPtr> > polys;
	polys.reserve(16);
	//Addition of polyhedra. Add more polyhedra at wish, but try to avoid intersections with other pairs, for better visualization.
	polys.push_back(addPairOfPolys(CPolyhedron::CreateHexahedron(10),CPolyhedron::CreateOctahedron(10),objs,-12.5,-12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateIcosahedron(10),CPolyhedron::CreateDodecahedron(10),objs,-12.5,12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateRhombicuboctahedron(10),CPolyhedron::CreateCuboctahedron(10),objs,12.5,12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateArchimedeanRegularAntiprism(4,9),CPolyhedron::CreateRegularDoublePyramid(9,10,15,6),objs,12.5,-12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateCuboctahedron(10),CPolyhedron::CreateRhombicDodecahedron(10),objs,-37.5,-37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateRhombicuboctahedron(10),CPolyhedron::CreateDeltoidalIcositetrahedron(10),objs,-37.5,-12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateIcosidodecahedron(10),CPolyhedron::CreateRhombicTriacontahedron(10),objs,-37.5,12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateRhombicosidodecahedron(10),CPolyhedron::CreateDeltoidalHexecontahedron(10),objs,-37.5,37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateTruncatedTetrahedron(10),CPolyhedron::CreateTriakisTetrahedron(10),objs,-12.5,-37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateTruncatedHexahedron(10),CPolyhedron::CreateTriakisOctahedron(10),objs,-12.5,37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateTruncatedOctahedron(10),CPolyhedron::CreateTetrakisHexahedron(10),objs,12.5,-37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateTruncatedDodecahedron(10),CPolyhedron::CreateTriakisIcosahedron(10),objs,12.5,37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateTruncatedIcosahedron(10),CPolyhedron::CreatePentakisDodecahedron(10),objs,37.5,-37.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateRandomPolyhedron(10),CPolyhedron::CreateRandomPolyhedron(10),objs,37.5,-12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateDodecahedron(10),CPolyhedron::CreateDeltoidalHexecontahedron(10),objs,37.5,12.5));
	polys.push_back(addPairOfPolys(CPolyhedron::CreateTriakisIcosahedron(10),CPolyhedron::CreatePentakisDodecahedron(10),objs,37.5,37.5));
	objs<<getIntersections(polys);

	scene1->insert(objs);
	window.get3DSceneAndLock()=scene1;
	window.unlockAccess3DScene();
	window.setCameraElevationDeg(25.0f);
	window.forceRepaint();
	window.waitForKey();
}

int main()	{
	srand((unsigned int)mrpt::system::extractDayTimeFromTimestamp(mrpt::system::getCurrentLocalTime()));
	try	{
		display();
		return 0;
	}	catch (exception &e)	{
		cout<<"Error: "<<e.what()<<'.'<<endl;
		return -1;
	}	catch (...)	{
		cout<<"Unknown Error.\n";
		return -1;
	}
}
