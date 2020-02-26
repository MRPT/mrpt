/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// Windows:
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

// Include libraries in linking:
#if MRPT_HAS_OPENGL_GLUT && defined(_WIN32)
// WINDOWS:
#if defined(_MSC_VER)
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GlU32.lib")
#endif
#endif  // MRPT_HAS_OPENGL_GLUT

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CAngularObservationMesh, CRenderizable, mrpt::opengl)

void CAngularObservationMesh::addTriangle(
	const TPoint3D& p1, const TPoint3D& p2, const TPoint3D& p3) const
{
	const TPoint3D* arr[3] = {&p1, &p2, &p3};
	mrpt::opengl::TTriangle t;
	for (size_t i = 0; i < 3; i++)
	{
		t.x[i] = arr[i]->x;
		t.y[i] = arr[i]->y;
		t.z[i] = arr[i]->z;
		t.r[i] = m_color.R * (1.f / 255);
		t.g[i] = m_color.G * (1.f / 255);
		t.b[i] = m_color.B * (1.f / 255);
		t.a[i] = m_color.A * (1.f / 255);
	}
	triangles.push_back(t);
	CRenderizable::notifyChange();
}

void CAngularObservationMesh::updateMesh() const
{
	CRenderizable::notifyChange();

	size_t numRows = scanSet.size();
	triangles.clear();
	if (numRows <= 1)
	{
		actualMesh.setSize(0, 0);
		validityMatrix.setSize(0, 0);
		meshUpToDate = true;
		return;
	}
	if (pitchBounds.size() != numRows && pitchBounds.size() != 2) return;
	size_t numCols = scanSet[0].getScanSize();
	actualMesh.setSize(numRows, numCols);
	validityMatrix.setSize(numRows, numCols);
	std::vector<double> pitchs(numRows);
	if (pitchBounds.size() == 2)
	{
		double p1 = pitchBounds[0];
		double p2 = pitchBounds[1];
		for (size_t i = 0; i < numRows; i++)
			pitchs[i] = p1 + (p2 - p1) * static_cast<double>(i) /
								 static_cast<double>(numRows - 1);
	}
	else
		for (size_t i = 0; i < numRows; i++) pitchs[i] = pitchBounds[i];
	const bool rToL = scanSet[0].rightToLeft;
	for (size_t i = 0; i < numRows; i++)
	{
		const auto& ss_i = scanSet[i];
		const double pitchIncr = scanSet[i].deltaPitch;
		const double aperture = scanSet[i].aperture;
		const CPose3D origin = scanSet[i].sensorPose;
		// This is not an error...
		for (size_t j = 0; j < numCols; j++)
			if ((validityMatrix(i, j) = ss_i.getScanRangeValidity(j)))
			{
				double pYaw = aperture * ((static_cast<double>(j) /
										   static_cast<double>(numCols - 1)) -
										  0.5);
				// Without the pitch since it's already within each sensorPose:
				actualMesh(i, j) =
					((origin +
					  CPose3D(0, 0, 0, rToL ? pYaw : -pYaw, pitchIncr)) +
					 CPoint3D(ss_i.getScanRange(j), 0, 0))
						.asTPoint();
			}
	}

	triangles.reserve(2 * (numRows - 1) * (numCols - 1));
	for (size_t k = 0; k < numRows - 1; k++)
	{
		for (size_t j = 0; j < numCols - 1; j++)
		{
			int b1 = validityMatrix(k, j) ? 1 : 0;
			int b2 = validityMatrix(k, j + 1) ? 1 : 0;
			int b3 = validityMatrix(k + 1, j) ? 1 : 0;
			int b4 = validityMatrix(k + 1, j + 1) ? 1 : 0;
			switch (b1 + b2 + b3 + b4)
			{
				case 0:
				case 1:
				case 2:
					break;
				case 3:
					if (!b1)
						addTriangle(
							actualMesh(k, j + 1), actualMesh(k + 1, j),
							actualMesh(k + 1, j + 1));
					else if (!b2)
						addTriangle(
							actualMesh(k, j), actualMesh(k + 1, j),
							actualMesh(k + 1, j + 1));
					else if (!b3)
						addTriangle(
							actualMesh(k, j), actualMesh(k, j + 1),
							actualMesh(k + 1, j + 1));
					else if (!b4)
						addTriangle(
							actualMesh(k, j), actualMesh(k, j + 1),
							actualMesh(k + 1, j));
					break;
				case 4:
					addTriangle(
						actualMesh(k, j), actualMesh(k, j + 1),
						actualMesh(k + 1, j));
					addTriangle(
						actualMesh(k + 1, j + 1), actualMesh(k, j + 1),
						actualMesh(k + 1, j));
			}
		}
	}
	meshUpToDate = true;
}

void CAngularObservationMesh::renderUpdateBuffers() const
{
	MRPT_TODO("Implement!");
}

void CAngularObservationMesh::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT
	if (mEnableTransparency)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}
	if (!meshUpToDate) updateMesh();
	if (!m_Wireframe) glBegin(GL_TRIANGLES);
	for (const auto& t : triangles)
	{
		float ax = t.x[1] - t.x[0];
		float bx = t.x[2] - t.x[0];
		float ay = t.y[1] - t.y[0];
		float by = t.y[2] - t.y[0];
		float az = t.z[1] - t.z[0];
		float bz = t.z[2] - t.z[0];
		glNormal3f(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
		if (m_Wireframe) glBegin(GL_LINE_LOOP);
		for (int k = 0; k < 3; k++)
		{
			glColor4f(t.r[k], t.g[k], t.b[k], t.a[k]);
			glVertex3f(t.x[k], t.y[k], t.z[k]);
		}
		if (m_Wireframe) glEnd();
	}
	if (!m_Wireframe) glEnd();
#endif
}

bool CAngularObservationMesh::traceRay(
	const mrpt::poses::CPose3D& o, double& dist) const
{
	MRPT_UNUSED_PARAM(o);
	MRPT_UNUSED_PARAM(dist);
	// TODO: redo
	return false;
}

bool CAngularObservationMesh::setScanSet(
	const std::vector<mrpt::obs::CObservation2DRangeScan>& scans)
{
	CRenderizable::notifyChange();

	// Returns false if the scan is inconsistent
	if (scans.size() > 0)
	{
		size_t setSize = scans[0].getScanSize();
		bool rToL = scans[0].rightToLeft;
		for (auto it = scans.begin() + 1; it != scans.end(); ++it)
		{
			if (it->getScanSize() != setSize) return false;
			if (it->rightToLeft != rToL) return false;
		}
	}
	scanSet = scans;
	meshUpToDate = false;
	return true;
}

void CAngularObservationMesh::setPitchBounds(
	const double initial, const double final)
{
	CRenderizable::notifyChange();

	pitchBounds.clear();
	pitchBounds.push_back(initial);
	pitchBounds.push_back(final);
	meshUpToDate = false;
}
void CAngularObservationMesh::setPitchBounds(const std::vector<double>& bounds)
{
	CRenderizable::notifyChange();

	pitchBounds = bounds;
	meshUpToDate = false;
}
void CAngularObservationMesh::getPitchBounds(
	double& initial, double& final) const
{
	initial = pitchBounds.front();
	final = pitchBounds.back();
}
void CAngularObservationMesh::getPitchBounds(std::vector<double>& bounds) const
{
	bounds = pitchBounds;
}
void CAngularObservationMesh::getScanSet(
	std::vector<CObservation2DRangeScan>& scans) const
{
	scans = scanSet;
}

void CAngularObservationMesh::generateSetOfTriangles(
	CSetOfTriangles::Ptr& res) const
{
	if (!meshUpToDate) updateMesh();
	res->insertTriangles(triangles.begin(), triangles.end());
	// for (vector<mrpt::opengl::TTriangle>::iterator
	// it=triangles.begin();it!=triangles.end();++it) res->insertTriangle(*it);
}

struct CAngularObservationMesh_fnctr
{
	CPointsMap* m;
	CAngularObservationMesh_fnctr(CPointsMap* p) : m(p) {}
	inline void operator()(const CObservation2DRangeScan& obj)
	{
		m->insertObservation(obj);
	}
};

void CAngularObservationMesh::generatePointCloud(CPointsMap* out_map) const
{
	ASSERT_(out_map);
	out_map->clear();
	/*	size_t numRows=scanSet.size();
		if ((pitchBounds.size()!=numRows)&&(pitchBounds.size()!=2)) return;
		std::vector<double> pitchs(numRows);
		if (pitchBounds.size()==2)	{
			double p1=pitchBounds[0];
			double p2=pitchBounds[1];
			for (size_t i=0;i<numRows;i++)
	   pitchs[i]=p1+(p2-p1)*static_cast<double>(i)/static_cast<double>(numRows-1);
		}	else for (size_t i=0;i<numRows;i++) pitchs[i]=pitchBounds[i];
		for (size_t i=0;i<numRows;i++) out_map->insertObservation(&scanSet[i]);
	*/

	std::for_each(
		scanSet.begin(), scanSet.end(), CAngularObservationMesh_fnctr(out_map));
}

uint8_t CAngularObservationMesh::serializeGetVersion() const { return 0; }
void CAngularObservationMesh::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// Version 0:
	out << pitchBounds << scanSet << m_Wireframe << mEnableTransparency;
}

void CAngularObservationMesh::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			readFromStreamRender(in);
			in >> pitchBounds >> scanSet >> m_Wireframe >> mEnableTransparency;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	meshUpToDate = false;
}

void CAngularObservationMesh::TDoubleRange::values(
	std::vector<double>& vals) const
{
	double value = initialValue();
	double incr = increment();
	size_t am = amount();
	vals.resize(am);
	for (size_t i = 0; i < am - 1; i++, value += incr) vals[i] = value;
	vals[am - 1] = finalValue();
}

void CAngularObservationMesh::getTracedRays(CSetOfLines::Ptr& res) const
{
	if (!meshUpToDate) updateMesh();
	size_t count = 0;
	for (int i = 0; i < validityMatrix.rows(); i++)
		for (int j = 0; j < validityMatrix.cols(); j++)
			if (validityMatrix(i, j)) count++;
	res->reserve(count);
	for (int i = 0; i < actualMesh.rows(); i++)
		for (int j = 0; j < actualMesh.cols(); j++)
			if (validityMatrix(i, j))
				res->appendLine(
					(scanSet[i].sensorPose).asTPose(), actualMesh(i, j));
}

class FAddUntracedLines
{
   public:
	CSetOfLines::Ptr& lins;
	const CPoint3D& pDist;
	std::vector<double> pitchs;
	FAddUntracedLines(
		CSetOfLines::Ptr& l, const CPoint3D& p, const std::vector<double>& pi)
		: lins(l), pDist(p), pitchs()
	{
		pitchs.reserve(pi.size());
		for (auto it = pi.rbegin(); it != pi.rend(); ++it)
			pitchs.push_back(*it);
	}
	void operator()(const CObservation2DRangeScan& obs)
	{
		size_t hm = obs.getScanSize();
		for (size_t i = 0; i < obs.getScanSize(); i++)
			if (obs.getScanRangeValidity(i)) hm--;
		lins->reserve(hm);
		for (size_t i = 0; i < obs.getScanSize(); i++)
			if (!obs.getScanRangeValidity(i))
			{
				double yaw = obs.aperture *
							 ((static_cast<double>(i) /
							   static_cast<double>(obs.getScanSize() - 1)) -
							  0.5);
				lins->appendLine(
					obs.sensorPose.asTPose(),
					(obs.sensorPose +
					 CPose3D(
						 0, 0, 0, obs.rightToLeft ? yaw : -yaw,
						 obs.deltaPitch * i + pitchs.back(), 0) +
					 pDist)
						.asTPoint());
			}
		pitchs.pop_back();
	}
};
void CAngularObservationMesh::getUntracedRays(
	CSetOfLines::Ptr& res, double dist) const
{
	for_each(
		scanSet.begin(), scanSet.end(),
		FAddUntracedLines(res, dist, pitchBounds));
}

TPolygon3D createFromTriangle(const mrpt::opengl::TTriangle& t)
{
	TPolygon3D res(3);
	for (size_t i = 0; i < 3; i++)
	{
		res[i].x = t.x[i];
		res[i].y = t.y[i];
		res[i].z = t.z[i];
	}
	return res;
}

void CAngularObservationMesh::generateSetOfTriangles(
	std::vector<TPolygon3D>& res) const
{
	if (!meshUpToDate) updateMesh();
	res.resize(triangles.size());
	transform(
		triangles.begin(), triangles.end(), res.begin(), createFromTriangle);
}

void CAngularObservationMesh::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	if (!meshUpToDate) updateMesh();

	bb_min = mrpt::math::TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& t : triangles)
	{
		keep_min(bb_min.x, t.x[0]);
		keep_max(bb_max.x, t.x[0]);
		keep_min(bb_min.y, t.y[0]);
		keep_max(bb_max.y, t.y[0]);
		keep_min(bb_min.z, t.z[0]);
		keep_max(bb_max.z, t.z[0]);

		keep_min(bb_min.x, t.x[1]);
		keep_max(bb_max.x, t.x[1]);
		keep_min(bb_min.y, t.y[1]);
		keep_max(bb_max.y, t.y[1]);
		keep_min(bb_min.z, t.z[1]);
		keep_max(bb_max.z, t.z[1]);

		keep_min(bb_min.x, t.x[2]);
		keep_max(bb_max.x, t.x[2]);
		keep_min(bb_min.y, t.y[2]);
		keep_max(bb_max.y, t.y[2]);
		keep_min(bb_min.z, t.z[2]);
		keep_max(bb_max.z, t.z[2]);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
