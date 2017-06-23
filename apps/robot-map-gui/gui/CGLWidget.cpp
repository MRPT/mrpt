#include "CGLWidget.h"

#include "mrpt/maps/TMetricMapInitializer.h"
#include "mrpt/utils/CConfigFile.h"
#include "mrpt/utils/CFileGZOutputStream.h"
#include "mrpt/utils/CFileGZInputStream.h"
#include "mrpt/opengl/CGridPlaneXY.h"
#include "mrpt/opengl/CPointCloud.h"

#include "cmath"

#include <QMouseEvent>
#include <QDebug>


using namespace mrpt;
using namespace mrpt::opengl;
void qgluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
	const GLdouble ymax = zNear * tan(fovy * M_PI / 360.0);
	const GLdouble ymin = -ymax;
	const GLdouble xmin = ymin * aspect;
	const GLdouble xmax = ymax * aspect;
	glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
}

std::string METRIC_MAP_CONFIG_SECTION  =  "MappingApplication";

CGlWidget::CGlWidget(QWidget *parent)
	: QGLWidget(parent)
	, m_3Dscene(COpenGLScene::Create())
	, m_previsionPos(QPoint(0, 0))
	, m_isPressMouse(false)
{
	COpenGLViewport::Ptr view = m_3Dscene->getViewport("main");
	ASSERT_(view);

	setMouseTracking(true);
	// The ground:
	mrpt::opengl::CGridPlaneXY::Ptr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
	groundPlane->setColor(0.4,0.4,0.4);
	view->insert( groundPlane );

	// The camera pointing to the current robot pose:
	{
		CCamera &cam = view->getCamera();
		cam.setAzimuthDegrees(-90);
		cam.setElevationDegrees(90);
		cam.setZoomDistance(40);
		cam.setOrthogonal(false);
	}




}

void CGlWidget::fillMap(const CSetOfObjects::Ptr &renderizableMap)
{
	COpenGLViewport::Ptr view = m_3Dscene->getViewport("main");
	ASSERT_(view);
	view->insert(renderizableMap);

	m_map = renderizableMap;

	update();
}

void CGlWidget::setSelected(const math::TPose3D &pose)
{
	CPointCloud::Ptr points = CPointCloud::Create();
	points->insertPoint(pose.x, pose.y, pose.z);
	points->setColor(mrpt::utils::TColorf(mrpt::utils::TColor::red));
	points->setPointSize(10.);
	m_map->insert(points);
	update();
	/*
	for (auto it = m_map->begin(); it != m_map->end(); ++it)
	{
		CPointCloud *points = dynamic_cast<CPointCloud *>(it->get());
		if (points)
		{
			for (int i = 0; i < points->size(); ++i)
			{
				if (points->getPoint(i) == mrpt::math::TPoint3D(pose.x, pose.y, pose.z))
				{
					(*it)->setColorR(1);
					(*it)->setColorG(0);
				}
				else
				{
					(*it)->setColorR(0);
					(*it)->setColorG(1);
				}
			}
		}

	}*/
}

void CGlWidget::initializeGL()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
}

void CGlWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glTranslatef(-1.5f,0.0f,-6.0f);
	if (m_3Dscene.get())
	{
		m_3Dscene->render();
	}
}

void CGlWidget::resizeGL(int width, int height)
{
	if (height==0) height=1;
	glViewport(0,0,width,height);
}
void CGlWidget::mousePressEvent(QMouseEvent *event)
{
	qDebug() << "press "<< Qt::MouseButton(event->button());
	if (event->button() == Qt::LeftButton)
	{
		m_isPressMouse = true;
	}
	m_previsionPos = event->pos();
	QGLWidget::mousePressEvent(event);
}

void CGlWidget::mouseMoveEvent(QMouseEvent *event)
{
	qDebug() << Qt::MouseButton(event->button());
	if (m_isPressMouse)
	{
		COpenGLViewport::Ptr view = m_3Dscene->getViewport("main");
		ASSERT_(view);

		CCamera &cam = view->getCamera();
		mrpt::math::TPoint3D point(
					cam.getPointingAtX() + m_previsionPos.x() - event->pos().x(),
					cam.getPointingAtY() - m_previsionPos.y() + event->pos().y(),
					cam.getPointingAtZ()
					);
		cam.setPointingAt(point);

		m_previsionPos = event->pos();
		update();
	}

	QGLWidget::mouseMoveEvent(event);
}

void CGlWidget::mouseReleaseEvent(QMouseEvent *event)
{
	m_isPressMouse = false;
	QGLWidget::mouseReleaseEvent(event);
}

void CGlWidget::wheelEvent(QWheelEvent *event)
{
	COpenGLViewport::Ptr view = m_3Dscene->getViewport("main");
	ASSERT_(view);
	CCamera &cam = view->getCamera();

	float zoom = cam.getZoomDistance() - (event->delta() / 60);
	cam.setZoomDistance(zoom);
	update();
	QGLWidget::wheelEvent(event);
}
