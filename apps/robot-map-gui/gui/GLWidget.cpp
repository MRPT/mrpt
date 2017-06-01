#include "GLWidget.h"

#include "mrpt/maps/TMetricMapInitializer.h"
#include "mrpt/utils/CConfigFile.h"
#include "mrpt/utils/CFileGZOutputStream.h"
#include "mrpt/utils/CFileGZInputStream.h"

#include "cmath"


void qgluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
	const GLdouble ymax = zNear * tan(fovy * M_PI / 360.0);
	const GLdouble ymin = -ymax;
	const GLdouble xmin = ymin * aspect;
	const GLdouble xmax = ymax * aspect;
	glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
}

std::string METRIC_MAP_CONFIG_SECTION  =  "MappingApplication";

GlWidget::GlWidget(QWidget *parent)
	: QGLWidget(parent)
	, m_obj(mrpt::opengl::CSetOfObjects::Create())
{
}

void GlWidget::loadFile(const QString &file_name, const QString &config)
{
	m_simplemap = mrpt::maps::CSimpleMap();
	mrpt::utils::CFileGZInputStream file( file_name.toStdString().c_str());

	file >> m_simplemap;

	m_metricmap = mrpt::maps::CMultiMetricMap();

	mrpt::maps::TSetOfMetricMapInitializers		mapCfg;
	mapCfg.loadFromConfigFile( mrpt::utils::CConfigFile(config.toStdString()), METRIC_MAP_CONFIG_SECTION);

	m_metricmap.setListOfMaps( &mapCfg );
	m_metricmap.loadFromProbabilisticPosesAndObservations(m_simplemap);

	mrpt::maps::CSimplePointsMap::Ptr prt = m_metricmap.getMapByClass<mrpt::maps::CSimplePointsMap>();
	prt->getAs3DObject(m_obj);

	update();
}

void GlWidget::initializeGL()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
}

void GlWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glTranslatef(-1.5f,0.0f,-6.0f);
	if (m_obj.get())
	{
		m_obj->render();
	}
}

void GlWidget::resizeGL(int width, int height)
{
	if (height==0) height=1;
	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	qgluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
