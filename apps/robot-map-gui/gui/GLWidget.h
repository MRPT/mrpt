#pragma once
#include <QGLWidget>

#include "mrpt/opengl/CSetOfObjects.h"
#include "mrpt/maps/CSimpleMap.h"
#include "mrpt/maps/CMultiMetricMap.h"


class GlWidget : public QGLWidget
{
public:
	GlWidget(QWidget* parent=nullptr);
	void loadFile(const QString &file_name, const QString &config);

public:
	virtual void initializeGL() override;
	virtual void paintGL() override;
	virtual void resizeGL(int width, int height) override;

	mrpt::maps::CSimpleMap m_simplemap;
	mrpt::maps::CMultiMetricMap m_metricmap;
	mrpt::opengl::CSetOfObjects::Ptr m_obj;
};
