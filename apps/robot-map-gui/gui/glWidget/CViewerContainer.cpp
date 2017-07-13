/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CViewerContainer.h"
#include "ui_CViewerContainer.h"
#include "CGLWidget.h"
#include "gui/observationTree/CRangeScanNode.h"
#include "gui/observationTree/CPosesNode.h"


CViewerContainer::CViewerContainer(QWidget *parent)
	: QWidget(parent)
	, m_ui(std::make_unique<Ui::CViewerContainer>())
{
	m_ui->setupUi(this);

	QObject::connect(m_ui->m_zoom, SIGNAL(valueChanged(double)), SLOT(zoomChanged(double)));
	QObject::connect(m_ui->m_zoomSlider, SIGNAL(valueChanged(int)), SLOT(zoomChanged(int)));

	QObject::connect(m_ui->m_tabWidget, SIGNAL(currentChanged(int)), SLOT(updateZoomInfo(int)));
}

void CViewerContainer::showRangeScan(CNode *node)
{
	CRangeScanNode *obsNode = dynamic_cast<CRangeScanNode *>(node);
	assert(obsNode);

	mrpt::opengl::CPlanarLaserScan::Ptr obj = mrpt::opengl::CPlanarLaserScan::Create();
	obj->setScan(*(obsNode->observation().get()));
	obj->setPose( obsNode->getPose() );
	obj->setSurfaceColor(1.0f,0.0f,0.0f, 0.5f);

	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget *w = m_ui->m_tabWidget->widget(i);
		CGlWidget *gl = dynamic_cast<CGlWidget *>(w);
		assert(gl);
		gl->setSelected(obsNode->getPose());
		gl->setLaserScan(obj);
	}
}

void CViewerContainer::showRobotDirection(CNode *node)
{
	CPosesNode *posesNode = dynamic_cast<CPosesNode *>(node);
	assert(posesNode);

	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget *w = m_ui->m_tabWidget->widget(i);
		CGlWidget *gl = dynamic_cast<CGlWidget *>(w);
		assert(gl);
		gl->setSelected(posesNode->getPose());
	}
}

void CViewerContainer::applyConfigChanges(RenderizableMaps renderizableMaps)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		std::string name = m_ui->m_tabWidget->tabText(i).toStdString();
		auto it = renderizableMaps.find(name);
		assert(it != renderizableMaps.end());

		CGlWidget *gl = dynamic_cast<CGlWidget *>(m_ui->m_tabWidget->widget(i));
		assert(gl);

		gl->fillMap(it->second);
	}
}

void CViewerContainer::updateConfigChanges(RenderizableMaps renderizableMaps, CDocument *doc, bool isShowAllObs)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget *w = m_ui->m_tabWidget->widget(i);
		delete w;
	}

	m_ui->m_tabWidget->clear();

	for(auto &it: renderizableMaps)
	{
		CGlWidget *gl = new CGlWidget();
		gl->fillMap(it.second);
		m_ui->m_tabWidget->addTab(gl, QString::fromStdString(it.first));

		gl->setDocument(doc);
		gl->setSelectedObservation(isShowAllObs);
		QObject::connect(gl, SIGNAL(zoomChanged(float)), SLOT(changeZoomInfo(float)));
	}
}

void CViewerContainer::setDocument(CDocument *doc)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget *gl = dynamic_cast<CGlWidget *>(m_ui->m_tabWidget->widget(i));
		assert(gl);

		gl->setDocument(doc);
	}
}

void CViewerContainer::showAllObservation(bool is)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget *gl = dynamic_cast<CGlWidget *>(m_ui->m_tabWidget->widget(i));
		assert(gl);
		gl->setSelectedObservation(is);
	}
}

void CViewerContainer::updateZoomInfo(int index)
{
	if (m_ui->m_tabWidget->count())
	{
		CGlWidget *gl = dynamic_cast<CGlWidget *>(m_ui->m_tabWidget->widget(index));
		assert(gl);

		changeZoomInfo(gl->getZoom());
	}
}

void CViewerContainer::changeZoomInfo(float zoom)
{
	int zoomInt = zoom;
	m_ui->m_zoomSlider->setValue(zoomInt);

	double zoomDouble = zoom;
	m_ui->m_zoom->setValue(zoomDouble);
}

void CViewerContainer::zoomChanged(double d)
{
	float zoomFloat = d;
	int zoomInt = zoomFloat;
	m_ui->m_zoomSlider->setValue(zoomInt);
	CGlWidget *gl = dynamic_cast<CGlWidget *>(m_ui->m_tabWidget->currentWidget());
	assert(gl);
	gl->setZoom(zoomFloat);
}

void CViewerContainer::zoomChanged(int d)
{
	double zoomD = d;
	float zoomFloat = zoomD;
	m_ui->m_zoom->setValue(zoomD);
	CGlWidget *gl = dynamic_cast<CGlWidget *>(m_ui->m_tabWidget->currentWidget());
	assert(gl);
	gl->setZoom(zoomFloat);
}
