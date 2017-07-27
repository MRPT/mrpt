/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CViewerContainer.h"
#include "ui_CViewerContainer.h"
#include "CGLWidget.h"
#include "gui/observationTree/CRangeScanNode.h"

CViewerContainer::CViewerContainer(QWidget* parent)
	: QWidget(parent), m_ui(std::make_unique<Ui::CViewerContainer>())
{
	m_ui->setupUi(this);

	QObject::connect(
		m_ui->m_zoom, SIGNAL(valueChanged(double)), SLOT(zoomChanged(double)));
	QObject::connect(
		m_ui->m_zoomSlider, SIGNAL(valueChanged(int)), SLOT(zoomChanged(int)));

	QObject::connect(
		m_ui->m_tabWidget, SIGNAL(currentChanged(int)),
		SLOT(updatePanelInfo(int)));

	QObject::connect(
		m_ui->m_azimDeg, SIGNAL(valueChanged(double)),
		SLOT(updateAzimuthDegrees(double)));
	QObject::connect(
		m_ui->m_elevationDeg, SIGNAL(valueChanged(double)),
		SLOT(updateElevationDegrees(double)));
}

CViewerContainer::~CViewerContainer()
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		delete m_ui->m_tabWidget->widget(i);
	}
}

void CViewerContainer::showRangeScan(CNode* node)
{
	CRangeScanNode* obsNode = dynamic_cast<CRangeScanNode*>(node);
	assert(obsNode);

	mrpt::opengl::CPlanarLaserScan::Ptr obj =
		mrpt::opengl::CPlanarLaserScan::Create();
	obj->setScan(*(obsNode->observation().get()));
	obj->setPose(obsNode->getPose());
	obj->setSurfaceColor(1.0f, 0.0f, 0.0f, 0.5f);

	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget* w = m_ui->m_tabWidget->widget(i);
		CGlWidget* gl = dynamic_cast<CGlWidget*>(w);
		assert(gl);
		gl->setSelected(obsNode->getPose());
		gl->setLaserScan(obj);
	}
}

void CViewerContainer::showRobotDirection(const mrpt::poses::CPose3D& pose)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget* w = m_ui->m_tabWidget->widget(i);
		CGlWidget* gl = dynamic_cast<CGlWidget*>(w);
		assert(gl);
		gl->setSelected(pose);
	}
}

void CViewerContainer::applyConfigChanges(RenderizableMaps renderizableMaps)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		auto sTypeIter = m_tabsInfo.find(i);
		assert(sTypeIter != m_tabsInfo.end());

		auto it = renderizableMaps.find(sTypeIter->second);
		assert(it != renderizableMaps.end());

		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);

		gl->fillMap(it->second);
	}
}

void CViewerContainer::updateConfigChanges(
	RenderizableMaps renderizableMaps, CDocument* doc, bool isShowAllObs)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget* w = m_ui->m_tabWidget->widget(i);
		delete w;
	}
	m_tabsInfo.clear();

	m_ui->m_tabWidget->clear();

	for (auto& it : renderizableMaps)
	{
		bool is2D = it.first.type == TypeOfConfig::PointsMap ||
					it.first.type == TypeOfConfig::Occupancy;
		CGlWidget* gl = new CGlWidget(is2D);
		gl->fillMap(it.second);

		QString name = QString::fromStdString(typeToName(it.first.type)) +
					   QString::number(it.first.index);
		int tabIndex = m_ui->m_tabWidget->addTab(gl, name);
		m_tabsInfo.emplace(tabIndex, it.first);

		gl->setDocument(doc);
		gl->setSelectedObservation(isShowAllObs);
		QObject::connect(
			gl, SIGNAL(zoomChanged(float)), SLOT(changeZoomInfo(float)));
		QObject::connect(
			gl, SIGNAL(mousePosChanged(double, double)),
			SLOT(updateMouseInfo(double, double)));
		QObject::connect(
			gl, SIGNAL(azimuthChanged(float)), SLOT(changeAzimuthDeg(float)));
		QObject::connect(
			gl, SIGNAL(elevationChanged(float)),
			SLOT(changeElevationDeg(float)));
	}
}

void CViewerContainer::setDocument(CDocument* doc)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);

		gl->setDocument(doc);
	}
}

void CViewerContainer::showAllObservation(bool is)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);
		gl->setSelectedObservation(is);
	}
}

void CViewerContainer::changeCurrentBot(int value)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);
		gl->setBot(value);
	}
}

void CViewerContainer::setVisibleGrid(bool is)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);
		gl->setVisibleGrid(is);
	}
}

void CViewerContainer::changeBackgroundColor(QColor color)
{
	float r = color.red() / 255.0;
	float g = color.green() / 255.0;
	float b = color.blue() / 255.0;
	float a = color.alpha() / 255.0;
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);
		gl->setBackgroundColor(r, g, b, a);
	}
}

void CViewerContainer::changeGridColor(QColor color)
{
	float r = color.red() / 255.0;
	float g = color.green() / 255.0;
	float b = color.blue() / 255.0;
	float a = color.alpha() / 255.0;
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);
		gl->setGridColor(r, g, b, a);
	}
}

void CViewerContainer::updatePanelInfo(int index)
{
	if (m_ui->m_tabWidget->count())
	{
		CGlWidget* gl =
			dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(index));
		assert(gl);

		changeZoomInfo(gl->getZoom());
		changeAzimuthDeg(gl->getAzimuthDegrees());
		changeElevationDeg(gl->getElevationDegrees());
	}
}

void CViewerContainer::changeZoomInfo(float zoom)
{
	int zoomInt = zoom;
	m_ui->m_zoomSlider->blockSignals(true);
	m_ui->m_zoomSlider->setValue(zoomInt);
	m_ui->m_zoomSlider->blockSignals(false);

	double zoomDouble = zoom;
	m_ui->m_zoom->blockSignals(true);
	m_ui->m_zoom->setValue(zoomDouble);
	m_ui->m_zoom->blockSignals(false);
}

void CViewerContainer::changeAzimuthDeg(float deg)
{
	m_ui->m_azimDeg->blockSignals(true);
	m_ui->m_azimDeg->setValue(deg);
	m_ui->m_azimDeg->blockSignals(false);
}

void CViewerContainer::changeElevationDeg(float deg)
{
	m_ui->m_elevationDeg->blockSignals(true);
	m_ui->m_elevationDeg->setValue(deg);
	m_ui->m_elevationDeg->blockSignals(false);
}

void CViewerContainer::zoomChanged(double d)
{
	float zoomFloat = d;
	int zoomInt = zoomFloat;
	m_ui->m_zoomSlider->setValue(zoomInt);
	CGlWidget* gl =
		dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->currentWidget());
	assert(gl);
	gl->setZoom(zoomFloat);
}

void CViewerContainer::zoomChanged(int d)
{
	double zoomD = d;
	float zoomFloat = zoomD;
	m_ui->m_zoom->setValue(zoomD);
	CGlWidget* gl =
		dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->currentWidget());
	assert(gl);
	gl->setZoom(zoomFloat);
}

void CViewerContainer::updateMouseInfo(double x, double y)
{
	QString str = "x: " + QString::number(x) + "; y: " + QString::number(y);
	m_ui->m_infoUnderCursor->setText(str);
}

void CViewerContainer::updateAzimuthDegrees(double deg)
{
	CGlWidget* gl = getCurrentTabWidget();
	if (!gl) return;

	gl->blockSignals(true);
	gl->setAzimuthDegrees(deg);
	gl->blockSignals(false);
	gl->update();
}

void CViewerContainer::updateElevationDegrees(double deg)
{
	CGlWidget* gl = getCurrentTabWidget();
	if (!gl) return;

	gl->blockSignals(true);
	gl->setElevationDegrees(deg);
	gl->blockSignals(false);
	gl->update();
}

void CViewerContainer::setGeneralSetting(const SGeneralSetting& setting)
{
	float rBack = setting.backgroundColor.red() / 255.0;
	float gBack = setting.backgroundColor.green() / 255.0;
	float bBack = setting.backgroundColor.blue() / 255.0;
	float aBack = setting.backgroundColor.alpha() / 255.0;
	float r = setting.gridColor.red() / 255.0;
	float g = setting.gridColor.green() / 255.0;
	float b = setting.gridColor.blue() / 255.0;
	float a = setting.gridColor.alpha() / 255.0;

	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		assert(gl);

		gl->setGridColor(r, g, b, a);
		gl->setVisibleGrid(setting.isGridVisibleChanged);
		gl->setBackgroundColor(rBack, gBack, bBack, aBack);
		gl->setBot(setting.currentBot);

		gl->setObservationSize(setting.robotPosesSize);
		gl->setSelectedObservationSize(setting.selectedRobotPosesSize);
		gl->setObservationColor(setting.robotPosesColor);
		gl->setSelectedObservationColor(setting.selectedRobotPosesColor);
	}

	CGlWidget* gl = getCurrentTabWidget();
	if (gl) gl->update();
}

CGlWidget* CViewerContainer::getCurrentTabWidget() const
{
	if (m_ui->m_tabWidget->count() <= 0) return nullptr;

	CGlWidget* gl =
		dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->currentWidget());
	assert(gl);

	return gl;
}
