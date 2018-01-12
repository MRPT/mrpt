/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CViewerContainer.h"
#include "ui_CViewerContainer.h"
#include "CGLWidget.h"
#include "gui/observationTree/CRangeScanNode.h"

#include <QTextEdit>
#include <QDebug>

CViewerContainer::CViewerContainer(QWidget* parent)
	: QWidget(parent),
	  m_ui(std::make_unique<Ui::CViewerContainer>()),
	  m_information(new QTextEdit(this)),
	  m_helpLoadMap(
		  tr("For loading the map, press File -> Open simplemap or Ctrl + "
			 "O.\n\n\n")),
	  m_helpLoadConfig(
		  tr("For display any maps you must download the configuration file. "
			 "For this press File -> Load config or Ctrl + E.\n\n\n"
			 "If you loaded simplemap, you can yourself add the map from "
			 "Configuration widget"))
{
	m_ui->setupUi(this);
	connect(
		m_ui->m_zoom, SIGNAL(valueChanged(double)), SLOT(zoomChanged(double)));
	connect(
		m_ui->m_zoomSlider, SIGNAL(valueChanged(int)), SLOT(zoomChanged(int)));

	connect(
		m_ui->m_azimDeg, SIGNAL(valueChanged(double)),
		SLOT(updateAzimuthDegrees(double)));
	connect(
		m_ui->m_elevationDeg, SIGNAL(valueChanged(double)),
		SLOT(updateElevationDegrees(double)));

	connect(
		m_ui->m_tabWidget, &QTabWidget::currentChanged, this,
		&CViewerContainer::updatePanelInfo);

	m_information->setReadOnly(true);
	m_information->setText(m_helpLoadMap);

	m_ui->m_tabWidget->addTab(m_information, "Welcome!");
}

CViewerContainer::~CViewerContainer()
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		delete m_ui->m_tabWidget->widget(i);
	}
}

void CViewerContainer::changeHelpTextToAboutConfig()
{
	m_information->setText(m_helpLoadConfig);
}

void CViewerContainer::showRangeScan(CNode* node)
{
	CRangeScanNode* obsNode = dynamic_cast<CRangeScanNode*>(node);
	ASSERT_(obsNode);

	auto obj = mrpt::make_aligned_shared<mrpt::opengl::CPlanarLaserScan>();
	obj->setScan(*(obsNode->observation().get()));
	obj->setPose(obsNode->getPose());
	obj->setSurfaceColor(1.0f, 0.0f, 0.0f, 0.5f);

	forEachGl([obsNode, obj](CGlWidget* gl) {
		gl->setSelected(obsNode->getPose().asTPose());
		gl->setLaserScan(obj);
	});
}

void CViewerContainer::forEachGl(const std::function<void(CGlWidget*)>& func)
{
	if (containsHelpInfo()) return;

	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		ASSERT_(gl);
		func(gl);
	}
}

void CViewerContainer::showRobotDirection(const mrpt::poses::CPose3D& pose)
{
	forEachGl([pose](CGlWidget* gl) { gl->setSelected(pose.asTPose()); });
}

void CViewerContainer::applyConfigChanges(RenderizableMaps renderizableMaps)
{
	if (containsHelpInfo()) return;

	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		auto sTypeIter = m_tabsInfo.find(i);
		ASSERT_(sTypeIter != m_tabsInfo.end());

		auto it = renderizableMaps.find(sTypeIter->second);
		ASSERT_(it != renderizableMaps.end());
		CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(i));
		ASSERT_(gl);

		gl->fillMap(it->second);
		gl->updateObservations();
	}
}

void CViewerContainer::updateConfigChanges(
	RenderizableMaps renderizableMaps, CDocument* doc, bool isShowAllObs)
{
	for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
	{
		QWidget* w = m_ui->m_tabWidget->widget(i);
		if (w != m_information) w->deleteLater();
	}
	m_tabsInfo.clear();
	m_ui->m_tabWidget->clear();

	if (renderizableMaps.empty())
	{
		m_ui->m_tabWidget->addTab(m_information, "Welcome!");
		return;
	}

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

		connect(
			gl, &CGlWidget::zoomChanged, this,
			&CViewerContainer::changeZoomInfo);
		connect(
			gl, &CGlWidget::mousePosChanged, this,
			&CViewerContainer::updateMouseInfo);
		connect(
			gl, &CGlWidget::azimuthChanged, this,
			&CViewerContainer::changeAzimuthDeg);
		connect(
			gl, &CGlWidget::elevationChanged, this,
			&CViewerContainer::changeElevationDeg);

		connect(
			gl, SIGNAL(deleteRobotPoses(const std::vector<size_t>&)),
			SIGNAL(deleteRobotPoses(const std::vector<size_t>&)));

		connect(
			gl, SIGNAL(showPoseDirection(size_t, double, double, double)),
			SIGNAL(showPoseDirection(size_t, double, double, double)));

		connect(
			gl,
			SIGNAL(moveRobotPoses(const std::vector<size_t>&, const QPointF&)),
			SIGNAL(moveRobotPoses(const std::vector<size_t>&, const QPointF&)));

		connect(
			gl, &CGlWidget::selectedChanged, this,
			&CViewerContainer::changedSelected);

		connect(
			gl, &CGlWidget::selectedChanged, this,
			&CViewerContainer::changedSelected);

		connect(
			this, &CViewerContainer::selectedChanged, gl,
			&CGlWidget::updateSelectionWithoutSignals);
	}
}

void CViewerContainer::setDocument(CDocument* doc)
{
	forEachGl([doc](CGlWidget* gl) { gl->setDocument(doc); });
}

void CViewerContainer::showAllObservation(bool is)
{
	forEachGl([is](CGlWidget* gl) { gl->setSelectedObservation(is); });
}

void CViewerContainer::changedSelected(
	const std::vector<CRobotPose::Ptr>& robotPoses)
{
	std::vector<size_t> indexes;
	for (auto& pose : robotPoses) indexes.push_back(pose->getId());
	emit selectedChanged(indexes);
}

void CViewerContainer::updatePanelInfo(int index)
{
	if (containsHelpInfo() || m_ui->m_tabWidget->count() == 0) return;

	CGlWidget* gl = dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->widget(index));
	ASSERT_(gl);

	changeZoomInfo(gl->getZoom());
	changeAzimuthDeg(gl->getAzimuthDegrees());
	changeElevationDeg(gl->getElevationDegrees());
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
	if (containsHelpInfo() || m_ui->m_tabWidget->count() == 0) return;

	float zoomFloat = d;
	int zoomInt = zoomFloat;
	m_ui->m_zoomSlider->setValue(zoomInt);
	CGlWidget* gl =
		dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->currentWidget());
	ASSERT_(gl);
	gl->setZoom(zoomFloat);
}

void CViewerContainer::zoomChanged(int d)
{
	if (containsHelpInfo() || m_ui->m_tabWidget->count() == 0) return;

	double zoomD = d;
	float zoomFloat = zoomD;
	m_ui->m_zoom->setValue(zoomD);
	CGlWidget* gl =
		dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->currentWidget());
	ASSERT_(gl);
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

	forEachGl([r, g, b, a, rBack, gBack, bBack, aBack, setting](CGlWidget* gl) {
		gl->setGridColor(r, g, b, a);
		gl->setVisibleGrid(setting.isGridVisibleChanged);
		gl->setBackgroundColor(rBack, gBack, bBack, aBack);
		gl->setBot(setting.currentBot);

		gl->setObservationSize(setting.robotPosesSize);
		gl->setSelectedObservationSize(setting.selectedRobotPosesSize);
		gl->setObservationColor(setting.robotPosesColor);
		gl->setSelectedObservationColor(setting.selectedRobotPosesColor);
	});

	CGlWidget* gl = getCurrentTabWidget();
	if (gl) gl->update();
}

CGlWidget* CViewerContainer::getCurrentTabWidget() const
{
	if (containsHelpInfo() || m_ui->m_tabWidget->count() <= 0) return nullptr;

	CGlWidget* gl =
		dynamic_cast<CGlWidget*>(m_ui->m_tabWidget->currentWidget());
	ASSERT_(gl);

	return gl;
}

bool CViewerContainer::containsHelpInfo() const
{
	return m_ui->m_tabWidget->widget(0) == m_information;
}
