/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "CMainWindow.h"
#include "ui_CMainWindow.h"
#include "CDocument.h"
#include "CUndoManager.h"

#include "observationTree/CObservationTreeModel.h"
#include "observationTree/CObservationImageNode.h"
#include "observationTree/CObservationStereoImageNode.h"
#include "observationTree/CObservationsNode.h"
#include "observationTree/CPairNode.h"
#include "gui/observationTree/CPosesNode.h"

#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QTreeWidgetItem>
#include <QDebug>

#include "mrpt/gui/CQtGlCanvasBase.h"
#include "mrpt/poses/CPose3D.h"

CMainWindow::CMainWindow(QWidget* parent)
	: QMainWindow(parent),
	  m_document(nullptr),
	  m_model(nullptr),
	  m_ui(std::make_unique<Ui::CMainWindow>())
{
	m_ui->setupUi(this);

	m_ui->m_dockWidgetNodeViewer->setVisible(false);
	m_ui->m_dockWidgetConfig->setVisible(false);

	m_ui->m_undoAction->setShortcut(QKeySequence(QKeySequence::Undo));
	m_ui->m_redoAction->setShortcut(QKeySequence(QKeySequence::Redo));

	connect(
		m_ui->m_configWidget, &CConfigWidget::openedConfig, this,
		&CMainWindow::openConfig);
	connect(
		m_ui->m_configWidget, &CConfigWidget::applyConfigurationForCurrentMaps,
		this, &CMainWindow::applyConfigurationForCurrentMaps);
	connect(
		m_ui->m_configWidget, &CConfigWidget::addedMap, this,
		&CMainWindow::updateConfig);
	connect(
		m_ui->m_configWidget, &CConfigWidget::removedMap, this,
		&CMainWindow::updateConfig);
	connect(
		m_ui->m_configWidget, &CConfigWidget::updatedConfig, this,
		&CMainWindow::updateConfig);
	connect(
		m_ui->m_configWidget, &CConfigWidget::backgroundColorChanged,
		m_ui->m_viewer, &CViewerContainer::changeBackgroundColor);
	connect(
		m_ui->m_configWidget, &CConfigWidget::gridColorChanged, m_ui->m_viewer,
		&CViewerContainer::changeGridColor);
	connect(
		m_ui->m_configWidget, &CConfigWidget::gridVisibleChanged,
		m_ui->m_viewer, &CViewerContainer::setVisibleGrid);
	connect(
		m_ui->m_configWidget, &CConfigWidget::currentBotChanged, m_ui->m_viewer,
		&CViewerContainer::changeCurrentBot);

	connect(
		m_ui->m_observationsTree, &CObservationTree::clicked, this,
		&CMainWindow::itemClicked);

	connect(
		m_ui->m_actionOpen, &QAction::triggered, this, &CMainWindow::openMap);
	connect(m_ui->m_undoAction, &QAction::triggered, this, &CMainWindow::undo);
	connect(m_ui->m_redoAction, &QAction::triggered, this, &CMainWindow::redo);
	connect(
		m_ui->m_actionMapConfiguration, &QAction::triggered, this,
		&CMainWindow::showMapConfiguration);
	connect(
		m_ui->m_actionLoadConfig, &QAction::triggered, m_ui->m_configWidget,
		&CConfigWidget::openConfig);
	connect(
		m_ui->m_actionShowAllObs, &QAction::triggered, m_ui->m_viewer,
		&CViewerContainer::showAllObservation);

	connect(
		m_ui->m_expandAll, &QPushButton::released, m_ui->m_observationsTree,
		&CObservationTree::expandAll);
	connect(
		m_ui->m_collapseAll, &QPushButton::released, m_ui->m_observationsTree,
		&CObservationTree::collapseAll);

	connect(
		m_ui->m_viewer, &CViewerContainer::deleteRobotPoses, this,
		&CMainWindow::deleteRobotPoses);
	connect(
		m_ui->m_viewer, &CViewerContainer::moveRobotPoses, this,
		&CMainWindow::moveRobotPoses);
}

CMainWindow::~CMainWindow()
{
	clearObservationsViewer();

	if (m_document) delete m_document;

	if (m_model) delete m_model;
}

void CMainWindow::openMap()
{
	QString fileName = QFileDialog::getOpenFileName(
		this, tr("Open File"), "", tr("Files (*.simplemap *.simplemap.gz)"));

	if (fileName.size() == 0) return;

	createNewDocument();

	try
	{
		m_document->loadSimpleMap(fileName.toStdString());

		mrpt::maps::TSetOfMetricMapInitializers mapCfg =
			m_ui->m_configWidget->config();
		m_document->setListOfMaps(mapCfg);

		updateRenderMapFromConfig();

		if (m_model) delete m_model;
		m_model = new CObservationTreeModel(
			m_document->simplemap(), m_ui->m_observationsTree);
		m_ui->m_observationsTree->setModel(m_model);
	}
	catch (std::exception&)
	{
		createNewDocument();
		qDebug() << "Unexpected runtime error!";
	}
}

void CMainWindow::itemClicked(const QModelIndex& index)
{
	CNode* node = m_model->getNodeFromIndexSafe(index);
	CNode::ObjectType type = node->type();

	m_ui->m_dockWidgetNodeViewer->setVisible(false);
	clearObservationsViewer();

	switch (type)
	{
		case CNode::ObjectType::RangeScan:
		{
			m_ui->m_viewer->showRangeScan(node);
			break;
		}
		case CNode::ObjectType::Pos:
		{
			CPosesNode* posesNode = dynamic_cast<CPosesNode*>(node);
			assert(posesNode);
			m_ui->m_viewer->showRobotDirection(posesNode->getPose());
			break;
		}
		case CNode::ObjectType::Image:
		{
			m_ui->m_dockWidgetNodeViewer->setVisible(true);
			CObservationImageNode* imageNode =
				dynamic_cast<CObservationImageNode*>(node);
			assert(imageNode);
			mrpt::gui::CQtGlCanvasBase* gl = new mrpt::gui::CQtGlCanvasBase();
			gl->mainViewport()->setImageView(imageNode->observation()->image);
			m_ui->m_contentsNodeViewer->layout()->addWidget(gl);
			m_ui->m_viewer->showRobotDirection(imageNode->getPose());
			break;
		}
		case CNode::ObjectType::StereoImage:
		{
			m_ui->m_dockWidgetNodeViewer->setVisible(true);
			CObservationStereoImagesNode* stereoImageNode =
				dynamic_cast<CObservationStereoImagesNode*>(node);
			assert(stereoImageNode);

			mrpt::obs::CObservationStereoImages::Ptr observation =
				stereoImageNode->observation();

			mrpt::gui::CQtGlCanvasBase* gl1 = new mrpt::gui::CQtGlCanvasBase();
			gl1->mainViewport()->setImageView(observation->imageLeft);

			QLayout* layout = m_ui->m_contentsNodeViewer->layout();
			layout->addWidget(gl1);

			if (observation->hasImageRight)
			{
				mrpt::gui::CQtGlCanvasBase* gl2 =
					new mrpt::gui::CQtGlCanvasBase();
				gl2->mainViewport()->setImageView(observation->imageRight);
				layout->addWidget(gl2);
			}
			m_ui->m_viewer->showRobotDirection(stereoImageNode->getPose());
			break;
		}
		default:
			break;
	}
}

void CMainWindow::updateConfig()
{
	if (!m_document) return;
	mrpt::maps::TSetOfMetricMapInitializers mapCfg =
		m_ui->m_configWidget->config();
	m_document->setListOfMaps(mapCfg);
	updateRenderMapFromConfig();
}

void CMainWindow::openConfig(const std::string& str)
{
	if (!m_document) return;

	m_document->setConfig(str);

	auto config = m_document->config();
	m_ui->m_configWidget->setConfig(config);
	updateRenderMapFromConfig();
}

void CMainWindow::applyConfigurationForCurrentMaps()
{
	if (!m_document) return;

	mrpt::maps::TSetOfMetricMapInitializers mapCfg =
		m_ui->m_configWidget->config();
	m_document->setListOfMaps(mapCfg);

	auto renderizableMaps = m_document->renderizableMaps();
	m_ui->m_viewer->applyConfigChanges(renderizableMaps);
	m_ui->m_viewer->setGeneralSetting(m_ui->m_configWidget->generalSetting());
}

void CMainWindow::showMapConfiguration()
{
	QDialog* d = new QDialog();
	QBoxLayout* lay = new QBoxLayout(QBoxLayout::LeftToRight);

	QWidget* w = m_ui->m_dockWidgetConfig->widget();
	lay->addWidget(w);
	d->setLayout(lay);
	d->exec();
	m_ui->m_dockWidgetConfig->setWidget(w);
	delete d;
}

void CMainWindow::addRobotPosesFromMap(
	std::vector<int> idx,
	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs)
{
	if (m_document && !idx.empty())
	{
		m_document->insert(idx, posesObsPairs);
		updateRenderMapFromConfig();
	}
}

void CMainWindow::deleteRobotPosesFromMap(const std::vector<int> &idx)
{
	if (m_document && !idx.empty())
	{
		m_document->remove(idx);
		updateRenderMapFromConfig();
	}
}

void CMainWindow::moveRobotPosesOnMap(const std::vector<int> &idx, const QPointF &dist)
{
	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs =
		m_document->get(idx);
	for (int i = 0; i < idx.size(); ++i)
	{
		mrpt::poses::CPose3DPDF::Ptr posePDF = posesObsPairs.at(i).first;
		mrpt::poses::CPose3D pose = posePDF->getMeanVal();

		pose.setFromValues(
			pose[0], pose[1], pose[2], pose.yaw(),
			pose.pitch(), pose.roll());
		posePDF->changeCoordinatesReference(
			mrpt::poses::CPose3D(dist.x(), dist.y(), 0.0));
	}
	m_document->move(idx, posesObsPairs);
	updateRenderMapFromConfig();
}

void CMainWindow::undo()
{
	if (!CUndoManager::instance().hasUndo()) return;
	UserAction action = CUndoManager::instance().undoAction();
	action();
}

void CMainWindow::redo()
{
	if (!CUndoManager::instance().hasRedo()) return;
	UserAction action = CUndoManager::instance().redoAction();
	action();
}

void CMainWindow::deleteRobotPoses(const std::vector<int>& idx)
{
	if (!m_document || idx.empty()) return;

	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs =
		m_document->getReverse(idx);

	std::vector<int> reverseInd = m_document->remove(idx);
	updateRenderMapFromConfig();

	auto redo = [idx, this]() { this->deleteRobotPosesFromMap(idx); };

	std::reverse(reverseInd.begin(), reverseInd.end());
	auto undo = [reverseInd, posesObsPairs, this]() {
		this->addRobotPosesFromMap(reverseInd, posesObsPairs);
	};

	CUndoManager::instance().addAction(undo, redo);
}

void CMainWindow::moveRobotPoses(const std::vector<int>& idx, const QPointF &dist)
{
	if (!m_document || idx.empty()) return;

	moveRobotPosesOnMap(idx, dist);
	auto redo = [idx, dist, this]() { this->moveRobotPosesOnMap(idx, dist); };
	auto undo = [idx, dist, this]() {
		this->moveRobotPosesOnMap(idx, -dist);;
	};
	CUndoManager::instance().addAction(undo, redo);
}

void CMainWindow::updateRenderMapFromConfig()
{
	auto renderizableMaps = m_document->renderizableMaps();
	m_ui->m_viewer->updateConfigChanges(
		renderizableMaps, m_document, m_ui->m_actionShowAllObs->isChecked());
}

void CMainWindow::createNewDocument()
{
	if (m_document) delete m_document;

	m_document = new CDocument();

	m_ui->m_viewer->setDocument(m_document);
}

void CMainWindow::clearObservationsViewer()
{
	QLayout* layout = m_ui->m_contentsNodeViewer->layout();
	QLayoutItem* child;
	while ((child = layout->takeAt(0)) != 0) delete child;
}
