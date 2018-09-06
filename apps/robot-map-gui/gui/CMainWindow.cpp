/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
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
#include <QErrorMessage>
#include <QDebug>

#include "mrpt/gui/CQtGlCanvasBase.h"
#include "mrpt/poses/CPose3D.h"
#include "mrpt/gui/about_box.h"
#include "mrpt/gui/error_box.h"
#include "mrpt/math/wrap2pi.h"
#include <mrpt/core/bits_math.h>

using mrpt::DEG2RAD;
using mrpt::RAD2DEG;

CMainWindow::CMainWindow(QWidget* parent)
	: QMainWindow(parent), m_ui(std::make_unique<Ui::CMainWindow>())
{
	m_ui->setupUi(this);

	m_ui->m_dockWidgetNodeViewer->setVisible(false);
	m_ui->m_dockWidgetDirection->setVisible(false);
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
		m_ui->m_observationsTree, &CObservationTree::clicked, this,
		&CMainWindow::itemClicked);
	connect(
		m_ui->m_actionAbout, &QAction::triggered, this, &CMainWindow::about);
	connect(
		m_ui->m_actionOpen, &QAction::triggered, this, &CMainWindow::openMap);
	connect(
		m_ui->m_actionSave, &QAction::triggered, this, &CMainWindow::saveMap);
	connect(
		m_ui->m_actionSaveAsText, &QAction::triggered, this,
		&CMainWindow::saveAsText);
	connect(
		m_ui->m_actionSaveAsPNG, &QAction::triggered, this,
		&CMainWindow::saveAsPNG);
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
		m_ui->m_poseDirection, &CPoseDirection::updateDirection, this,
		&CMainWindow::updateDirection);

	connect(
		m_ui->m_expandAll, &QPushButton::released, m_ui->m_observationsTree,
		&CObservationTree::expandAll);
	connect(
		m_ui->m_collapseAll, &QPushButton::released, m_ui->m_observationsTree,
		&CObservationTree::collapseAll);

	connect(
		m_ui->m_viewer, &CViewerContainer::selectedChanged, this,
		&CMainWindow::selectedChanged);
	connect(
		m_ui->m_viewer, &CViewerContainer::deleteRobotPoses, this,
		&CMainWindow::deleteRobotPoses);
	connect(
		m_ui->m_viewer, &CViewerContainer::moveRobotPoses, this,
		&CMainWindow::moveRobotPoses);
	connect(
		m_ui->m_viewer, &CViewerContainer::selectedChanged, this,
		&CMainWindow::selectedChanged);
	connect(
		m_ui->m_viewer, &CViewerContainer::showPoseDirection, this,
		&CMainWindow::showPoseDirection);

	m_ui->m_actionSave->setDisabled(true);
	m_ui->m_actionSaveAsText->setDisabled(true);

	m_recentFiles = m_settings.value("Recent").toStringList();
	addRecentFilesToMenu();
}

CMainWindow::~CMainWindow()
{
	clearObservationsViewer();

	if (m_document) delete m_document;

	if (m_model) delete m_model;

	m_settings.setValue("Recent", m_recentFiles);
}

void CMainWindow::loadMap(const QString& fileName)
{
	if (fileName.size() == 0) return;

	createNewDocument();

	{
		QFile file(fileName);
		if (!file.exists())
		{
			showErrorMessage("File doesn't exist!");
			return;
		}
	}
	std::string file = fileName.toStdString();

	mrpt::gui::tryCatch(
		[file, this]() {
			m_document->loadSimpleMap(file);
			mrpt::maps::TSetOfMetricMapInitializers mapCfg =
				m_ui->m_configWidget->config();
			m_document->setListOfMaps(mapCfg);
			updateRenderMapFromConfig();
			m_ui->m_viewer->changeHelpTextToAboutConfig();

			if (m_model) delete m_model;
			m_model = new CObservationTreeModel(
				m_document->simplemap(), m_ui->m_observationsTree);
			m_ui->m_observationsTree->setModel(m_model);

			addToRecent(file);
		},
		"The file is corrupted and cannot be opened!");
}

void CMainWindow::about()
{
	mrpt::gui::show_mrpt_about_box_Qt("Robot-made maps viewer");
}

void CMainWindow::openMap()
{
	QString path;
	if (!m_recentFiles.empty())
	{
		QFileInfo fi(m_recentFiles.front());
		path = fi.absolutePath();
	}

	QString fileName = QFileDialog::getOpenFileName(
		this, tr("Open File"), path, tr("Files (*.simplemap *.simplemap.gz)"));

	loadMap(fileName);
}

void CMainWindow::saveMap()
{
	if (!m_document) return;

	m_document->saveSimpleMap();
	updateSaveButtonState();
}

void CMainWindow::saveAsText()
{
	if (!m_document) return;

	QString fileName = QFileDialog::getSaveFileName(
		this, tr("Save as txt"), "", tr("Files (*.txt)"));

	if (fileName.size() == 0) return;

	m_document->saveAsText(fileName.toStdString());
}

void CMainWindow::saveAsPNG()
{
	if (!m_document) return;

	QString fileName = QFileDialog::getSaveFileName(
		this, tr("Save as txt"), "", tr("Files (*.png)"));

	if (fileName.size() == 0) return;

	m_document->saveAsPng(fileName.toStdString());
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
			ASSERT_(posesNode);
			m_ui->m_viewer->showRobotDirection(posesNode->getPose());
			break;
		}
		case CNode::ObjectType::Image:
		{
			m_ui->m_dockWidgetNodeViewer->setVisible(true);
			CObservationImageNode* imageNode =
				dynamic_cast<CObservationImageNode*>(node);
			ASSERT_(imageNode);
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
			ASSERT_(stereoImageNode);

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

void CMainWindow::selectedChanged(const std::vector<size_t>& idx)
{
	m_ui->m_observationsTree->changeSelected(idx);
}

void CMainWindow::addRobotPosesFromMap(
	std::vector<size_t> idx,
	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs)
{
	if (!m_document || idx.empty()) return;

	m_document->insert(idx, posesObsPairs);
	updateSaveButtonState();
	applyMapsChanges();
}

void CMainWindow::deleteRobotPosesFromMap(const std::vector<size_t>& idx)
{
	if (!m_document || idx.empty()) return;

	m_document->remove(idx);
	updateSaveButtonState();
	applyMapsChanges();
}

void CMainWindow::moveRobotPosesOnMap(
	const std::vector<size_t>& idx, const QPointF& dist)
{
	if (!m_document || idx.empty()) return;

	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs =
		m_document->get(idx);
	for (size_t i = 0; i < idx.size(); ++i)
	{
		mrpt::poses::CPose3DPDF::Ptr posePDF = posesObsPairs.at(i).first;
		mrpt::poses::CPose3D pose = posePDF->getMeanVal();

		pose.setFromValues(
			pose[0], pose[1], pose[2], pose.yaw(), pose.pitch(), pose.roll());
		posePDF->changeCoordinatesReference(
			mrpt::poses::CPose3D(dist.x(), dist.y(), 0.0));
	}
	m_document->move(idx, posesObsPairs);
	updateSaveButtonState();
	applyMapsChanges();
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

void CMainWindow::deleteRobotPoses(const std::vector<size_t>& idx)
{
	if (!m_document || idx.empty()) return;

	mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs =
		m_document->getReverse(idx);

	auto reverseInd = m_document->remove(idx);
	updateSaveButtonState();
	applyMapsChanges();

	auto redo = [idx, this]() { this->deleteRobotPosesFromMap(idx); };

	std::reverse(reverseInd.begin(), reverseInd.end());
	auto undo = [reverseInd, posesObsPairs, this]() {
		this->addRobotPosesFromMap(reverseInd, posesObsPairs);
	};

	CUndoManager::instance().addAction(undo, redo);
}

void CMainWindow::moveRobotPoses(
	const std::vector<size_t>& idx, const QPointF& dist)
{
	if (!m_document || idx.empty()) return;

	moveRobotPosesOnMap(idx, dist);
	auto redo = [idx, dist, this]() { this->moveRobotPosesOnMap(idx, dist); };
	auto undo = [idx, dist, this]() {
		this->moveRobotPosesOnMap(idx, -dist);
		;
	};
	CUndoManager::instance().addAction(undo, redo);
}

void CMainWindow::openRecent()
{
	QAction* action = qobject_cast<QAction*>(sender());
	loadMap(action->text());
}

void CMainWindow::saveMetricMapRepresentation()
{
	if (!m_document) return;

	QString fileName = QFileDialog::getSaveFileName(
		this, tr("Save as representation"), "", tr("Files (*)"));

	if (fileName.size() == 0) return;

	QAction* action = qobject_cast<QAction*>(sender());
	m_document->saveMetricMapRepresentationToFile(
		fileName.toStdString(), action->text().toStdString());
}

void CMainWindow::saveMetricmapInBinaryFormat()
{
	if (!m_document) return;

	QString fileName = QFileDialog::getSaveFileName(
		this, tr("Save in binary format"), "", tr("Files (*)"));

	if (fileName.size() == 0) return;

	QAction* action = qobject_cast<QAction*>(sender());

	m_document->saveMetricmapInBinaryFormat(
		fileName.toStdString(), action->text().toStdString());
}

void CMainWindow::updateDirection(
	size_t index, double yaw, double pitch, double roll)
{
	if (!m_document) return;

	auto posesObsPair = m_document->get(index);

	auto posePDF = posesObsPair.first;
	auto pose = posePDF->getMeanVal();

	pose.setFromValues(
		pose[0], pose[1], pose[2], DEG2RAD(yaw), DEG2RAD(pitch), DEG2RAD(roll));
	auto newPosePDF = std::make_shared<mrpt::poses::CPose3DPDFGaussian>(
		pose, posePDF->getCovariance());

	posesObsPair.first = newPosePDF;
	m_document->move(index, posesObsPair);

	updateSaveButtonState();

	m_ui->m_poseDirection->blockSignals(true);
	applyMapsChanges();
	m_ui->m_poseDirection->blockSignals(false);
}

void CMainWindow::showPoseDirection(
	size_t idx, double yaw, double pitch, double roll)
{
	m_ui->m_dockWidgetDirection->setVisible(true);
	m_ui->m_poseDirection->setIndex(idx);
	m_ui->m_poseDirection->setDirection(
		RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll));
}

void CMainWindow::hidePoseDirection()
{
	m_ui->m_dockWidgetDirection->setVisible(false);
}

void CMainWindow::updateRenderMapFromConfig()
{
	if (!m_document) return;

	auto renderizableMaps = m_document->renderizableMaps();
	m_ui->m_viewer->updateConfigChanges(
		renderizableMaps, m_document, m_ui->m_actionShowAllObs->isChecked());

	m_ui->m_actionSaveAsText->setDisabled(!m_document->hasPointsMap());

	m_ui->m_saveMetricMapRepresentation->clear();
	m_ui->m_saveMetricmapInBinaryFormat->clear();

	for (auto& it : renderizableMaps)
	{
		QString name = QString::fromStdString(typeToName(it.first.type)) +
					   QString::number(it.first.index);

		auto action = m_ui->m_saveMetricMapRepresentation->addAction(name);
		connect(
			action, &QAction::triggered, this,
			&CMainWindow::saveMetricMapRepresentation);

		auto actionBinary =
			m_ui->m_saveMetricmapInBinaryFormat->addAction(name);
		connect(
			actionBinary, &QAction::triggered, this,
			&CMainWindow::saveMetricmapInBinaryFormat);
	}
}

void CMainWindow::applyMapsChanges()
{
	auto renderizableMaps = m_document->renderizableMaps();
	m_ui->m_viewer->applyConfigChanges(renderizableMaps);
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

void CMainWindow::addToRecent(const std::string& fileName)
{
	addToRecent(QString::fromStdString(fileName));
}

void CMainWindow::addToRecent(const QString& fileName)
{
	auto iter = std::find(m_recentFiles.begin(), m_recentFiles.end(), fileName);

	if (iter != m_recentFiles.end()) m_recentFiles.erase(iter);

	m_recentFiles.push_front(fileName);

	if (m_recentFiles.size() > 10) m_recentFiles.pop_back();

	addRecentFilesToMenu();
}

void CMainWindow::addRecentFilesToMenu()
{
	m_ui->m_menuRecentFiles->clear();
	for (auto& recent : m_recentFiles)
	{
		auto action = m_ui->m_menuRecentFiles->addAction(recent);
		connect(action, &QAction::triggered, this, &CMainWindow::openRecent);
	}
}

void CMainWindow::showErrorMessage(const QString& str) const
{
	QErrorMessage msg;
	msg.showMessage(str);
	msg.exec();
}

void CMainWindow::updateSaveButtonState()
{
	m_ui->m_actionSave->setDisabled(!m_document->isFileChanged());
}
