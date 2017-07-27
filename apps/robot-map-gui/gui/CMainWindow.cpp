<<<<<<< HEAD
/* +------------------------------------------------------------------------+
=======
/* +---------------------------------------------------------------------------+
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
<<<<<<< HEAD
   +------------------------------------------------------------------------+ */
=======
+ -------------------------------------------------------------------------- -
	+* /
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540
#include "CMainWindow.h"
#include "ui_CMainWindow.h"
#include "CDocument.h"
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

<<<<<<< HEAD CMainWindow::CMainWindow(QWidget* parent) : QMainWindow(parent)
{
}
CMainWindow::~CMainWindow() {}
======= CMainWindow::CMainWindow(QWidget* parent) : QMainWindow(parent), m_document(nullptr), m_model(nullptr), m_ui(std::make_unique < Ui::CMainWindow > ())
{
	m_ui->setupUi(this);
	QObject::connect(
		m_ui->openAction, SIGNAL(triggered(bool)), SLOT(openMap()));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(openedConfig(const std::string)),
		SLOT(updateConfig(const std::string)));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(applyConfigurationForCurrentMaps()),
		SLOT(applyConfigurationForCurrentMaps()));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(addedMap()), SLOT(updateConfig()));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(removedMap()), SLOT(updateConfig()));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(updatedConfig()), SLOT(updateConfig()));
	QObject::connect(
		m_ui->m_observationsTree, SIGNAL(clicked(const QModelIndex&)),
		SLOT(itemClicked(const QModelIndex&)));
	QObject::connect(
		m_ui->m_actionLoadConfig, SIGNAL(triggered(bool)), m_ui->m_configWidget,
		SLOT(openConfig()));

	QObject::connect(
		m_ui->m_actionShowAllObs, SIGNAL(triggered(bool)), m_ui->m_viewer,
		SLOT(showAllObservation(bool)));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(backgroundColorChanged(QColor)),
		m_ui->m_viewer, SLOT(changeBackgroundColor(QColor)));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(gridColorChanged(QColor)), m_ui->m_viewer,
		SLOT(changeGridColor(QColor)));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(gridVisibleChanged(bool)), m_ui->m_viewer,
		SLOT(setVisibleGrid(bool)));
	QObject::connect(
		m_ui->m_configWidget, SIGNAL(currentBotChanged(int)), m_ui->m_viewer,
		SLOT(changeCurrentBot(int)));

	QObject::connect(
		m_ui->m_actionMapConfiguration, SIGNAL(triggered(bool)),
		SLOT(showMapConfiguration()));

	QObject::connect(
		m_ui->m_expandAll, SIGNAL(released()), m_ui->m_observationsTree,
		SLOT(expandAll()));
	QObject::connect(
		m_ui->m_collapseAll, SIGNAL(released()), m_ui->m_observationsTree,
		SLOT(collapseAll()));

	m_ui->m_dockWidgetNodeViewer->setVisible(false);
	m_ui->m_dockWidgetConfig->setVisible(false);
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

void CMainWindow::updateConfig(const std::string str)
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
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540
