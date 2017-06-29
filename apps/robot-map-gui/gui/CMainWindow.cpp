/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CMainWindow.h"
#include "CGLWidget.h"
#include "CDocument.h"
#include "observationTree/CObservationTreeModel.h"
#include "observationTree/CPosesNode.h"

#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QTreeWidgetItem>

#include "ui_CMainWindow.h"


CMainWindow::CMainWindow(QWidget *parent)
	: QMainWindow(parent)
	, m_document(nullptr)
	, m_model(nullptr)
	, m_ui(std::make_unique<Ui::CMainWindow>())
{
	m_ui->setupUi(this);
	QObject::connect(m_ui->openAction,		SIGNAL(triggered(bool)),			SLOT(openMap()));
	QObject::connect(m_ui->m_configWidget, SIGNAL(openedConfig(const std::string)), SLOT(updateConfig(const std::string)));
	QObject::connect(m_ui->m_configWidget, SIGNAL(addedMap()), SLOT(updateConfig()));
	QObject::connect(m_ui->m_configWidget, SIGNAL(updatedConfig()), SLOT(updateConfig()));
	QObject::connect(m_ui->m_observationsTree,	SIGNAL(clicked(const QModelIndex &)),	SLOT(itemClicked(const QModelIndex &)));
}

CMainWindow::~CMainWindow()
{
	if (m_document)
		delete m_document;

	if (m_model)
		delete m_model;

	delete m_ui->m_configWidget;
}

void CMainWindow::addMap(std::string name)
{
	updateRenderMapFromConfig();
}

void CMainWindow::openMap()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.simplemap *.simplemap.gz)"));

	if (fileName.size() == 0)
		return;

	if (m_document)
		delete m_document;

	m_document = new CDocument(fileName.toStdString());

	updateRenderMapFromConfig();

	if (m_model)
		delete m_model;
	m_model = new CObservationTreeModel(m_document->simplemap(), m_ui->m_observationsTree);
	m_ui->m_observationsTree->setModel(m_model);
}

void CMainWindow::itemClicked(const QModelIndex &index)
{
	CNode* node = m_model->getNodeFromIndexSafe(index);
	CPoseNode *posesNode = dynamic_cast<CPoseNode *>(node);

	if (posesNode)
	{
		for (int i = 0; i < m_ui->m_tabWidget->count(); ++i)
		{
			QWidget *w = m_ui->m_tabWidget->widget(i);
			CGlWidget *gl = dynamic_cast<CGlWidget *>(w);
			if (gl)
				gl->setSelected(posesNode->getPose());
		}
	}
}

void CMainWindow::updateConfig()
{
	mrpt::maps::TSetOfMetricMapInitializers mapCfg = m_ui->m_configWidget->config();
	m_document->setListOfMaps(mapCfg);
	updateRenderMapFromConfig();
}

void CMainWindow::updateConfig(const std::string str)
{
	m_document->setConfig(str);

	auto config = m_document->config();
	m_ui->m_configWidget->setConfig(config);
	updateRenderMapFromConfig();
}

void CMainWindow::updateRenderMapFromConfig()
{
	m_ui->m_tabWidget->clear();

	auto renderizableMaps = m_document->renderizableMaps();
	for(auto &it: renderizableMaps)
	{
		CGlWidget *gl = new CGlWidget(m_ui->m_tabWidget);
		gl->fillMap(it.second);
		m_ui->m_tabWidget->addTab(gl, QString::fromStdString(it.first));
	}
}
