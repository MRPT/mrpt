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
	QObject::connect(m_ui->m_treeView,	SIGNAL(clicked(const QModelIndex &)),	SLOT(itemClicked(const QModelIndex &)));
}

CMainWindow::~CMainWindow()
{
	if (m_document)
		delete m_document;

	if (m_model)
		delete m_model;
}

void CMainWindow::openMap()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.simplemap)"));
	QString configName = QFileDialog::getOpenFileName(this, tr("Open Config File"), "", tr("Files (*.ini)"));

	if (fileName.size() == 0 || configName.size() == 0)
		return;


	if (m_document)
		delete m_document;

	m_document = new CDocument(fileName.toStdString(), configName.toStdString());

	auto renderizableMaps = m_document->renderizableMaps();
	m_ui->m_tabWidget->clear();
	for(auto &it: renderizableMaps)
	{
		CGlWidget *gl = new CGlWidget(m_ui->m_tabWidget);
		gl->fillMap(it.second);
		m_ui->m_tabWidget->addTab(gl, QString::fromStdString(it.first));
	}


	if (m_model)
		delete m_model;
	m_model = new CObservationTreeModel(m_document->simplemap(), m_ui->m_treeView);
	m_ui->m_treeView->setModel(m_model);
	m_ui->m_treeView->header()->close();

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
