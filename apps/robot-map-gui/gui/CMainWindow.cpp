#include "CMainWindow.h"
#include "CGLWidget.h"
#include "CDocument.h"
#include "observationTree/CObservationTreeModel.h"

#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>

#include "ui_CMainWindow.h"


CMainWindow::CMainWindow(QWidget *parent)
	: QMainWindow(parent)
	, m_document(nullptr)
	, m_model(nullptr)
	, m_ui(std::make_unique<Ui::CMainWindow>())
{
	m_ui->setupUi(this);
	QObject::connect(m_ui->openAction, SIGNAL(triggered(bool)), SLOT(openMap()));
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
