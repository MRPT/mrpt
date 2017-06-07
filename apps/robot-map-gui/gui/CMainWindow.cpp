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
	//, m_tabwidget(new QTabWidget(this))
	, m_ui(std::make_unique<Ui::CMainWindow>())
{
	m_ui->setupUi(this);
	QObject::connect(m_ui->openAction, SIGNAL(triggered(bool)), SLOT(openMap()));
	//QMainWindow::setCentralWidget(m_tabwidget);
}

CMainWindow::~CMainWindow()
{
	delete m_document;
}

void CMainWindow::openMap()
{
	if (m_document)
		delete m_document;


	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.simplemap)"));
	QString configName = QFileDialog::getOpenFileName(this, tr("Open Config File"), "", tr("Files (*.ini)"));

	if (fileName.size() && configName.size())
	{
		m_document = new CDocument(fileName.toStdString(), configName.toStdString());

		auto renderizableMaps = m_document->renderizableMaps();
		m_ui->m_tabWidget->clear();
		for(auto &it: renderizableMaps)
		{
			CGlWidget *gl = new CGlWidget(m_ui->m_tabWidget);
			gl->fillMap(it.second);
			m_ui->m_tabWidget->addTab(gl, QString::fromStdString(it.first));
		}

		CObservationTreeModel *model = new CObservationTreeModel(m_document->simplemap(), m_ui->m_tableView);
		m_ui->m_tableView->setModel(model);
	}
}
