#include "CMainWindow.h"
#include "GLWidget.h"

#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>


CMainWindow::CMainWindow(QWidget *parent)
	: QMainWindow(parent)
	, glWidget_(new GlWidget(this))
{
	initMenu();
	QMainWindow::setCentralWidget(glWidget_);
}

CMainWindow::~CMainWindow()
{

}

void CMainWindow::initMenu()
{
	setMenuBar(new QMenuBar(this));
	QMenu* file_menu = menuBar()->addMenu(QObject::tr("File"));
	QAction* new_file  = new QAction(tr("New"), this);
	file_menu->addAction(new_file);

	QAction* load_file  = new QAction(tr("Load"), this);
	file_menu->addAction(load_file);
	QObject::connect(load_file, SIGNAL(triggered(bool)), SLOT(openMap()));
}

void CMainWindow::openMap()
{
	QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.simplemap)"));
	QString config_name = QFileDialog::getOpenFileName(this, tr("Open Config File"), "", tr("Files (*.ini)"));
	glWidget_->loadFile(file_name, config_name);
}
