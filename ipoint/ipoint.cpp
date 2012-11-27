#include "ipoint.h"
#include <QMessageBox>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>


IntrusionPointWindow::IntrusionPointWindow(QWidget * parent) :
  QMainWindow(parent)
{
  setAttribute(Qt::WA_DeleteOnClose, true);
  view_ = new ViewWindow(this);
  if ( view_ )
    setCentralWidget(view_);
  setWindowIcon(QIcon(":/images/main_icon.png"));
  setWindowTitle(tr("Delaunay polygons triangulation algorithm"));
  createMenu();
  createToolBar();
  createStatusBar();
}

IntrusionPointWindow::~IntrusionPointWindow()
{
}

void IntrusionPointWindow::closeEvent(QCloseEvent * event)
{
  event->accept();
}

void IntrusionPointWindow::onNew()
{
  if ( view_ )
    view_->reset();
}

void IntrusionPointWindow::onLoad()
{
  QString fname = QFileDialog::getOpenFileName(0, QObject::tr("Load polyline"), QObject::tr(""), QObject::tr("Text files (*.txt)"));
  if ( view_ )
    view_->load(fname);
}

void IntrusionPointWindow::onSave()
{
  QString fname = QFileDialog::getSaveFileName(0, QObject::tr("Save polyline"), QObject::tr("polyline.txt"), QObject::tr("Text files (*.txt)"));
  if ( view_ )
    view_->save(fname);
}

void IntrusionPointWindow::createMenu()
{
  onNewAction_ = new QAction(tr("&New"), this);
  onNewAction_->setIcon(QIcon(":/images/new_icon.png"));
  onNewAction_->setShortcut(tr("Ctrl+N"));
  onNewAction_->setStatusTip(tr("Create a new document"));
  connect(onNewAction_, SIGNAL(triggered()), this, SLOT(onNew()));

  onLoadAction_ = new QAction(tr("&Open"), this);
  onLoadAction_->setIcon(QIcon(":/images/file_open.png"));
  onLoadAction_->setShortcut(tr("Ctrl+O"));
  onLoadAction_->setStatusTip(tr("Load polyline"));
  connect(onLoadAction_, SIGNAL(triggered()), this, SLOT(onLoad()));

  onSaveAction_ = new QAction(tr("&Save"), this);
  onSaveAction_->setIcon(QIcon(":/images/file_save.png"));
  onSaveAction_->setShortcut(tr("Ctrl+S"));
  onSaveAction_->setStatusTip(tr("Save polyline"));
  connect(onSaveAction_, SIGNAL(triggered()), this, SLOT(onSave()));

  fileMenu_ = menuBar()->addMenu(tr("&File"));
  fileMenu_->addAction(onNewAction_);
  fileMenu_->addAction(onLoadAction_);
  fileMenu_->addAction(onSaveAction_);
}

void IntrusionPointWindow::createToolBar()
{
  fileToolbar_ = addToolBar(tr("&File"));
  fileToolbar_->addAction(onNewAction_);
  fileToolbar_->addAction(onLoadAction_);
  fileToolbar_->addAction(onSaveAction_);
}

void IntrusionPointWindow::createStatusBar()
{
  statusLabelMouse_ = new QLabel(tr("Mouse position"));
  statusLabelMouse_->setAlignment(Qt::AlignHCenter);
  statusLabelMouse_->setMinimumSize(statusLabelMouse_->sizeHint());
  mousePosLabel_ = new QLabel(tr("{0, 0}"));
  mousePosLabel_->setMinimumWidth(200);
  mousePosLabel_->setIndent(5);

  statusLabelTris_ = new QLabel(tr("Triangles count"));
  statusLabelTris_->setAlignment(Qt::AlignHCenter);
  statusLabelTris_->setMinimumSize(statusLabelTris_->sizeHint());
  trianglesLabel_ = new QLabel(tr("0"));
  trianglesLabel_->setMinimumWidth(100);
  trianglesLabel_->setIndent(5);

  statusBar()->addWidget(statusLabelMouse_);
  statusBar()->addWidget(mousePosLabel_);
  statusBar()->addWidget(statusLabelTris_);
  statusBar()->addWidget(trianglesLabel_);

  if ( view_ )
  {
    connect(view_, SIGNAL(trianglesChanged(size_t)), this, SLOT(updateTrianglesCount(size_t)));
    connect(view_, SIGNAL(mouseMoved(const QPoint & )), this, SLOT(updateMousePt(const QPoint & )));
  }
}

void IntrusionPointWindow::updateMousePt(const QPoint & mousePt)
{
  QString str;
  str.sprintf( "{%d, %d}", mousePt.x(), mousePt.y() );
  mousePosLabel_->setText( str );
}

void IntrusionPointWindow::updateTrianglesCount(size_t trisN)
{
  QString str;
  str.sprintf( "%d", trisN );
  trianglesLabel_->setText( str );
}