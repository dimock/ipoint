#include "ipoint.h"
#include <QMessageBox>
#include <QMenuBar>
#include <QStatusBar>


IntrusionPointWindow::IntrusionPointWindow(QWidget * parent) :
  QMainWindow(parent)
{
  setAttribute(Qt::WA_DeleteOnClose, true);
  view_ = new ViewWindow(this);
  if ( view_ )
    setCentralWidget(view_);
  setWindowIcon(QIcon(":/images/main_icon.png"));
  setWindowTitle(tr("polygons triangulation algorithm *intrusion point's method*"));
  createMenu();
  createToolBar();
  createStatusBar();
}

IntrusionPointWindow::~IntrusionPointWindow()
{
  //QMessageBox::information(0, tr("FYI"), tr("IntrusionPointWindow deleted"));
}

void IntrusionPointWindow::closeEvent(QCloseEvent * event)
{
//  QMainWindow::closeEvent(event);
//  QMessageBox::information(0, tr("FYI"), tr("IntrusionPointWindow is going to be closed"));
  event->accept();
}

void IntrusionPointWindow::onNew()
{
  if ( view_ )
    view_->reset();
}

void IntrusionPointWindow::createMenu()
{
  onNewAction_ = new QAction(tr("&New"), this);
  onNewAction_->setIcon(QIcon(":/images/new_icon.png"));
  onNewAction_->setShortcut(tr("Ctrl+N"));
  onNewAction_->setStatusTip(tr("Create a new document"));
  connect(onNewAction_, SIGNAL(triggered()), this, SLOT(onNew()));

  fileMenu_ = menuBar()->addMenu(tr("&File"));
  fileMenu_->addAction(onNewAction_);
}

void IntrusionPointWindow::createToolBar()
{
  fileToolbar_ = addToolBar(tr("&File"));
  fileToolbar_->addAction(onNewAction_);
}

void IntrusionPointWindow::createStatusBar()
{
  statusLabel_ = new QLabel(tr("Mouse position"));
  statusLabel_->setAlignment(Qt::AlignHCenter);
  statusLabel_->setMinimumSize(statusLabel_->sizeHint());
  mousePosLabel_ = new QLabel(tr("{0, 0}"));
  mousePosLabel_->setIndent(5);
  statusBar()->addWidget(statusLabel_);
  statusBar()->addWidget(mousePosLabel_);

  if ( view_ )
    connect(view_, SIGNAL(mouseMoved(const QPoint & )), this, SLOT(updateStatusBar(const QPoint & )));
}

void IntrusionPointWindow::updateStatusBar(const QPoint & mousePt)
{
  QString str;
  str.sprintf( "{%d, %d}", mousePt.x(), mousePt.y() );
  mousePosLabel_->setText( str );
}