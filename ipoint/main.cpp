#include <QApplication>
#include "ipoint.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  IntrusionPointWindow * wnd = new IntrusionPointWindow;
  wnd->resize(800, 600);
  wnd->show();
  return app.exec();
}
