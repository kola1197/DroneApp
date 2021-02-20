#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QSurfaceFormat fmt;
    fmt.setSamples(20);
    QSurfaceFormat::setDefaultFormat(fmt);
    MainWindow w;
    w.setTestImage();
    w.show();
    return a.exec();
}
