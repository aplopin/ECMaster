#include "mainwindow.h"

#include <QApplication>
#include <QObject>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;

    QObject::connect(&window.TPDO_object, SIGNAL(changedData()), &window, SLOT(getValue()));

    window.setWindowTitle("EtherCAT Master");
    //window.setFixedSize(479, 609);
    window.show();

    return app.exec();
}
