#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QObject>
#include "tpdoobject.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_btnServoON_clicked();

    void on_btnStartServer_clicked();

    void on_btnOperationModeWrite_clicked();

    void on_btnMaxMotorVelocity_clicked();

    void on_btnQuickStop_clicked();

    void on_btnReset_clicked();

    void on_btnTargetPositionWrite_clicked();

    void on_btnTargetVelocityWrite_clicked();

    void on_btnAccelerationWrite_clicked();

    void on_btnDecelerationWrite_clicked();

public slots:

    void getValue();

private:
    Ui::MainWindow *ui;

public:
    QThread thread_TPDO;
    TPDOObject TPDO_object;
};

#endif // MAINWINDOW_H
