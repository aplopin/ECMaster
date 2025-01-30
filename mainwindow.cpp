#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QString"
#include <QTableWidget>
#include <QObject>

#include "server.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Задание свойств таблицы вывода данных PDO
    ui->tableWidget->setRowCount(8);
    ui->tableWidget->setColumnCount(10);

    // Отключение отображение заголовков строк
    ui->tableWidget->verticalHeader()->hide();

    // Задание названий заголовков столбцов
    ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "Register name" << "Address" << "Axis1" << "Axis2" << "Axis3" << "Axis4"\
                                               << "Axis5" << "Axis6" << "Axis7" << "Axis8");

    // Заполнение столбца названий регистров TPDO
    ui->tableWidget->setItem(0, 0, new QTableWidgetItem("Status Word (-)"));
    ui->tableWidget->setItem(1, 0, new QTableWidgetItem("Actual Position (Unit)"));
    ui->tableWidget->setItem(2, 0, new QTableWidgetItem("Actual Velocity (Unit/s)"));
    ui->tableWidget->setItem(3, 0, new QTableWidgetItem("Actual Torque (0.1%)"));
    ui->tableWidget->setItem(4, 0, new QTableWidgetItem("Position Command Value (Unit)"));
    ui->tableWidget->setItem(5, 0, new QTableWidgetItem("Velocity Command Value (Unit/s)"));
    ui->tableWidget->setItem(6, 0, new QTableWidgetItem("Torque Command Value (0.1%)"));
    ui->tableWidget->setItem(7, 0, new QTableWidgetItem("Error Code (-)"));

    // Заполнение адресов регистров TPDO
    ui->tableWidget->setItem(0, 1, new QTableWidgetItem("0x6041 - 00h"));
    ui->tableWidget->setItem(1, 1, new QTableWidgetItem("0x60B0 - 00h"));
    ui->tableWidget->setItem(2, 1, new QTableWidgetItem("0x606C - 00h"));
    ui->tableWidget->setItem(3, 1, new QTableWidgetItem("0x607A - 00h"));
    ui->tableWidget->setItem(4, 1, new QTableWidgetItem("0x6062 - 00h"));
    ui->tableWidget->setItem(5, 1, new QTableWidgetItem("0x606B - 00h"));
    ui->tableWidget->setItem(6, 1, new QTableWidgetItem("0x6074 - 00h"));
    ui->tableWidget->setItem(7, 1, new QTableWidgetItem("0x603F - 00h"));

    // Выделение красным цветом заголовков осей - оси не подключены!
    for(uint8_t i = 2; i < ui->tableWidget->columnCount(); i ++)
    {
        ui->tableWidget->horizontalHeaderItem(i)->setBackground(Qt::red);
    }

    // Определение размеров столбцов. Размер строк определен в mainwindow.ui
    ui->tableWidget->setColumnWidth(0, 200);
    for(uint8_t i = 1; i < ui->tableWidget->columnCount(); i ++)
    {
        ui->tableWidget->setColumnWidth(i, 85);
    }

    // Отключения прокрутки ScrollBar по горизонтали и вертикали
    ui->tableWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->tableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // Запуск метода run() осуществляется по сигналу запуска от соответствующего потока
    connect(&thread_TPDO, &QThread::started, &TPDO_object, &TPDOObject::run);
    // Остановка потока выполняется по сигналу finished от соответствующего объекта в потоке
    connect(&TPDO_object, &TPDOObject::finished, &thread_TPDO, &QThread::terminate);

    // Передача объекта в поток
    TPDO_object.moveToThread(&thread_TPDO);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnStartServer_clicked()
{
    if(start_flag)
    {
        ui->btnStartServer->setText("START");
        ui->btnServoON->setText("ServoOFF");
        ui->btnServoON->setStyleSheet(QString::fromUtf8("background-color: rgb(224, 27, 36)"));

        ui->btnServoON->setEnabled(false);
        ui->btnOperationModeWrite->setEnabled(false);
        ui->btnTargetPositionWrite->setEnabled(false);
        ui->btnMaxMotorVelocity->setEnabled(false);
        ui->btnQuickStop->setEnabled(false);
        ui->btnReset->setEnabled(false);

        ui->btnOperationModeWrite->setEnabled(false);
        ui->btnTargetPositionWrite->setEnabled(false);
        ui->btnMaxMotorVelocity->setEnabled(false);
        ui->btnTargetVelocityWrite->setEnabled(false);
        ui->btnAccelerationWrite->setEnabled(false);
        ui->btnDecelerationWrite->setEnabled(false);

        ui->btnTest1->setEnabled(false);

        TPDO_object.setRunning(false);
        thread_TPDO.quit();

        servo_off();
        stop_server();

        for(uint8_t i = 0; i < ec_slavecount; i ++)
        {
            ui->tableWidget->horizontalHeaderItem(i + 2)->setBackground(Qt::yellow);
        }


        start_flag = false;
        servo_enable_flag = false;
    }
    else
    {

        ui->btnStartServer->setText("STOP");
        ui->btnServoON->setEnabled(true);
        ui->btnReset->setEnabled(true);

        ui->btnOperationModeWrite->setEnabled(true);
        ui->btnTargetPositionWrite->setEnabled(true);
        ui->btnMaxMotorVelocity->setEnabled(true);
        ui->btnTargetVelocityWrite->setEnabled(true);
        ui->btnAccelerationWrite->setEnabled(true);
        ui->btnDecelerationWrite->setEnabled(true);

        start_server(ifname);

        TPDO_object.setRunning(true);
        thread_TPDO.start();

        start_flag = true;
    }
}

void MainWindow::on_btnServoON_clicked()
{
    if(servo_enable_flag)
    {
        ui->btnServoON->setText("ServoOFF");
        ui->btnServoON->setStyleSheet(QString::fromUtf8("background-color: rgb(224, 27, 36)"));

        servo_off();

        //ui->btnOperationModeWrite->setEnabled(false);
        //ui->btnTargetPositionWrite->setEnabled(false);
        //ui->btnMaxMotorVelocity->setEnabled(false);
        ui->btnQuickStop->setEnabled(false);
        ui->btnTest1->setEnabled(false);
        //ui->btnTest2->setEnabled(false);
        //ui->btnTest3->setEnabled(false);
        //ui->btnTest4->setEnabled(false);

        servo_enable_flag = false;
    }
    else
    {
        ui->btnServoON->setText("ServoON");
        ui->btnServoON->setStyleSheet(QString::fromUtf8("background-color: rgb(45, 213, 14)"));

        servo_on();

        //ui->btnOperationModeWrite->setEnabled(true);
        //ui->btnTargetPositionWrite->setEnabled(true);
        //ui->btnMaxMotorVelocity->setEnabled(true);
        ui->btnQuickStop->setEnabled(true);
        ui->btnTest1->setEnabled(true);
        //ui->btnTest2->setEnabled(true);
        //ui->btnTest3->setEnabled(true);
        //ui->btnTest4->setEnabled(true);

        servo_enable_flag = true;
    }
}

void MainWindow::on_btnOperationModeWrite_clicked()
{
    int8_t operation_mode = ui->lineOperationMode->text().toInt();
    qDebug() << "main window - operation_mode = " << operation_mode << "\n";
    set_operation_mode(operation_mode);
}


void MainWindow::on_btnMaxMotorVelocity_clicked()
{
    int32_t max_motor_speed = ui->lineMaxMotorVelocity->text().toInt();
    qDebug() << "main window - max_motor_speed = " << max_motor_speed << "\n";
    set_max_motor_speed(max_motor_speed);
}


void MainWindow::on_btnTargetPositionWrite_clicked()
{
    int32_t target_position = ui->lineTargetPosition->text().toInt();
    qDebug() << "main window - target_position = " << target_position << "\n";
    set_target_position(target_position);
}



void MainWindow::on_btnTargetVelocityWrite_clicked()
{
    int32_t target_velocity = ui->lineTargetVelocity->text().toInt();
    qDebug() << "main window - target_velocity = " << target_velocity<< "\n";
    set_target_velocity(target_velocity);
}



void MainWindow::on_btnAccelerationWrite_clicked()
{
    int32_t acceleration = ui->lineProfileAcceleration->text().toInt();
    qDebug() << "main window - acceleration = " << acceleration<< "\n";
    set_acceleration(acceleration);
}


void MainWindow::on_btnDecelerationWrite_clicked()
{
    int32_t deceleration = ui->lineProfileDeceleration->text().toInt();
    qDebug() << "main window - deceleration = " << deceleration<< "\n";
    set_deceleration(deceleration);
}

void MainWindow::on_btnQuickStop_clicked()
{
    quick_stop();
}


void MainWindow::on_btnReset_clicked()
{
    servo_reset();
}

void MainWindow::on_btnTest1_clicked()
{
    test1_flag = 1;
    (void) osal_thread_create(&thread4, stack64k * 4, (void *) &test1_func, NULL);
}

void MainWindow::getValue()
{
    char str[8][16] = {0};

    if(inOP)
    {
        for(uint8_t i = 0; i < ec_slavecount; i ++)
        {
            if(!ec_slave[i + 1].islost)
            {
                ui->tableWidget->horizontalHeaderItem(i + 2)->setBackground(Qt::green);
            }
            else ui->tableWidget->horizontalHeaderItem(i + 2)->setBackground(Qt::red);

            sprintf(&str[0][0], "0x%x", ptr_input[i]->value_6041);
            sprintf(&str[1][0], "%d", ptr_input[i]->value_6064);
            sprintf(&str[2][0], "%d", ptr_input[i]->value_606C);
            sprintf(&str[3][0], "%d", ptr_input[i]->value_6077);
            sprintf(&str[4][0], "%d", ptr_input[i]->value_6062);
            sprintf(&str[5][0], "%d", ptr_input[i]->value_606B);
            sprintf(&str[6][0], "%d", ptr_input[i]->value_6074);
            sprintf(&str[7][0], "0x%x", ptr_input[i]->value_603F);

            for(uint8_t j = 0; j < 8 ; j ++)
            {
                ui->tableWidget->setItem(j, i + 2, new QTableWidgetItem(&str[j][0]));
            }
        } 
    }
}
