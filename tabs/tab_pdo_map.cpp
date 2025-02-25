#include "tab_pdo_map.h"
#include "ui_tab_pdo_map.h"
#include "slaveinfo/slaveinfo.h"

tab_pdo_map::tab_pdo_map(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::tab_pdo_map)
{
    ui->setupUi(this);
}

tab_pdo_map::~tab_pdo_map()
{
    delete ui;
}

void tab_pdo_map::on_btnSlaveinfo_clicked()
{
    slaveinfo_start();
}

