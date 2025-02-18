#include "tab_sync_tests.h"
#include "ui_tab_sync_tests.h"
#include "server.h"

tab_sync_tests::tab_sync_tests(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::tab_sync_tests)
{
    ui->setupUi(this);
}

tab_sync_tests::~tab_sync_tests()
{
    delete ui;
}

void tab_sync_tests::on_btnTest1_clicked()
{
    test1_flag = 1;
    (void) osal_thread_create(&thread4, stack64k * 4, (void *) &test1_func, NULL);
}
