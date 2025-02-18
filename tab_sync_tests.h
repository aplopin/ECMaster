#ifndef TAB_SYNC_TESTS_H
#define TAB_SYNC_TESTS_H

#include <QWidget>

namespace Ui {
class tab_sync_tests;
}

class tab_sync_tests : public QWidget
{
    Q_OBJECT

public:
    explicit tab_sync_tests(QWidget *parent = nullptr);
    ~tab_sync_tests();

private slots:
    void on_btnTest1_clicked();

private:
    Ui::tab_sync_tests *ui;
};

#endif // TAB_SYNC_TESTS_H
