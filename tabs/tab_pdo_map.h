#ifndef TAB_PDO_MAP_H
#define TAB_PDO_MAP_H

#include <QWidget>

namespace Ui {
class tab_pdo_map;
}

class tab_pdo_map : public QWidget
{
    Q_OBJECT

public:
    explicit tab_pdo_map(QWidget *parent = nullptr);
    ~tab_pdo_map();

private slots:
    void on_btnSlaveinfo_clicked();

private:
    Ui::tab_pdo_map *ui;
};

#endif // TAB_PDO_MAP_H
