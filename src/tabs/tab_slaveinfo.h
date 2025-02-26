#ifndef TAB_SLAVEINFO_H
#define TAB_SLAVEINFO_H

#include <QWidget>
#include <inttypes.h>
#include <stdio.h>
#include "ethercat.h"

#define OTYPE_VAR               0x0007
#define OTYPE_ARRAY             0x0008
#define OTYPE_RECORD            0x0009

#define ATYPE_Rpre              0x01
#define ATYPE_Rsafe             0x02
#define ATYPE_Rop               0x04
#define ATYPE_Wpre              0x08
#define ATYPE_Wsafe             0x10
#define ATYPE_Wop               0x20

namespace Ui {
class tab_slaveinfo;
}

class tab_slaveinfo : public QWidget
{
    Q_OBJECT

private:
    char IOmap[4096];
    ec_ODlistt ODlist;
    ec_OElistt OElist;
    bool printSDO = FALSE;
    bool printMAP = FALSE;
    char usdo[128];
    char ifbuf[1024] = {"eno1"};

public:
    explicit tab_slaveinfo(QWidget *parent = nullptr);
    ~tab_slaveinfo();

private:
    char* dtype2string(uint16 dtype, uint16 bitlen);
    char* otype2string(uint16 otype);
    char* access2string(uint16 access);
    char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype);
    int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
    int si_map_sdo(int slave);
    int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
    int si_map_sii(int slave);
    void si_sdo(int cnt);
    void slaveinfo(char *ifname);
    void slaveinfo_start(void);

private slots:
    void on_btnSlaveinfo_clicked();

    void on_btnClearPlain_clicked();

private:
    Ui::tab_slaveinfo *ui;
};

#endif // TAB_SLAVEINFO_H
