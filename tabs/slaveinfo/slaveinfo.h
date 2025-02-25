#ifndef SLAVEINFO_H
#define SLAVEINFO_H

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

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

#endif // SLAVEINFO_H
