#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QString"
#include <iostream>
#include <ostream>
#include <QTableWidget>
#include <QObject>
#include <QFont>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>

#include "osal.h"
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

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define stack64k (64 * 1024)
#define CTIME 500

char IOmap[4096];
ec_ODlistt ODlist;
ec_OElistt OElist;
boolean printSDO = FALSE;
boolean printMAP = FALSE;
char usdo[128];

pthread_t thread1, thread2, thread3;
int dorun = 0;
int do_flag = 1;
int64 toff, gl_delta;
volatile int wkc;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
boolean inOP;
uint8 currentgroup = 0;

bool start_flag = 0;
bool servo_enable_flag = 0;
char ifname[] = {"eno1"};

typedef struct PACKED
{
    uint16_t value_6040; // Control Word
    int32_t value_607A; // Target Position
    uint32_t value_6080; // Max Motor Speed
    int32_t value_60FF; // Target Velocity
    int32_t value_6083; // Profile Acceleration
    int32_t value_6084; // Profile Decceleration
    int8_t value_6060; // Operation Mode

} slave_out_t;

typedef struct PACKED
{
    uint16_t value_6041; // Status Word
    int32_t value_6064; // Actual Motor Position
    int32_t value_606C; // Actual Motor Velocity
    int16_t value_6077; // Actual Motor Torque
    int32_t value_6062; // Commanded Position
    int32_t value_606B; // Commanded Velocity
    int16_t value_6074; // Commanded Torque
    uint16_t value_603F; // Last Error Code

} slave_in_t;

slave_in_t * ptr_input[8];

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if ( ts->tv_nsec >= NSEC_PER_SEC )
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if(delta> (cycletime / 2)) { delta= delta - cycletime; }
    if(delta>0){ integral++; }
    if(delta<0){ integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec   ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    ec_send_processdata();
    while(1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            dorun++;
            /* if we have some digital output, cycle */
            if( digout ) *digout = (uint8) ((dorun / 16) & 0xff);

            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }
            ec_send_processdata();
        }

        if(do_flag == 0) break;
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;

    (void) ptr;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave ++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            //if(!ec_group[currentgroup].docheckstate)
            //printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);

        if(do_flag == 0) break;
    }
}

/*
OSAL_THREAD_FUNC read_pdo(void)
{
    while(1)
    {
        if(inOP == TRUE)
        {
            printf("value_6041 - 0x%x\n", ptr_input->value_6041);
            printf("value_6064 - %d\n", ptr_input->value_6064);
            printf("value_606C - %d\n", ptr_input->value_606C);
            printf("value_6077 - %d\n", ptr_input->value_6077);
            printf("value_6062 - %d\n", ptr_input->value_6062);
            printf("value_606B - %d\n", ptr_input->value_606B);
            printf("value_6074 - %d\n", ptr_input->value_6074);
            printf("value_603F - 0x%x\n\n", ptr_input->value_603F);
        }

        if(do_flag == 0) break;

        osal_usleep(250000);
    }
}
*/

void user_pdo_map_config(void)
{
    int retval;
    //uint16 u16val;

    retval = 0;

    /* Map velocity PDO assignment via Complete Access*/
    uint16 map_1c12[2] = {0x0001, 0x1600}; //RPDO
    uint16 map_1c13[2] = {0x0001, 0x1A00}; //TPDO

    retval += ec_SDOwrite(1, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
    retval += ec_SDOwrite(1, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

    //printf("Slave %d set, retval = %d\n", slave, retval);
}

void start_server(char *ifname)
{
    int cnt, i, oloop, iloop, ctime;
    printf("SOEM (Simple Open EtherCAT Master)\nEtherCAT server!\n");

    dorun = 0;
    do_flag = 1;
    ctime = CTIME; //us

    /* create RT thread */
    (void) osal_thread_create_rt(&thread1, stack64k * 8, (void *) &ecatthread, (void*) &ctime);

    /* create thread to handle slave error handling in OP */
    (void) osal_thread_create(&thread2, stack64k * 4, (void *) &ecatcheck, NULL);

    /* create thread to read and show slave TPDO */
    //(void) osal_thread_create(&thread3, stack64k * 4, (void *) &read_pdo, NULL);

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */
        if ( ec_config(FALSE, &IOmap) > 0 )
        {
            user_pdo_map_config();

            printf("%d slaves found and configured.\n",ec_slavecount);

            /* define structres slave_input_t for data input */
            for(uint8_t i = 0; i < ec_slavecount; i ++)
            {
                ptr_input[i] = (slave_in_t*)ec_slave[i].inputs;
            }

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

            /* configure DC options for every DC capable slave found in the list */
            ec_configdc();

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                printf("         Out:%p,%4d In:%p,%4d\n",
                       ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
                /* check for EL2004 or EL2008 */
                if( !digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052)))
                {
                    digout = ec_slave[cnt].outputs;
                }
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* request OP state for all slaves */
            ec_writestate(0);
            /* activate cyclic process data */
            dorun = 1;
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i <= ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

void stop_server(void)
{
    dorun = 0;
    inOP = FALSE;
    do_flag = 0;

    printf("Request INIT state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);

    printf("Stop EtherCAT server, close socket\n");
    /* stop SOEM, close socket */
    ec_close();
}

void servo_on(void)
{
    printf("Servo is enabled!\n");

    slave_out_t *ptr_output[ec_slavecount];

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x0000;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x0006;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x0007;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x000F;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void servo_off(void)
{
    printf("Servo is disabled!\n");

    slave_out_t *ptr_output[ec_slavecount];

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x0000;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void servo_reset(void)
{
    slave_out_t *ptr_output[ec_slavecount];

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x0080;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;

        ptr_output[i]->value_6040 = 0x0000;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_target_position(int32_t target_position)
{
    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_607A = target_position;
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_max_motor_speed(int32_t max_motor_speed)
{
    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_6080 = max_motor_speed;
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_target_velocity(int32_t target_velocity)
{
    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_60FF = target_velocity;
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_operation_mode(int8_t operation_mode)
{
    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_6060 = operation_mode;
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_acceleration(int32_t acceleration)
{
    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_6083 = acceleration;
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_deceleration(int32_t deceleration)
{
    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_6084 = deceleration;
    ec_send_processdata();
    osal_usleep(CTIME);
}

void quick_stop(void)
{
    printf("Quick Stop!\n");

    slave_out_t * ptr_output = (slave_out_t*)ec_slave[1].outputs;

    ptr_output->value_6040 = 0x0002;
    ec_send_processdata();
    osal_usleep(CTIME);
}


//--------------------------------------------------------------
//slaveinfo
//--------------------------------------------------------------

char* dtype2string(uint16 dtype, uint16 bitlen)
{
    static char str[32] = { 0 };

    switch(dtype)
    {
    case ECT_BOOLEAN:
        sprintf(str, "BOOLEAN");
        break;
    case ECT_INTEGER8:
        sprintf(str, "INTEGER8");
        break;
    case ECT_INTEGER16:
        sprintf(str, "INTEGER16");
        break;
    case ECT_INTEGER32:
        sprintf(str, "INTEGER32");
        break;
    case ECT_INTEGER24:
        sprintf(str, "INTEGER24");
        break;
    case ECT_INTEGER64:
        sprintf(str, "INTEGER64");
        break;
    case ECT_UNSIGNED8:
        sprintf(str, "UNSIGNED8");
        break;
    case ECT_UNSIGNED16:
        sprintf(str, "UNSIGNED16");
        break;
    case ECT_UNSIGNED32:
        sprintf(str, "UNSIGNED32");
        break;
    case ECT_UNSIGNED24:
        sprintf(str, "UNSIGNED24");
        break;
    case ECT_UNSIGNED64:
        sprintf(str, "UNSIGNED64");
        break;
    case ECT_REAL32:
        sprintf(str, "REAL32");
        break;
    case ECT_REAL64:
        sprintf(str, "REAL64");
        break;
    case ECT_BIT1:
        sprintf(str, "BIT1");
        break;
    case ECT_BIT2:
        sprintf(str, "BIT2");
        break;
    case ECT_BIT3:
        sprintf(str, "BIT3");
        break;
    case ECT_BIT4:
        sprintf(str, "BIT4");
        break;
    case ECT_BIT5:
        sprintf(str, "BIT5");
        break;
    case ECT_BIT6:
        sprintf(str, "BIT6");
        break;
    case ECT_BIT7:
        sprintf(str, "BIT7");
        break;
    case ECT_BIT8:
        sprintf(str, "BIT8");
        break;
    case ECT_VISIBLE_STRING:
        sprintf(str, "VISIBLE_STR(%d)", bitlen);
        break;
    case ECT_OCTET_STRING:
        sprintf(str, "OCTET_STR(%d)", bitlen);
        break;
    default:
        sprintf(str, "dt:0x%4.4X (%d)", dtype, bitlen);
    }
    return str;
}

char* otype2string(uint16 otype)
{
    static char str[32] = { 0 };

    switch(otype)
    {
    case OTYPE_VAR:
        sprintf(str, "VAR");
        break;
    case OTYPE_ARRAY:
        sprintf(str, "ARRAY");
        break;
    case OTYPE_RECORD:
        sprintf(str, "RECORD");
        break;
    default:
        sprintf(str, "ot:0x%4.4X", otype);
    }
    return str;
}

char* access2string(uint16 access)
{
    static char str[32] = { 0 };

    sprintf(str, "%s%s%s%s%s%s",
            ((access & ATYPE_Rpre) != 0 ? "R" : "_"),
            ((access & ATYPE_Wpre) != 0 ? "W" : "_"),
            ((access & ATYPE_Rsafe) != 0 ? "R" : "_"),
            ((access & ATYPE_Wsafe) != 0 ? "W" : "_"),
            ((access & ATYPE_Rop) != 0 ? "R" : "_"),
            ((access & ATYPE_Wop) != 0 ? "W" : "_"));
    return str;
}

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
    int l = sizeof(usdo) - 1, i;
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
    if (EcatError)
    {
        return ec_elist2string();
    }
    else
    {
        static char str[64] = { 0 };
        switch(dtype)
        {
        case ECT_BOOLEAN:
            u8 = (uint8*) &usdo[0];
            if (*u8) sprintf(str, "TRUE");
            else sprintf(str, "FALSE");
            break;
        case ECT_INTEGER8:
            i8 = (int8*) &usdo[0];
            sprintf(str, "0x%2.2x / %d", *i8, *i8);
            break;
        case ECT_INTEGER16:
            i16 = (int16*) &usdo[0];
            sprintf(str, "0x%4.4x / %d", *i16, *i16);
            break;
        case ECT_INTEGER32:
        case ECT_INTEGER24:
            i32 = (int32*) &usdo[0];
            sprintf(str, "0x%8.8x / %d", *i32, *i32);
            break;
        case ECT_INTEGER64:
            //i64 = (int64*) &usdo[0];
            //sprintf(str, "0x%16.16"PRIx64" / %"PRId64, *i64, *i64);
            break;
        case ECT_UNSIGNED8:
            u8 = (uint8*) &usdo[0];
            sprintf(str, "0x%2.2x / %u", *u8, *u8);
            break;
        case ECT_UNSIGNED16:
            u16 = (uint16*) &usdo[0];
            sprintf(str, "0x%4.4x / %u", *u16, *u16);
            break;
        case ECT_UNSIGNED32:
        case ECT_UNSIGNED24:
            u32 = (uint32*) &usdo[0];
            sprintf(str, "0x%8.8x / %u", *u32, *u32);
            break;
        case ECT_UNSIGNED64:
            //u64 = (uint64*) &usdo[0];
            //sprintf(str, "0x%16.16"PRIx64" / %"PRIu64, *u64, *u64);
            break;
        case ECT_REAL32:
            sr = (float*) &usdo[0];
            sprintf(str, "%f", *sr);
            break;
        case ECT_REAL64:
            dr = (double*) &usdo[0];
            sprintf(str, "%f", *dr);
            break;
        case ECT_BIT1:
        case ECT_BIT2:
        case ECT_BIT3:
        case ECT_BIT4:
        case ECT_BIT5:
        case ECT_BIT6:
        case ECT_BIT7:
        case ECT_BIT8:
            u8 = (uint8*) &usdo[0];
            sprintf(str, "0x%x / %u", *u8, *u8);
            break;
        case ECT_VISIBLE_STRING:
            strcpy(str, "\"");
            strcat(str, usdo);
            strcat(str, "\"");
            break;
        case ECT_OCTET_STRING:
            str[0] = 0x00;
            for (i = 0 ; i < l ; i++)
            {
                sprintf(es, "0x%2.2x ", usdo[i]);
                strcat( str, es);
            }
            break;
        default:
            sprintf(str, "Unknown type");
        }
        return str;
    }
}

/** Read PDO assign structure */
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;
    int abs_offset, abs_bit;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
        /* number of available sub indexes */
        nidx = rdat;
        bsize = 0;
        /* read all PDO's */
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            rdl = sizeof(rdat); rdat = 0;
            /* read PDO assign */
            wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            /* result is index of PDO */
            idx = etohs(rdat);
            if (idx > 0)
            {
                rdl = sizeof(subcnt); subcnt = 0;
                /* read number of subindexes of PDO */
                wkc = ec_SDOread(slave,idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                subidx = subcnt;
                /* for each subindex */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    rdl = sizeof(rdat2); rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    bitlen = LO_BYTE(rdat2);
                    bsize += bitlen;
                    obj_idx = (uint16)(rdat2 >> 16);
                    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;
                    ODlist.Slave = slave;
                    ODlist.Index[0] = obj_idx;
                    OElist.Entries = 0;
                    wkc = 0;
                    /* read object entry from dictionary if not a filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                        wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                    printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                    if((wkc > 0) && OElist.Entries)
                    {
                        printf(" %-12s %s\n", dtype2string(OElist.DataType[obj_subidx], bitlen), OElist.Name[obj_subidx]);
                    }
                    else
                        printf("\n");
                    bitoffset += bitlen;
                };
            };
        };
    };
    /* return total found bitlength (PDO) */
    return bsize;
}

int si_map_sdo(int slave)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;

    printf("PDO mapping according to CoE :\n");
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM); nSM = 0;
    /* read SyncManager Communication Type object count */
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > EC_MAXSM)
            nSM = EC_MAXSM;
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            rdl = sizeof(tSM); tSM = 0;
            /* read SyncManager Communication Type */
            wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if (wkc > 0)
            {
                if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                    printf("Activated SM type workaround, possible incorrect mapping.\n");
                }
                if(tSM)
                    tSM += SMt_bug_add; // only add if SMt > 0

                if (tSM == 3) // outputs
                {
                    /* read the assign RXPDO */
                    printf("  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap[0]), outputs_bo );
                    outputs_bo += Tsize;
                }
                if (tSM == 4) // inputs
                {
                    /* read the assign TXPDO */
                    printf("  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap[0]), inputs_bo );
                    inputs_bo += Tsize;
                }
            }
        }
    }

    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset)
{
    uint16 a , w, c, e, er;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;

    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
        a = PDO->Startpos;
        w = ec_siigetbyte(slave, a++);
        w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
        /* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
            PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            /* number of entries in PDO */
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
            a++;
            obj_name = ec_siigetbyte(slave, a++);
            a += 2;
            c += 2;
            if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
            {
                str_name[0] = 0;
                if(obj_name)
                    ec_siistring(str_name, slave, obj_name);
                if (t)
                    printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                else
                    printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                printf("     addr b   index: sub bitl data_type    name\n");
                /* read all entries defined in PDO */
                for (er = 1; er <= e; er++)
                {
                    c += 4;
                    obj_idx = ec_siigetbyte(slave, a++);
                    obj_idx += (ec_siigetbyte(slave, a++) << 8);
                    obj_subidx = ec_siigetbyte(slave, a++);
                    obj_name = ec_siigetbyte(slave, a++);
                    obj_datatype = ec_siigetbyte(slave, a++);
                    bitlen = ec_siigetbyte(slave, a++);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;

                    PDO->BitSize[PDO->nPDO] += bitlen;
                    a += 2;

                    /* skip entry if filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                    {
                        str_name[0] = 0;
                        if(obj_name)
                            ec_siistring(str_name, slave, obj_name);

                        printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                        printf(" %-12s %s\n", dtype2string(obj_datatype, bitlen), str_name);
                    }
                    bitoffset += bitlen;
                    totalsize += bitlen;
                }
                PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
                c++;
            }
            else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
            {
                c += 4 * e;
                a += 8 * e;
                c++;
            }
            if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
        }
        while (c < PDO->Length);
    }
    if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}


int si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    printf("PDO mapping according to SII :\n");

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8*)&IOmap), outputs_bo );
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8*)&IOmap), inputs_bo );
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

void si_sdo(int cnt)
{
    int i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if( ec_readODlist(cnt, &ODlist))
    {
        printf(" CoE Object Description found, %d entries.\n",ODlist.Entries);
        for( i = 0 ; i < ODlist.Entries ; i++)
        {
            uint8_t max_sub;
            char name[128] = { 0 };

            ec_readODdescription(i, &ODlist);
            while(EcatError) printf(" - %s\n", ec_elist2string());
            snprintf(name, sizeof(name) - 1, "\"%s\"", ODlist.Name[i]);
            if (ODlist.ObjectCode[i] == OTYPE_VAR)
            {
                printf("0x%04x      %-40s      [%s]\n", ODlist.Index[i], name,
                       otype2string(ODlist.ObjectCode[i]));
            }
            else
            {
                printf("0x%04x      %-40s      [%s  maxsub(0x%02x / %d)]\n",
                       ODlist.Index[i], name, otype2string(ODlist.ObjectCode[i]),
                       ODlist.MaxSub[i], ODlist.MaxSub[i]);
            }
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(i, &ODlist, &OElist);
            while(EcatError) printf("- %s\n", ec_elist2string());

            if(ODlist.ObjectCode[i] != OTYPE_VAR)
            {
                int l = sizeof(max_sub);
                ec_SDOread(cnt, ODlist.Index[i], 0, FALSE, &l, &max_sub, EC_TIMEOUTRXM);
            }
            else {
                max_sub = ODlist.MaxSub[i];
            }

            for( j = 0 ; j < max_sub+1 ; j++)
            {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
                {
                    snprintf(name, sizeof(name) - 1, "\"%s\"", OElist.Name[j]);
                    printf("    0x%02x      %-40s      [%-16s %6s]      ", j, name,
                           dtype2string(OElist.DataType[j], OElist.BitLength[j]),
                           access2string(OElist.ObjAccess[j]));
                    if ((OElist.ObjAccess[j] & 0x0007))
                    {
                        printf("%s", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
                    }
                    printf("\n");
                }
            }
        }
    }
    else
    {
        while(EcatError) printf("%s", ec_elist2string());
    }
}

void slaveinfo(char *ifname)
{
    int cnt, i, j, nSM;
    uint16 ssigen;
    int expectedWKC;

    printf("Starting slaveinfo\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */
        if ( ec_config(FALSE, &IOmap) > 0 )
        {
            ec_configdc();
            while(EcatError) printf("%s", ec_elist2string());
            printf("%d slaves found and configured.\n",ec_slavecount);
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
            if (ec_slave[0].state != EC_STATE_SAFE_OP )
            {
                printf("Not all slaves reached safe operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_SAFE_OP)
                    {
                        printf("Slave %d State=%2x StatusCode=%4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }


            ec_readstate();
            for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
                printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
                       (ec_slave[cnt].activeports & 0x02) > 0 ,
                       (ec_slave[cnt].activeports & 0x04) > 0 ,
                       (ec_slave[cnt].activeports & 0x08) > 0 );
                printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
                printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
                for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
                {
                    if(ec_slave[cnt].SM[nSM].StartAddr > 0)
                        printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, etohs(ec_slave[cnt].SM[nSM].StartAddr), etohs(ec_slave[cnt].SM[nSM].SMlength),
                               etohl(ec_slave[cnt].SM[nSM].SMflags), ec_slave[cnt].SMtype[nSM]);
                }
                for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
                {
                    printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                           etohl(ec_slave[cnt].FMMU[j].LogStart), etohs(ec_slave[cnt].FMMU[j].LogLength), ec_slave[cnt].FMMU[j].LogStartbit,
                           ec_slave[cnt].FMMU[j].LogEndbit, etohs(ec_slave[cnt].FMMU[j].PhysStart), ec_slave[cnt].FMMU[j].PhysStartBit,
                           ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
                }
                printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                       ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
                printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
                ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
                /* SII general section */
                if (ssigen)
                {
                    ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
                    ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
                    ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
                    ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
                    if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
                    {
                        ec_slave[cnt].blockLRW = 1;
                        ec_slave[0].blockLRW++;
                    }
                    ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
                    ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
                    ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
                }
                printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
                       ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
                printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
                       ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
                if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
                    si_sdo(cnt);
                if(printMAP)
                {
                    if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                        si_map_sdo(cnt);
                    else
                        si_map_sii(cnt);
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("\nRequest init state for all slaves\n");
        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);
        printf("End slaveinfo, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

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
    ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "Register name" << "Adress" << "Axis1" << "Axis2" << "Axis3" << "Axis4"\
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

        TPDO_object.setRunning(false);

        stop_server();
        //if(...) to servo_off();

        start_flag = false;
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

void MainWindow::getValue()
{
    char str[8][16] = {0};

    if(inOP)
    {
        for(uint8_t i = 0; i < ec_slavecount; i ++)
        {
            sprintf(&str[0][0], "0x%x", ptr_input[i]->value_6041);
            sprintf(&str[1][0], "%d", ptr_input[i]->value_6064);
            sprintf(&str[2][0], "%d", ptr_input[i]->value_606C);
            sprintf(&str[3][0], "%d", ptr_input[i]->value_6077);
            sprintf(&str[4][0], "%d", ptr_input[i]->value_6062);
            sprintf(&str[5][0], "%d", ptr_input[i]->value_606B);
            sprintf(&str[6][0], "%d", ptr_input[i]->value_6074);
            sprintf(&str[7][0], "0x%x", ptr_input[i]->value_603F);

            for(uint8_t j; j < 8 ; j ++)
            {
                ui->tableWidget->setItem(j, i + 3, new QTableWidgetItem(&str[i][0]));
            }
        }
    }
}


