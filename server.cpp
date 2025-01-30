#include "server.h"

char IOmap[4096];
pthread_t thread1, thread2, thread3, thread4;
int dorun = 0;
int do_flag = 1;
int64 toff, gl_delta;
volatile int wkc;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile boolean inOP;
uint8 currentgroup = 0;

volatile bool start_flag = 0;
volatile bool servo_enable_flag = 0;
volatile bool test1_flag = 0;
const char ifname[] = {"eno1"};

volatile slave_in_t * ptr_input[SERVDCOUNT];
static slave_out_t * ptr_output[SERVDCOUNT];

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

OSAL_THREAD_FUNC ecatcheck(void *ptr )
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

OSAL_THREAD_FUNC read_pdo(void)
{
    while(1)
    {
        if(inOP == TRUE)
        {
            printf("Slave - 1:\n");
            printf("value_6041 - 0x%x\n", ptr_input[0]->value_6041);
            printf("value_6064 - %d\n", ptr_input[0]->value_6064);
            printf("value_606C - %d\n", ptr_input[0]->value_606C);
            printf("value_6077 - %d\n", ptr_input[0]->value_6077);
            printf("value_6062 - %d\n", ptr_input[0]->value_6062);
            printf("value_606B - %d\n", ptr_input[0]->value_606B);
            printf("value_6074 - %d\n", ptr_input[0]->value_6074);
            printf("value_603F - 0x%x\n\n", ptr_input[0]->value_603F);

            printf("Slave - 2:\n");
            printf("value_6041 - 0x%x\n", ptr_input[1]->value_6041);
            printf("value_6064 - %d\n", ptr_input[1]->value_6064);
            printf("value_606C - %d\n", ptr_input[1]->value_606C);
            printf("value_6077 - %d\n", ptr_input[1]->value_6077);
            printf("value_6062 - %d\n", ptr_input[1]->value_6062);
            printf("value_606B - %d\n", ptr_input[1]->value_606B);
            printf("value_6074 - %d\n", ptr_input[1]->value_6074);
            printf("value_603F - 0x%x\n\n", ptr_input[1]->value_603F);
        }

        if(do_flag == 0) break;

        osal_usleep(250000);
    }
}

OSAL_THREAD_FUNC test1_func(void)
{
    while(1)
    {
        if(test1_flag == 1)
        {
            set_operation_mode(3);

            set_max_motor_speed(250000);
            set_acceleration(125000);
            set_deceleration(125000);

            // Speed task block
            set_target_velocity(250000);
            osal_usleep(2000000);
            set_target_velocity(0);
            osal_usleep(2000000);
            set_target_velocity(-250000);
            osal_usleep(2000000);
            set_target_velocity(0);
            osal_usleep(2000000);
            set_target_velocity(125000);
            osal_usleep(1000000);
            set_target_velocity(0);
            osal_usleep(1000000);
            set_target_velocity(-125000);
            osal_usleep(1000000);
            set_target_velocity(0);
            osal_usleep(1000000);

            set_max_motor_speed(500000);
            set_acceleration(500000);
            set_deceleration(500000);

            set_target_velocity(500000);
            osal_usleep(1000000);
            set_target_velocity(0);
            osal_usleep(1000000);
            set_target_velocity(-500000);
            osal_usleep(1000000);
            set_target_velocity(0);
            osal_usleep(1000000);
            set_target_velocity(250000);
            osal_usleep(500000);
            set_target_velocity(0);
            osal_usleep(500000);
            set_target_velocity(-250000);
            osal_usleep(500000);
            set_target_velocity(0);
            osal_usleep(500000);

            test1_flag = 0;
        }

        if(test1_flag == 0) break;

        osal_usleep(250000);
    }
}

void user_pdo_map_config(void)
{
    /* Map velocity PDO assignment via Complete Access*/
    uint16 map_1c12[2] = {0x0001, 0x1600}; //RPDO
    uint16 map_1c13[2] = {0x0001, 0x1A00}; //TPDO

    for(uint8_t i = 0; i < ec_slavecount; i ++)
    {
        (void) ec_SDOwrite(i + 1, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        (void) ec_SDOwrite(i + 1, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    }
}

void start_server(const char *ifname)
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
            printf("%d slaves found and configured.\n",ec_slavecount);

            /* PDO device map setting */
            user_pdo_map_config();

            /* define structres slave_input_t for data input */
            for(uint8_t i = 0; i < ec_slavecount; i ++)
            {
                ptr_input[i] = (slave_in_t*)ec_slave[i + 1].inputs;
                //printf("address = %d;\n", ptr_input[i]);
            }

            /* define structres slave_output_t for data output */
            for(uint8_t i = 0; i < ec_slavecount; i ++)
            {
                ptr_output[i] = (slave_out_t*)ec_slave[i + 1].outputs;
                //printf("address = %d;\n", ptr_output[i]);
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

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0000;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0006;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0007;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x000F;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void servo_off(void)
{
    printf("Servo is disabled!\n");

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0000;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void servo_reset(void)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0080;
    }
    ec_send_processdata();
    osal_usleep(CTIME);

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0000;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_target_position(int32_t target_position)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_607A = target_position;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_max_motor_speed(int32_t max_motor_speed)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6080 = max_motor_speed;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_target_velocity(int32_t target_velocity)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_60FF = target_velocity;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_operation_mode(int8_t operation_mode)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6060 = operation_mode;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_acceleration(int32_t acceleration)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6083 = acceleration;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void set_deceleration(int32_t deceleration)
{
    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6084 = deceleration;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}

void quick_stop(void)
{
    printf("Quick Stop!\n");

    for(int8_t i = 0; i < ec_slavecount; i ++)
    {
        ptr_output[i]->value_6040 = 0x0002;
    }
    ec_send_processdata();
    osal_usleep(CTIME);
}
