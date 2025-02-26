#ifndef SERVER_H
#define SERVER_H

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>

#include "ethercat.h"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define stack64k (64 * 1024)
#define CTIME 500
#define SERVDCOUNT 8

extern volatile boolean inOP;

extern volatile bool start_flag;
extern volatile bool servo_enable_flag;
extern volatile bool test1_flag;
extern const char ifname[];

extern pthread_t thread4;

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

extern volatile slave_in_t * ptr_input[SERVDCOUNT];

void add_timespec(struct timespec *ts, int64 addtime);
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);
OSAL_THREAD_FUNC_RT ecatthread(void *ptr);
OSAL_THREAD_FUNC ecatcheck( void *ptr );
OSAL_THREAD_FUNC read_pdo(void);
OSAL_THREAD_FUNC test1_func(void);
void user_pdo_map_config(void);
void start_server(const char *ifname);
void stop_server(void);
void servo_on(void);
void servo_off(void);
void servo_reset(void);
void set_target_position(int32_t target_position);
void set_max_motor_speed(int32_t max_motor_speed);
void set_target_velocity(int32_t target_velocity);
void set_operation_mode(int8_t operation_mode);
void set_acceleration(int32_t acceleration);
void set_deceleration(int32_t deceleration);
void quick_stop(void);

#endif // SERVER_H
