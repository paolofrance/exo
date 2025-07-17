#pragma once

#include <iostream>
#include <semaphore.h>
#include <fcntl.h>
#include <thread>
#include <signal.h>
#include <math.h>

#define SHMEM_RX_LEN            4
#define SHMEM_RX_PACKET_SIZE    SHMEM_RX_LEN*sizeof(float)
#define SHMEM_TX_LEN            2
#define SHMEM_TX_PACKET_SIZE    SHMEM_TX_LEN*sizeof(float)
#define SHMEM_RX_OFFSET         0
#define SHMEM_TX_OFFSET         SHMEM_RX_PACKET_SIZE

sem_t* sem_master_impedance;
sem_t* sem_client_impedance;
sem_t* sem_master_feedback;
sem_t* sem_client_feedback;
sem_t* sem_sync;

const float ref_45deg = 0.7854f;
const float ref_90deg = 1.5708f;
const float ref_limit = 2.1000f;

const float odrive_frequency = 1000.0f; // 1 kHz
const float master_frequency = 100.0f;  // 100 Hz

void shmem_init(void);
