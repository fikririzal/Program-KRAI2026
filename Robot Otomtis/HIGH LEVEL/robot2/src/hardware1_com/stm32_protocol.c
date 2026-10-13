/*
 * stm32_protocol.c
 * Taruh file ini di project STM32.
 * Panggil protocol_rx_byte() dari UART interrupt / DMA callback.
 *
 * Frame dari Jetson — 36 byte:
 *  [0]     0xAA
 *  [1]     0x55
 *  [2]     0x01  (type)
 *  [3-4]   vx              int16 LE
 *  [5-6]   vy              int16 LE
 *  [7-8]   wz              int16 LE
 *  [9-10]  lifterRobot2    int16 LE
 *  [11-12] lifterRobot1    int16 LE
 *  [13-14] sudutTuretUtama int16 LE
 *  [15-16] sudutTuret1     int16 LE
 *  [17-18] sudutTuret2     int16 LE
 *  [19-20] lifterTuret1    int16 LE
 *  [21-22] lifterTuret2    int16 LE
 *  [23-24] griper1         int16 LE
 *  [25-26] griper2         int16 LE
 *  [27-28] griper3         int16 LE
 *  [29-30] griper4         int16 LE
 *  [31-32] sudutDynamixel1 int16 LE
 *  [33-34] sudutDynamixel2    int16 LE
 *  [35-36] lifterBaseBelakang int16 LE
 *  [37]    checksum = sum([0..36]) % 256
 */

#include "stm32_protocol.h"

#define FRAME_LEN  38
#define BUF_SIZE   72   /* 2× frame — cukup untuk sliding window */

static uint8_t  rx_buf[BUF_SIZE];
static uint16_t rx_head = 0;

volatile int16_t g_vx                = 0;
volatile int16_t g_vy                = 0;
volatile int16_t g_wz                = 0;
volatile int16_t g_lifter_robot1     = 0;
volatile int16_t g_lifter_robot2     = 0;
volatile int16_t g_sudut_turet_utama = 0;
volatile int16_t g_sudut_turet1      = 0;
volatile int16_t g_sudut_turet2      = 0;
volatile int16_t g_lifter_turet1     = 0;
volatile int16_t g_lifter_turet2     = 0;
volatile int16_t g_griper1           = 0;
volatile int16_t g_griper2           = 0;
volatile int16_t g_griper3           = 0;
volatile int16_t g_griper4           = 0;
volatile int16_t g_sudut_dynamixel1    = 0;
volatile int16_t g_sudut_dynamixel2    = 0;
volatile int16_t g_lifter_base_belakang = 0;

/* ------------------------------------------------------------------ */

static uint8_t calc_checksum(const uint8_t *f)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 1; i++) sum += f[i];
    return (uint8_t)(sum % 256);
}

static int16_t get16(const uint8_t *f, uint8_t idx)
{
    return (int16_t)((uint16_t)f[idx] | ((uint16_t)f[idx + 1] << 8));
}

static void parse(const uint8_t *f)
{
    g_vx                = get16(f, 3);
    g_vy                = get16(f, 5);
    g_wz                = get16(f, 7);
    g_lifter_robot2     = get16(f, 9);
    g_lifter_robot1     = get16(f, 11);
    g_sudut_turet_utama = get16(f, 13);
    g_sudut_turet1      = get16(f, 15);
    g_sudut_turet2      = get16(f, 17);
    g_lifter_turet1     = get16(f, 19);
    g_lifter_turet2     = get16(f, 21);
    g_griper1           = get16(f, 23);
    g_griper2           = get16(f, 25);
    g_griper3           = get16(f, 27);
    g_griper4           = get16(f, 29);
    g_sudut_dynamixel1    = get16(f, 31);
    g_sudut_dynamixel2    = get16(f, 33);
    g_lifter_base_belakang = get16(f, 35);
}

/* ------------------------------------------------------------------ */

void protocol_rx_byte(uint8_t byte)
{
    rx_buf[rx_head % BUF_SIZE] = byte;
    rx_head++;

    if (rx_head < FRAME_LEN) return;

    uint16_t start = (rx_head >= BUF_SIZE) ? rx_head - BUF_SIZE : 0;

    for (uint16_t i = start; i + FRAME_LEN <= rx_head; i++) {
        uint8_t frame[FRAME_LEN];
        for (uint8_t k = 0; k < FRAME_LEN; k++) {
            frame[k] = rx_buf[(i + k) % BUF_SIZE];
        }

        if (frame[0] != 0xAA || frame[1] != 0x55 || frame[2] != 0x01) continue;
        if (frame[FRAME_LEN - 1] != calc_checksum(frame))               continue;

        parse(frame);
        rx_head = 0;
        break;
    }
}

void protocol_rx_buffer(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        protocol_rx_byte(data[i]);
    }
}
