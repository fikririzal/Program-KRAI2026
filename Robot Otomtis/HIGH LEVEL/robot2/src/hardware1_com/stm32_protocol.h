#ifndef STM32_PROTOCOL_H
#define STM32_PROTOCOL_H

#include <stdint.h>

/* Data diterima dari Jetson (TX Jetson → RX STM32) */
extern volatile int16_t g_vx;
extern volatile int16_t g_vy;
extern volatile int16_t g_wz;
extern volatile int16_t g_lifter_robot1;
extern volatile int16_t g_lifter_robot2;
extern volatile int16_t g_sudut_turet_utama;
extern volatile int16_t g_sudut_turet1;
extern volatile int16_t g_sudut_turet2;
extern volatile int16_t g_lifter_turet1;
extern volatile int16_t g_lifter_turet2;
extern volatile int16_t g_griper1;
extern volatile int16_t g_griper2;
extern volatile int16_t g_griper3;
extern volatile int16_t g_griper4;
extern volatile int16_t g_sudut_dynamixel1;
extern volatile int16_t g_sudut_dynamixel2;
extern volatile int16_t g_lifter_base_belakang;

/* Panggil setiap byte datang dari UART (interrupt/DMA/polling) */
void protocol_rx_byte(uint8_t byte);

/* Alternatif: buffer penuh dari DMA */
void protocol_rx_buffer(const uint8_t *buf, uint16_t len);

#endif /* STM32_PROTOCOL_H */
