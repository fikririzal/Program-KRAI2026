#ifndef R1_MEIHUA_MANUAL_H
#define R1_MEIHUA_MANUAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define R1_MEIHUA_MANUAL_POINT_COUNT 18u

extern volatile uint8_t R1_MEIHUA_DBG_Active;
extern volatile uint8_t R1_MEIHUA_DBG_CurrentPoint;
extern volatile uint8_t R1_MEIHUA_DBG_LastExitPoint;
extern volatile uint8_t R1_MEIHUA_DBG_HasLastExit;
extern volatile uint8_t R1_MEIHUA_DBG_PointTableReady;
extern volatile float R1_MEIHUA_DBG_CurrentHeading;
extern volatile float R1_MEIHUA_DBG_LastExitHeading;
extern volatile float R1_MEIHUA_DBG_TargetXBase;
extern volatile float R1_MEIHUA_DBG_TargetY;

uint8_t R1MeihuaManual_ProcessJoystickStep(void);

#ifdef __cplusplus
}
#endif

#endif /* R1_MEIHUA_MANUAL_H */
