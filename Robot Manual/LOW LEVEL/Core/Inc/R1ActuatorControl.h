#ifndef R1_ACTUATOR_CONTROL_H
#define R1_ACTUATOR_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int8_t speedTongkat;
extern float posTongkatRAW;
extern uint8_t isLEDon;

void setPn1(float val);
void setPn2(float val);
void setPn3(float val);
void initPneumaticState(void);

void oldPneumatikMotor(int8_t speed, float rawPos);
void setMotorTongkat(int8_t speed);
void setServoTongkat(float rawPos);
void setLED(uint8_t isOn);

#ifdef __cplusplus
}
#endif

#endif /* R1_ACTUATOR_CONTROL_H */
