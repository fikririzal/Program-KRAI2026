#include "R1ActuatorControl.h"

#include "USART.h"

#define R1_ACT_PNEUMATIC_INIT_PN1 0.0f
#define R1_ACT_PNEUMATIC_INIT_PN2 1.0f
#define R1_ACT_PNEUMATIC_INIT_PN3 0.0f

#define R1_ACT_TONGKAT_HOME_RAW 300.0f

static float r1_act_pn1 = R1_ACT_PNEUMATIC_INIT_PN1;
static float r1_act_pn2 = R1_ACT_PNEUMATIC_INIT_PN2;
static float r1_act_pn3 = R1_ACT_PNEUMATIC_INIT_PN3;

int8_t speedTongkat = 0;
float posTongkatRAW = R1_ACT_TONGKAT_HOME_RAW;
uint8_t isLEDon = 0;

static void sendPneumatic(void) {
  Pneumatic(r1_act_pn1, r1_act_pn2, r1_act_pn3);
}

void setPn1(float val) {
  r1_act_pn1 = val;
  sendPneumatic();
}

void setPn2(float val) {
  r1_act_pn2 = val;
  sendPneumatic();
}

void setPn3(float val) {
  r1_act_pn3 = val;
  sendPneumatic();
}

void initPneumaticState(void) {
  r1_act_pn1 = R1_ACT_PNEUMATIC_INIT_PN1;
  r1_act_pn2 = R1_ACT_PNEUMATIC_INIT_PN2;
  r1_act_pn3 = R1_ACT_PNEUMATIC_INIT_PN3;
  sendPneumatic();
}

void oldPneumatikMotor(int8_t speed, float rawPos) {
  speedTongkat = speed;
  posTongkatRAW = rawPos;
  PneumaticMotor(speedTongkat, posTongkatRAW, isLEDon);
}

void setMotorTongkat(int8_t speed) {
  speedTongkat = speed;
  PneumaticMotor(speedTongkat, posTongkatRAW, isLEDon);
}

void setServoTongkat(float rawPos) {
  posTongkatRAW = rawPos;
  PneumaticMotor(speedTongkat, posTongkatRAW, isLEDon);
}

void setLED(uint8_t isOn) {
  isLEDon = isOn;
  PneumaticMotor(speedTongkat, posTongkatRAW, isLEDon);
}
