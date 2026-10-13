#include "R1MeihuaManual.h"

#include "R1MotionControl.h"
#include "Swerve.h"
#include "joystickRizal.h"

typedef struct {
  float x_base;
  float y_mm;
} R1MeihuaManualPoint_t;

// kiri X 187 + offest odom 400 = 587, 600 - 587 = 13
// bawah y 1955 + offset odom 470 = 2425, 2425 - 2000 = 425, 600 - 425 = 175
// kanan X 4866 + offset odom 400 = 5266, 5400 - 5266 = 134

#define R1_MEIHUA_X_KANAN_BASE_MM 5400.0f
#define R1_MEIHUA_X_KIRI_BASE_MM   600.0f
#define R1_MEIHUA_Y_ATAS_BASE_MM  6600.0f
#define R1_MEIHUA_Y_BAWAH_BASE_MM  600.0f

#define R1_MEIHUA_OFFSET_KANAN_X_MM -134.0f
#define R1_MEIHUA_OFFSET_KIRI_X_MM  -13.0f
#define R1_MEIHUA_OFFSET_ATAS_Y_MM  -150.0f
#define R1_MEIHUA_OFFSET_BAWAH_Y_MM -175.0f

#define R1_MEIHUA_X_KANAN \
  (R1_MEIHUA_X_KANAN_BASE_MM + R1_MEIHUA_OFFSET_KANAN_X_MM)
#define R1_MEIHUA_X_KIRI \
  (R1_MEIHUA_X_KIRI_BASE_MM + R1_MEIHUA_OFFSET_KIRI_X_MM)
#define R1_MEIHUA_Y_ATAS \
  (R1_MEIHUA_Y_ATAS_BASE_MM + R1_MEIHUA_OFFSET_ATAS_Y_MM)
#define R1_MEIHUA_Y_BAWAH \
  (R1_MEIHUA_Y_BAWAH_BASE_MM + R1_MEIHUA_OFFSET_BAWAH_Y_MM)

// point 1, atau paliing kanan bawah X, Y (5400, 600)
// point 6, atau paliing kanan atas X, Y (5400, 6600)
// point 10, atau paliing kiri atas X, Y (600, 6600)
// point 15, atau paliing kiri bawah X, Y (600, 600)

/*
 * Titik disimpan sebagai koordinat denah BASE BLUE dari tembok kiri/belakang.
 * R1MotionControl mengubah target denah ke target odometry center robot.
 * Untuk arena RED, R1MotionControl mirror X dan heading tanpa planner lama.
 */
static const R1MeihuaManualPoint_t R1MeihuaManual_Points
    [R1_MEIHUA_MANUAL_POINT_COUNT] = {
        {R1_MEIHUA_X_KANAN, R1_MEIHUA_Y_BAWAH}, // 1  Kanan / Bawah
        {R1_MEIHUA_X_KANAN, 1800.0f},           // 2  Kanan
        {R1_MEIHUA_X_KANAN, 3000.0f},           // 3  Kanan
        {R1_MEIHUA_X_KANAN, 4200.0f},           // 4  Kanan
        {R1_MEIHUA_X_KANAN, 5400.0f},           // 5  Kanan
        {R1_MEIHUA_X_KANAN, R1_MEIHUA_Y_ATAS},  // 6  Kanan / Atas
        {4200.0f, R1_MEIHUA_Y_ATAS},            // 7  Atas
        {3000.0f, R1_MEIHUA_Y_ATAS},            // 8  Atas
        {1800.0f, R1_MEIHUA_Y_ATAS},            // 9  Atas
        {R1_MEIHUA_X_KIRI, R1_MEIHUA_Y_ATAS},   // 10 Atas / Kiri
        {R1_MEIHUA_X_KIRI, 5400.0f},            // 11 Kiri
        {R1_MEIHUA_X_KIRI, 4200.0f},            // 12 Kiri
        {R1_MEIHUA_X_KIRI, 3000.0f},            // 13 Kiri
        {R1_MEIHUA_X_KIRI, 1800.0f},            // 14 Kiri
        {R1_MEIHUA_X_KIRI, R1_MEIHUA_Y_BAWAH},  // 15 Kiri / Bawah
        {1800.0f, R1_MEIHUA_Y_BAWAH},           // 16 Bawah
        {3000.0f, R1_MEIHUA_Y_BAWAH},           // 17 Bawah
        {4200.0f, R1_MEIHUA_Y_BAWAH},           // 18 Bawah
};

volatile uint8_t R1_MEIHUA_DBG_Active = 0u;
volatile uint8_t R1_MEIHUA_DBG_CurrentPoint = 1u;
volatile uint8_t R1_MEIHUA_DBG_LastExitPoint = 1u;
volatile uint8_t R1_MEIHUA_DBG_HasLastExit = 0u;
volatile uint8_t R1_MEIHUA_DBG_PointTableReady = 0u;
volatile float R1_MEIHUA_DBG_CurrentHeading = 0.0f;
volatile float R1_MEIHUA_DBG_LastExitHeading = 0.0f;
volatile float R1_MEIHUA_DBG_TargetXBase = 0.0f;
volatile float R1_MEIHUA_DBG_TargetY = 0.0f;

static uint8_t r1_meihua_current_point = 1u;
static uint8_t r1_meihua_has_current_point = 0u;
static uint8_t r1_meihua_step_input_latched = 0u;
static float r1_meihua_current_heading = 0.0f;

static uint8_t R1MeihuaManual_ClampPoint(uint8_t point) {
  if (point < 1u) {
    return 1u;
  }

  if (point > R1_MEIHUA_MANUAL_POINT_COUNT) {
    return R1_MEIHUA_MANUAL_POINT_COUNT;
  }

  return point;
}

static uint8_t R1MeihuaManual_IsPointTableReady(void) {
  uint8_t i;

  for (i = 0u; i < R1_MEIHUA_MANUAL_POINT_COUNT; i++) {
    if ((R1MeihuaManual_Points[i].x_base != 0.0f) ||
        (R1MeihuaManual_Points[i].y_mm != 0.0f)) {
      return 1u;
    }
  }

  return 0u;
}

static uint8_t R1MeihuaManual_StepInputActive(void) {
  if ((JOYS_L_KANAN != 0) || (JOYS_L_KIRI != 0)) {
    return 1u;
  }

  return 0u;
}

static void R1MeihuaManual_UpdateDebugTarget(void) {
  uint8_t index = R1MeihuaManual_ClampPoint(r1_meihua_current_point) - 1u;

  R1_MEIHUA_DBG_CurrentPoint = r1_meihua_current_point;
  R1_MEIHUA_DBG_CurrentHeading = r1_meihua_current_heading;
  R1_MEIHUA_DBG_LastExitPoint = 0u;
  R1_MEIHUA_DBG_LastExitHeading = 0.0f;
  R1_MEIHUA_DBG_HasLastExit = 0u;
  R1_MEIHUA_DBG_PointTableReady = R1MeihuaManual_IsPointTableReady();
  R1_MEIHUA_DBG_TargetXBase = R1MeihuaManual_Points[index].x_base;
  R1_MEIHUA_DBG_TargetY = R1MeihuaManual_Points[index].y_mm;
}

static void R1MeihuaManual_SelectNextPoint(void) {
  if (r1_meihua_has_current_point == 0u) {
    r1_meihua_current_point = 1u;
    r1_meihua_has_current_point = 1u;
    return;
  }

  if (r1_meihua_current_point < R1_MEIHUA_MANUAL_POINT_COUNT) {
    r1_meihua_current_point++;
  } else {
    r1_meihua_current_point = 1u;
  }
}

static void R1MeihuaManual_SelectPrevPoint(void) {
  if (r1_meihua_has_current_point == 0u) {
    r1_meihua_current_point = R1_MEIHUA_MANUAL_POINT_COUNT;
    r1_meihua_has_current_point = 1u;
    return;
  }

  if (r1_meihua_current_point > 1u) {
    r1_meihua_current_point--;
  } else {
    r1_meihua_current_point = R1_MEIHUA_MANUAL_POINT_COUNT;
  }
}

static void R1MeihuaManual_GoToCurrentPoint(void) {
  uint8_t index;

  r1_meihua_current_point =
      R1MeihuaManual_ClampPoint(r1_meihua_current_point);
  index = r1_meihua_current_point - 1u;

  R1MeihuaManual_UpdateDebugTarget();

  R1Motion_GoToDenahPointFinalHeadingBase(
      R1MeihuaManual_Points[index].x_base,
      R1MeihuaManual_Points[index].y_mm,
      r1_meihua_current_heading,
      &R1Motion_ProfileMeihuaManual);
}

uint8_t R1MeihuaManual_ProcessJoystickStep(void) {
  R1MeihuaManual_UpdateDebugTarget();

  if (R1MeihuaManual_StepInputActive() == 0u) {
    r1_meihua_step_input_latched = 0u;
    R1_MEIHUA_DBG_Active = 0u;
    return 0u;
  }

  if (r1_meihua_step_input_latched != 0u) {
    return 1u;
  }

  r1_meihua_step_input_latched = 1u;

  if (JOYS_L_KANAN != 0) {
    R1MeihuaManual_SelectNextPoint();
  } else {
    R1MeihuaManual_SelectPrevPoint();
  }

  R1MeihuaManual_UpdateDebugTarget();
  if (R1_MEIHUA_DBG_PointTableReady == 0u) {
    R1_MEIHUA_DBG_Active = 0u;
    RodaStop();
    return 1u;
  }

  R1_MEIHUA_DBG_Active = 1u;
  R1MeihuaManual_GoToCurrentPoint();
  R1_MEIHUA_DBG_Active = 0u;

  return 1u;
}
