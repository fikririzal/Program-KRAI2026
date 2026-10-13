#ifndef R1_MOTION_CONTROL_H
#define R1_MOTION_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  R1_MOTION_ARENA_BLUE = 0,
  R1_MOTION_ARENA_RED = 1
} R1MotionArenaMode_t;

typedef struct {
  float max_output_speed;
  float start_output_speed;

  float arrive_distance_mm;
  float arrive_heading_deg;

  float pid_distance_limit_mm;
  float pid_heading_limit_deg;

  float kp_distance;
  float kd_distance;

  float kp_heading;
  float kd_heading;
} R1MotionProfileConfig_t;

extern const R1MotionProfileConfig_t R1Motion_ProfileMeihuaManual;
extern const R1MotionProfileConfig_t R1Motion_ProfileNormal;
extern const R1MotionProfileConfig_t R1Motion_ProfilePrecise;
extern const R1MotionProfileConfig_t R1Motion_ProfileFast;
extern const R1MotionProfileConfig_t R1Motion_ProfileFastPrecise;

extern volatile float R1_MOTION_DBG_DenahXBase;
extern volatile float R1_MOTION_DBG_DenahY;
extern volatile float R1_MOTION_DBG_OdomXBase;
extern volatile float R1_MOTION_DBG_OdomY;
extern volatile float R1_MOTION_DBG_ArenaX;
extern volatile float R1_MOTION_DBG_ArenaY;

void R1Motion_SetArenaMode(R1MotionArenaMode_t mode);
void R1Motion_ApplyProfile(const R1MotionProfileConfig_t* profile);

float R1Motion_NormalizeHeading(float angle_deg);
float R1Motion_AddHeading90(float heading_deg, int8_t step_count);

void R1Motion_TurnHeadingBase(float heading_base,
                              const R1MotionProfileConfig_t* profile);
void R1Motion_GoToPointFinalHeadingBase(
    float x_base,
    float y_mm,
    float heading_base,
    const R1MotionProfileConfig_t* profile);
void R1Motion_GoToDenahPointFinalHeadingBase(
    float x_denah_base,
    float y_denah,
    float heading_base,
    const R1MotionProfileConfig_t* profile);

#ifdef __cplusplus
}
#endif

#endif /* R1_MOTION_CONTROL_H */
