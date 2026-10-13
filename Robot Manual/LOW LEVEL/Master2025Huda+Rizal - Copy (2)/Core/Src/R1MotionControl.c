#include "R1MotionControl.h"

#include "Swerve.h"

#define R1_MOTION_STOP_BY_HEADING 1
#define R1_MOTION_DENAH_CENTER_X_OFFSET_MM 400.0f
#define R1_MOTION_DENAH_CENTER_Y_OFFSET_MM 470.0f

static R1MotionArenaMode_t r1_motion_arena_mode = R1_MOTION_ARENA_BLUE;

volatile float R1_MOTION_DBG_DenahXBase = 0.0f;
volatile float R1_MOTION_DBG_DenahY = 0.0f;
volatile float R1_MOTION_DBG_OdomXBase = 0.0f;
volatile float R1_MOTION_DBG_OdomY = 0.0f;
volatile float R1_MOTION_DBG_ArenaX = 0.0f;
volatile float R1_MOTION_DBG_ArenaY = 0.0f;

const R1MotionProfileConfig_t R1Motion_ProfileMeihuaManual = {
    .max_output_speed = 2000.0f,
    .start_output_speed = 1000.0f,

    .arrive_distance_mm = 150.0f,
    .arrive_heading_deg = 5.0f,

    .pid_distance_limit_mm = 500.0f,
    .pid_heading_limit_deg = 30.0f,

    .kp_distance = 35.0f,
    .kd_distance = 5.0f,

    .kp_heading = 1.2f,
    .kd_heading = 0.0f,
};

const R1MotionProfileConfig_t R1Motion_ProfileNormal = {
    .max_output_speed = 2000.0f,
    .start_output_speed = 1000.0f,

    .arrive_distance_mm = 200.0f,
    .arrive_heading_deg = 5.0f,

    .pid_distance_limit_mm = 700.0f,
    .pid_heading_limit_deg = 45.0f,

    .kp_distance = 40.0f,
    .kd_distance = 5.0f,

    .kp_heading = 1.0f,
    .kd_heading = 0.0f,
};

const R1MotionProfileConfig_t R1Motion_ProfilePrecise = {
    .max_output_speed = 800.0f,
    .start_output_speed = 600.0f,

    .arrive_distance_mm = 80.0f,
    .arrive_heading_deg = 5.0f,

    .pid_distance_limit_mm = 500.0f,
    .pid_heading_limit_deg = 30.0f,

    .kp_distance = 35.0f,
    .kd_distance = 15.0f,

    .kp_heading = 1.2f,
    .kd_heading = 0.0f,
};

const R1MotionProfileConfig_t R1Motion_ProfileFast = {
    .max_output_speed = 3000.0f,
    .start_output_speed = 1200.0f,

    .arrive_distance_mm = 250.0f,
    .arrive_heading_deg = 7.0f,

    .pid_distance_limit_mm = 900.0f,
    .pid_heading_limit_deg = 45.0f,

    .kp_distance = 45.0f,
    .kd_distance = 10.0f,

    .kp_heading = 1.0f,
    .kd_heading = 0.0f,
};

const R1MotionProfileConfig_t R1Motion_ProfileFastPrecise = {
    .max_output_speed = 3000.0f,
    .start_output_speed = 1000.0f,

    .arrive_distance_mm = 100.0f,
    .arrive_heading_deg = 5.0f,

    .pid_distance_limit_mm = 700.0f,
    .pid_heading_limit_deg = 45.0f,

    .kp_distance = 40.0f,
    .kd_distance = 13.0f,

    .kp_heading = 1.0f,
    .kd_heading = 0.0f,
};

void R1Motion_SetArenaMode(R1MotionArenaMode_t mode) {
  if (mode == R1_MOTION_ARENA_RED) {
    r1_motion_arena_mode = R1_MOTION_ARENA_RED;
  } else {
    r1_motion_arena_mode = R1_MOTION_ARENA_BLUE;
  }
}

void R1Motion_ApplyProfile(const R1MotionProfileConfig_t* profile) {
  if (profile == 0) {
    profile = &R1Motion_ProfileMeihuaManual;
  }

  MaxSpeed = profile->max_output_speed;
  LimitErrorJ = profile->pid_distance_limit_mm;
  LimitErrorH = profile->pid_heading_limit_deg;

  KPj = profile->kp_distance;
  KDj = profile->kd_distance;

  KPh = profile->kp_heading;
  KDh = profile->kd_heading;
}

float R1Motion_NormalizeHeading(float angle_deg) {
  while (angle_deg > 180.0f) {
    angle_deg -= 360.0f;
  }

  while (angle_deg < -180.0f) {
    angle_deg += 360.0f;
  }

  return angle_deg;
}

float R1Motion_AddHeading90(float heading_deg, int8_t step_count) {
  float result =
      R1Motion_NormalizeHeading(heading_deg + (90.0f * (float)step_count));

  if (result <= -180.0f) {
    result = 180.0f;
  }

  return result;
}

static float R1Motion_ToArenaX(float base_x) {
  if (r1_motion_arena_mode == R1_MOTION_ARENA_RED) {
    return -base_x;
  }

  return base_x;
}

static float R1Motion_ToArenaH(float base_h) {
  if (r1_motion_arena_mode == R1_MOTION_ARENA_RED) {
    return R1Motion_NormalizeHeading(-base_h);
  }

  return R1Motion_NormalizeHeading(base_h);
}

void R1Motion_TurnHeadingBase(float heading_base,
                              const R1MotionProfileConfig_t* profile) {
  float heading_arena;

  if (profile == 0) {
    profile = &R1Motion_ProfileMeihuaManual;
  }

  heading_arena = R1Motion_ToArenaH(heading_base);

  R1Motion_ApplyProfile(profile);
  JalanKeTitikRIZAL(PosX_World, PosY_World, heading_arena,
                    (int)profile->arrive_heading_deg,
                    R1_MOTION_STOP_BY_HEADING,
                    (int)profile->start_output_speed);
  RodaStop();
}

void R1Motion_GoToPointFinalHeadingBase(
    float x_base,
    float y_mm,
    float heading_base,
    const R1MotionProfileConfig_t* profile) {
  float x_arena;
  float heading_arena;

  if (profile == 0) {
    profile = &R1Motion_ProfileMeihuaManual;
  }

  x_arena = R1Motion_ToArenaX(x_base);
  heading_arena = R1Motion_ToArenaH(heading_base);

  R1Motion_ApplyProfile(profile);
  JalanKeTitikFinalHeadingRIZAL(x_arena,
                                y_mm,
                                heading_arena,
                                profile->arrive_distance_mm,
                                profile->arrive_heading_deg,
                                (int)profile->start_output_speed);
  RodaBrakeDynamicRIZAL(0.7f, 60.0f, 35U);
  RodaStop();
}

void R1Motion_GoToDenahPointFinalHeadingBase(
    float x_denah_base,
    float y_denah,
    float heading_base,
    const R1MotionProfileConfig_t* profile) {
  float x_odom_base;
  float y_odom;

  /*
   * Meihua points use BLUE field-denah coordinates from left/back walls.
   * Odometry still starts at robot-center (0,0), so target center is offset.
   */
  x_odom_base = x_denah_base - R1_MOTION_DENAH_CENTER_X_OFFSET_MM;
  y_odom = y_denah - R1_MOTION_DENAH_CENTER_Y_OFFSET_MM + 2000.0f;

  R1_MOTION_DBG_DenahXBase = x_denah_base;
  R1_MOTION_DBG_DenahY = y_denah;
  R1_MOTION_DBG_OdomXBase = x_odom_base;
  R1_MOTION_DBG_OdomY = y_odom;
  R1_MOTION_DBG_ArenaX = R1Motion_ToArenaX(x_odom_base);
  R1_MOTION_DBG_ArenaY = y_odom;

  R1Motion_GoToPointFinalHeadingBase(x_odom_base,
                                     y_odom,
                                     heading_base,
                                     profile);
}
