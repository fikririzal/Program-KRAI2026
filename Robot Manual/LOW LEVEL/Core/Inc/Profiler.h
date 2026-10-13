#ifndef PROFILER_H
#define PROFILER_H

#include "main.h"
#include <stdint.h>

/*
 * ============================================================
 * R1 SECTION PROFILER + TASK2 DEEP DEBUG
 * ============================================================
 * Enable/disable lewat compiler symbol:
 *   R1_PROFILER_ENABLE=1   profiler aktif
 *   R1_PROFILER_ENABLE=0   profiler mati
 * Kalau symbol tidak dibuat, default = 1 untuk build debug ini.
 */
#ifndef R1_PROFILER_ENABLE
#define R1_PROFILER_ENABLE 1
#endif

/* Section profiler IDs. Jangan ubah urutan ID lama agar log lama tetap mudah dibaca. */
typedef enum
{
    PROF_TASK1_LOOP = 0,
    PROF_TASK1_JOY_DEADZONE,

    PROF_TASK2_LOOP,
    PROF_SEQ_BASE_CURRENT,
    PROF_SEQ_BASE_SLOW,

    PROF_UART_SENDER_FRAME,
    PROF_UART_DMA_START,
    PROF_UART_WAIT_DONE,

    PROF_IMU_LOOP,
    PROF_IMU_PARSE_BYTE,

    PROF_JOY_DMA_CHECK,
    PROF_JOY_READ_LIKE_ARDUINO,
    PROF_JOY_PARSE_PACKET,

    PROF_JOYSTICK_RIZAL,
    PROF_SWERVE_KINEMATIK,
    PROF_SWERVE_RODA_STOP,
    PROF_USART_SET_RODA,
    PROF_USART_QUEUE_PUT,

    PROF_DYN_MOVE_SERVO,
    PROF_DYN_SET_SPEED,

    /* Generic blocking sequence macro profiler. */
    PROF_SEQ_WAIT_PRESS,
    PROF_SEQ_WAIT_RELEASE,
    PROF_SEQ_WAIT_ANY_PRESS,
    PROF_SEQ_DELAY,

    /* Task2 high-level branch profiler. */
    PROF_T2_INIT,
    PROF_T2_MODE_SELECT,
    PROF_T2_AUTONOM_BRANCH,
    PROF_T2_AUTO_CROSS_TOTAL,
    PROF_T2_MANUAL_BRANCH,
    PROF_T2_MANUAL_SQUARE_TOTAL,
    PROF_T2_MANUAL_TRIANGLE_TOTAL,
    PROF_T2_MANUAL_CROSS_TOTAL,
    PROF_T2_MANUAL_LIFTER_INPUT,
    PROF_T2_MANUAL_BASE_CURRENT,
    PROF_T2_MISC_SHARE_TOTAL,

	PROF_ODOM_RIZAL,

    PROF_COUNT
} ProfId_t;

/* Task2 state code untuk melihat posisi terakhir saat loop aktif lama. */
typedef enum
{
    T2_STATE_BOOT = 0,
    T2_STATE_INIT = 10,
    T2_STATE_MODE_SELECT = 20,

    T2_STATE_LOOP_BEGIN = 100,
    T2_STATE_AUTONOM_BRANCH = 200,
    T2_STATE_AUTO_CROSS = 210,
    T2_STATE_AUTO_JOY_FAST = 220,
    T2_STATE_AUTO_JOY_SLOW = 221,

    T2_STATE_MANUAL_BRANCH = 300,
    T2_STATE_MANUAL_SQUARE = 310,
    T2_STATE_MANUAL_TRIANGLE = 320,
    T2_STATE_MANUAL_TRIANGLE_FOREST = 321,
    T2_STATE_MANUAL_TRIANGLE_MAGAZINE = 322,
    T2_STATE_MANUAL_TRIANGLE_GRIPPER = 323,
    T2_STATE_MANUAL_TRIANGLE_FLOOR = 324,
    T2_STATE_MANUAL_CROSS = 330,
    T2_STATE_MANUAL_LIFTER_INPUT = 340,
    T2_STATE_MANUAL_BASE_CURRENT = 350,
    T2_STATE_MISC_SHARE = 360,
    T2_STATE_LOOP_END = 999,
} Task2StateCode_t;

typedef enum
{
    T2_WAIT_NONE = 0,
    T2_WAIT_PRESS = 1,
    T2_WAIT_RELEASE = 2,
    T2_WAIT_ANY_PRESS = 3,
    T2_WAIT_DELAY = 4,
} Task2WaitKind_t;

#if (R1_PROFILER_ENABLE != 0)

typedef struct
{
    const char *name;
    uint32_t warn_ms;

    volatile uint32_t hit_count;
    volatile uint32_t over_count;
    volatile uint32_t reenter_count;

    volatile uint32_t last_us;
    volatile uint32_t max_us;
    volatile uint32_t last_ms;
    volatile uint32_t max_ms;

    volatile uint8_t active;
    volatile uint32_t enter_ms;
    volatile uint32_t enter_cycle;
    volatile uint32_t active_ms;
} ProfRec_t;

extern ProfRec_t g_prof[PROF_COUNT];

extern volatile uint32_t prof_top_max_id;
extern volatile uint32_t prof_top_max_ms;
extern volatile uint32_t prof_top_active_id;
extern volatile uint32_t prof_top_active_ms;
extern volatile uint32_t prof_print_count;

/* Task2 debug variables, bisa dilihat di Live Expressions. */
extern volatile uint32_t t2_dbg_state;
extern volatile uint32_t t2_dbg_state_line;
extern volatile uint32_t t2_dbg_state_tick;
extern volatile uint32_t t2_dbg_loop_active;
extern volatile uint32_t t2_dbg_loop_enter_ms;
extern volatile uint32_t t2_dbg_loop_last_ms;
extern volatile uint32_t t2_dbg_loop_max_ms;
extern volatile uint32_t t2_dbg_loop_over_count;
extern volatile uint32_t t2_dbg_loop_line;
extern volatile uint32_t t2_dbg_loop_max_state;
extern volatile uint32_t t2_dbg_loop_max_state_line;
extern volatile uint32_t t2_dbg_loop_max_wait_kind;
extern volatile uint32_t t2_dbg_loop_max_wait_line;
extern volatile uint32_t t2_dbg_loop_active_ms;
extern volatile uint32_t t2_dbg_wait_kind;
extern volatile uint32_t t2_dbg_wait_line;
extern volatile uint32_t t2_dbg_wait_tick;
extern volatile uint32_t t2_dbg_wait_active_ms;
extern volatile uint32_t t2_dbg_wait_max_ms;
extern volatile uint32_t t2_dbg_wait_max_kind;
extern volatile uint32_t t2_dbg_wait_max_line;

void Prof_Init(void);
void Prof_Begin(ProfId_t id);
void Prof_End(ProfId_t id);
void Prof_Reset(void);
void Prof_PrintReport(void);

void Prof_Task2SetState(uint32_t state, uint32_t line);
void Prof_Task2LoopBegin(uint32_t line);
void Prof_Task2LoopEnd(uint32_t line);
void Prof_Task2WaitBegin(uint32_t wait_kind, uint32_t line);
void Prof_Task2WaitEnd(uint32_t line);
void Prof_Task2PrintDebug(void);

#define PROF_BEGIN(ID)          Prof_Begin((ID))
#define PROF_END(ID)            Prof_End((ID))
#define PROF_PRINT()            Prof_PrintReport()
#define PROF_INIT()             Prof_Init()
#define PROF_RESET()            Prof_Reset()

#define T2_DBG_STATE(STATE)     Prof_Task2SetState((uint32_t)(STATE), (uint32_t)__LINE__)
#define T2_DBG_LOOP_BEGIN()     Prof_Task2LoopBegin((uint32_t)__LINE__)
#define T2_DBG_LOOP_END()       Prof_Task2LoopEnd((uint32_t)__LINE__)
#define T2_DBG_WAIT_BEGIN(KIND) Prof_Task2WaitBegin((uint32_t)(KIND), (uint32_t)__LINE__)
#define T2_DBG_WAIT_END()       Prof_Task2WaitEnd((uint32_t)__LINE__)

#else

#define PROF_BEGIN(ID)          do { (void)(ID); } while (0)
#define PROF_END(ID)            do { (void)(ID); } while (0)
#define PROF_PRINT()            do { } while (0)
#define PROF_INIT()             do { } while (0)
#define PROF_RESET()            do { } while (0)
#define T2_DBG_STATE(STATE)     do { (void)(STATE); } while (0)
#define T2_DBG_LOOP_BEGIN()     do { } while (0)
#define T2_DBG_LOOP_END()       do { } while (0)
#define T2_DBG_WAIT_BEGIN(KIND) do { (void)(KIND); } while (0)
#define T2_DBG_WAIT_END()       do { } while (0)

static inline void Prof_Init(void)        { }
static inline void Prof_Begin(ProfId_t id){ (void)id; }
static inline void Prof_End(ProfId_t id)  { (void)id; }
static inline void Prof_Reset(void)       { }
static inline void Prof_PrintReport(void) { }
static inline void Prof_Task2SetState(uint32_t state, uint32_t line) { (void)state; (void)line; }
static inline void Prof_Task2LoopBegin(uint32_t line) { (void)line; }
static inline void Prof_Task2LoopEnd(uint32_t line) { (void)line; }
static inline void Prof_Task2WaitBegin(uint32_t kind, uint32_t line) { (void)kind; (void)line; }
static inline void Prof_Task2WaitEnd(uint32_t line) { (void)line; }
static inline void Prof_Task2PrintDebug(void) { }

#endif

#endif /* PROFILER_H */
