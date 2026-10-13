#include "Profiler.h"

#if (R1_PROFILER_ENABLE != 0)

#include <stdio.h>

ProfRec_t g_prof[PROF_COUNT] =
{
    [PROF_TASK1_LOOP]                 = { "Task1.Loop",        20 },
    [PROF_TASK1_JOY_DEADZONE]         = { "Task1.JoyDeadzone",  5 },

    [PROF_TASK2_LOOP]                 = { "Task2.Loop",        30 },
    [PROF_SEQ_BASE_CURRENT]           = { "SEQ.BaseCurrent",   15 },
    [PROF_SEQ_BASE_SLOW]              = { "SEQ.BaseSlow",      15 },

    [PROF_UART_SENDER_FRAME]          = { "UART.Frame",        30 },
    [PROF_UART_DMA_START]             = { "UART.DMA_Start",     5 },
    [PROF_UART_WAIT_DONE]             = { "UART.WaitDone",     25 },

    [PROF_IMU_LOOP]                   = { "IMU.Loop",          20 },
    [PROF_IMU_PARSE_BYTE]             = { "IMU.ParseByte",      3 },

    [PROF_JOY_DMA_CHECK]              = { "Joy.DMA_Check",      8 },
    [PROF_JOY_READ_LIKE_ARDUINO]      = { "Joy.ReadArduino",    8 },
    [PROF_JOY_PARSE_PACKET]           = { "Joy.ParsePacket",    3 },

    [PROF_JOYSTICK_RIZAL]             = { "Joy.HeadingRizal",  15 },
    [PROF_SWERVE_KINEMATIK]           = { "Swerve.Kinematik",   8 },
    [PROF_SWERVE_RODA_STOP]           = { "Swerve.RodaStop",   15 },
    [PROF_USART_SET_RODA]             = { "USART.SetRoda",      5 },
    [PROF_USART_QUEUE_PUT]            = { "USART.QueuePut",     3 },

    [PROF_DYN_MOVE_SERVO]             = { "DYN.MoveServo",     25 },
    [PROF_DYN_SET_SPEED]              = { "DYN.SetSpeed",      25 },

    [PROF_SEQ_WAIT_PRESS]             = { "SEQ.WaitPress",    100 },
    [PROF_SEQ_WAIT_RELEASE]           = { "SEQ.WaitRelease",  100 },
    [PROF_SEQ_WAIT_ANY_PRESS]         = { "SEQ.WaitAnyPress", 100 },
    [PROF_SEQ_DELAY]                  = { "SEQ.Delay",        100 },

    [PROF_T2_INIT]                    = { "T2.Init",          100 },
    [PROF_T2_MODE_SELECT]             = { "T2.ModeSelect",    100 },
    [PROF_T2_AUTONOM_BRANCH]          = { "T2.Autonom",        30 },
    [PROF_T2_AUTO_CROSS_TOTAL]        = { "T2.Auto.Cross",    100 },
    [PROF_T2_MANUAL_BRANCH]           = { "T2.Manual",         30 },
    [PROF_T2_MANUAL_SQUARE_TOTAL]     = { "T2.Manual.Square",100 },
    [PROF_T2_MANUAL_TRIANGLE_TOTAL]   = { "T2.Manual.Tri",   100 },
    [PROF_T2_MANUAL_CROSS_TOTAL]      = { "T2.Manual.Cross", 100 },
    [PROF_T2_MANUAL_LIFTER_INPUT]     = { "T2.Manual.Lifter", 10 },
    [PROF_T2_MANUAL_BASE_CURRENT]     = { "T2.Manual.Base",   20 },
    [PROF_T2_MISC_SHARE_TOTAL]        = { "T2.MiscShare",    100 },

	[PROF_ODOM_RIZAL]        = { "OdomTask",    100 },
};

volatile uint32_t prof_top_max_id = 0;
volatile uint32_t prof_top_max_ms = 0;
volatile uint32_t prof_top_active_id = 0;
volatile uint32_t prof_top_active_ms = 0;
volatile uint32_t prof_print_count = 0;

volatile uint32_t t2_dbg_state = T2_STATE_BOOT;
volatile uint32_t t2_dbg_state_line = 0;
volatile uint32_t t2_dbg_state_tick = 0;
volatile uint32_t t2_dbg_loop_active = 0;
volatile uint32_t t2_dbg_loop_enter_ms = 0;
volatile uint32_t t2_dbg_loop_last_ms = 0;
volatile uint32_t t2_dbg_loop_max_ms = 0;
volatile uint32_t t2_dbg_loop_over_count = 0;
volatile uint32_t t2_dbg_loop_line = 0;
volatile uint32_t t2_dbg_loop_max_state = 0;
volatile uint32_t t2_dbg_loop_max_state_line = 0;
volatile uint32_t t2_dbg_loop_max_wait_kind = 0;
volatile uint32_t t2_dbg_loop_max_wait_line = 0;
volatile uint32_t t2_dbg_loop_active_ms = 0;
volatile uint32_t t2_dbg_wait_kind = T2_WAIT_NONE;
volatile uint32_t t2_dbg_wait_line = 0;
volatile uint32_t t2_dbg_wait_tick = 0;
volatile uint32_t t2_dbg_wait_active_ms = 0;
volatile uint32_t t2_dbg_wait_max_ms = 0;
volatile uint32_t t2_dbg_wait_max_kind = 0;
volatile uint32_t t2_dbg_wait_max_line = 0;

static uint32_t Prof_GetCycle(void)
{
    return DWT->CYCCNT;
}

static uint32_t Prof_CycleToUs(uint32_t cycle_delta)
{
    uint32_t clk_per_us = SystemCoreClock / 1000000U;
    if (clk_per_us == 0U)
    {
        return 0U;
    }
    return cycle_delta / clk_per_us;
}

static const char *Prof_Task2StateName(uint32_t state)
{
    switch (state)
    {
        case T2_STATE_BOOT: return "BOOT";
        case T2_STATE_INIT: return "INIT";
        case T2_STATE_MODE_SELECT: return "MODE_SELECT";
        case T2_STATE_LOOP_BEGIN: return "LOOP_BEGIN";
        case T2_STATE_AUTONOM_BRANCH: return "AUTONOM";
        case T2_STATE_AUTO_CROSS: return "AUTO_CROSS";
        case T2_STATE_AUTO_JOY_FAST: return "AUTO_JOY_FAST";
        case T2_STATE_AUTO_JOY_SLOW: return "AUTO_JOY_SLOW";
        case T2_STATE_MANUAL_BRANCH: return "MANUAL";
        case T2_STATE_MANUAL_SQUARE: return "MANUAL_SQUARE";
        case T2_STATE_MANUAL_TRIANGLE: return "MANUAL_TRIANGLE";
        case T2_STATE_MANUAL_TRIANGLE_FOREST: return "TRI_FOREST";
        case T2_STATE_MANUAL_TRIANGLE_MAGAZINE: return "TRI_MAGAZINE";
        case T2_STATE_MANUAL_TRIANGLE_GRIPPER: return "TRI_GRIPPER";
        case T2_STATE_MANUAL_TRIANGLE_FLOOR: return "TRI_FLOOR";
        case T2_STATE_MANUAL_CROSS: return "MANUAL_CROSS";
        case T2_STATE_MANUAL_LIFTER_INPUT: return "MANUAL_LIFTER";
        case T2_STATE_MANUAL_BASE_CURRENT: return "MANUAL_BASE";
        case T2_STATE_MISC_SHARE: return "MISC_SHARE";
        case T2_STATE_LOOP_END: return "LOOP_END";
        default: return "UNKNOWN";
    }
}

static const char *Prof_Task2WaitName(uint32_t kind)
{
    switch (kind)
    {
        case T2_WAIT_NONE: return "NONE";
        case T2_WAIT_PRESS: return "PRESS";
        case T2_WAIT_RELEASE: return "RELEASE";
        case T2_WAIT_ANY_PRESS: return "ANY_PRESS";
        case T2_WAIT_DELAY: return "DELAY";
        default: return "UNKNOWN";
    }
}

void Prof_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    t2_dbg_state_tick = HAL_GetTick();
}

void Prof_Reset(void)
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)PROF_COUNT; i++)
    {
        g_prof[i].hit_count = 0U;
        g_prof[i].over_count = 0U;
        g_prof[i].reenter_count = 0U;
        g_prof[i].last_us = 0U;
        g_prof[i].max_us = 0U;
        g_prof[i].last_ms = 0U;
        g_prof[i].max_ms = 0U;
        g_prof[i].active = 0U;
        g_prof[i].enter_ms = 0U;
        g_prof[i].enter_cycle = 0U;
        g_prof[i].active_ms = 0U;
    }

    prof_top_max_id = 0U;
    prof_top_max_ms = 0U;
    prof_top_active_id = 0U;
    prof_top_active_ms = 0U;

    t2_dbg_loop_active = 0U;
    t2_dbg_loop_enter_ms = 0U;
    t2_dbg_loop_last_ms = 0U;
    t2_dbg_loop_max_ms = 0U;
    t2_dbg_loop_over_count = 0U;
    t2_dbg_loop_line = 0U;
    t2_dbg_loop_max_state = 0U;
    t2_dbg_loop_max_state_line = 0U;
    t2_dbg_loop_max_wait_kind = 0U;
    t2_dbg_loop_max_wait_line = 0U;
    t2_dbg_loop_active_ms = 0U;
    t2_dbg_wait_kind = T2_WAIT_NONE;
    t2_dbg_wait_line = 0U;
    t2_dbg_wait_tick = 0U;
    t2_dbg_wait_active_ms = 0U;
    t2_dbg_wait_max_ms = 0U;
    t2_dbg_wait_max_kind = 0U;
    t2_dbg_wait_max_line = 0U;
}

void Prof_Begin(ProfId_t id)
{
    if ((uint32_t)id >= (uint32_t)PROF_COUNT)
    {
        return;
    }

    ProfRec_t *p = &g_prof[id];

    if (p->active != 0U)
    {
        p->reenter_count++;
    }

    p->active = 1U;
    p->active_ms = 0U;
    p->enter_ms = HAL_GetTick();
    p->enter_cycle = Prof_GetCycle();
}

void Prof_End(ProfId_t id)
{
    uint32_t now_ms;
    uint32_t now_cycle;
    uint32_t delta_ms;
    uint32_t delta_cycle;
    uint32_t delta_us;

    if ((uint32_t)id >= (uint32_t)PROF_COUNT)
    {
        return;
    }

    ProfRec_t *p = &g_prof[id];

    if (p->active == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    now_cycle = Prof_GetCycle();

    delta_ms = now_ms - p->enter_ms;
    delta_cycle = now_cycle - p->enter_cycle;
    delta_us = Prof_CycleToUs(delta_cycle);

    p->last_ms = delta_ms;
    p->last_us = delta_us;
    p->hit_count++;

    if (delta_ms > p->max_ms)
    {
        p->max_ms = delta_ms;
    }

    if (delta_us > p->max_us)
    {
        p->max_us = delta_us;
    }

    if (delta_ms >= p->warn_ms)
    {
        p->over_count++;
    }

    p->active = 0U;
    p->active_ms = 0U;
}

void Prof_Task2SetState(uint32_t state, uint32_t line)
{
    t2_dbg_state = state;
    t2_dbg_state_line = line;
    t2_dbg_state_tick = HAL_GetTick();
}

void Prof_Task2LoopBegin(uint32_t line)
{
    t2_dbg_loop_active = 1U;
    t2_dbg_loop_line = line;
    t2_dbg_loop_enter_ms = HAL_GetTick();
    t2_dbg_loop_active_ms = 0U;
    Prof_Task2SetState(T2_STATE_LOOP_BEGIN, line);
}

void Prof_Task2LoopEnd(uint32_t line)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - t2_dbg_loop_enter_ms;

    t2_dbg_loop_last_ms = dt;

    if (dt > t2_dbg_loop_max_ms)
    {
        t2_dbg_loop_max_ms = dt;
        t2_dbg_loop_max_state = t2_dbg_state;
        t2_dbg_loop_max_state_line = t2_dbg_state_line;
        t2_dbg_loop_max_wait_kind = t2_dbg_wait_kind;
        t2_dbg_loop_max_wait_line = t2_dbg_wait_line;
    }

    if (dt >= 30U)
    {
        t2_dbg_loop_over_count++;
    }

    t2_dbg_loop_active = 0U;
    t2_dbg_loop_active_ms = 0U;
    Prof_Task2SetState(T2_STATE_LOOP_END, line);
}

void Prof_Task2WaitBegin(uint32_t wait_kind, uint32_t line)
{
    t2_dbg_wait_kind = wait_kind;
    t2_dbg_wait_line = line;
    t2_dbg_wait_tick = HAL_GetTick();
    t2_dbg_wait_active_ms = 0U;
}

void Prof_Task2WaitEnd(uint32_t line)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - t2_dbg_wait_tick;

    if (dt > t2_dbg_wait_max_ms)
    {
        t2_dbg_wait_max_ms = dt;
        t2_dbg_wait_max_kind = t2_dbg_wait_kind;
        t2_dbg_wait_max_line = t2_dbg_wait_line;
    }

    (void)line;
    t2_dbg_wait_kind = T2_WAIT_NONE;
    t2_dbg_wait_line = 0U;
    t2_dbg_wait_tick = now;
    t2_dbg_wait_active_ms = 0U;
}

static void Prof_UpdateTop(void)
{
    uint32_t i;
    uint32_t now_ms = HAL_GetTick();
    uint32_t top_max_id = 0U;
    uint32_t top_max_ms = 0U;
    uint32_t top_active_id = 0U;
    uint32_t top_active_ms = 0U;

    for (i = 0U; i < (uint32_t)PROF_COUNT; i++)
    {
        if (g_prof[i].max_ms > top_max_ms)
        {
            top_max_ms = g_prof[i].max_ms;
            top_max_id = i;
        }

        if (g_prof[i].active != 0U)
        {
            g_prof[i].active_ms = now_ms - g_prof[i].enter_ms;

            if (g_prof[i].active_ms > top_active_ms)
            {
                top_active_ms = g_prof[i].active_ms;
                top_active_id = i;
            }
        }
        else
        {
            g_prof[i].active_ms = 0U;
        }
    }

    prof_top_max_id = top_max_id;
    prof_top_max_ms = top_max_ms;
    prof_top_active_id = top_active_id;
    prof_top_active_ms = top_active_ms;

    if (t2_dbg_loop_active != 0U)
    {
        t2_dbg_loop_active_ms = now_ms - t2_dbg_loop_enter_ms;
    }
    else
    {
        t2_dbg_loop_active_ms = 0U;
    }

    if (t2_dbg_wait_kind != T2_WAIT_NONE)
    {
        t2_dbg_wait_active_ms = now_ms - t2_dbg_wait_tick;
    }
    else
    {
        t2_dbg_wait_active_ms = 0U;
    }
}

void Prof_Task2PrintDebug(void)
{
    uint32_t now = HAL_GetTick();

    printf("TASK2_DBG : loop_act=%lu last=%lu max=%lu over=%lu loop_line=%lu\n",
           (unsigned long)t2_dbg_loop_active_ms,
           (unsigned long)t2_dbg_loop_last_ms,
           (unsigned long)t2_dbg_loop_max_ms,
           (unsigned long)t2_dbg_loop_over_count,
           (unsigned long)t2_dbg_loop_line);

    printf("TASK2_STATE: state=%lu/%s age=%lu ms line=%lu max_state=%lu/%s max_line=%lu\n",
           (unsigned long)t2_dbg_state,
           Prof_Task2StateName(t2_dbg_state),
           (unsigned long)(now - t2_dbg_state_tick),
           (unsigned long)t2_dbg_state_line,
           (unsigned long)t2_dbg_loop_max_state,
           Prof_Task2StateName(t2_dbg_loop_max_state),
           (unsigned long)t2_dbg_loop_max_state_line);

    printf("TASK2_WAIT : active=%lu kind=%lu/%s age=%lu ms line=%lu max=%lu kind=%lu/%s line=%lu\n",
           (unsigned long)(t2_dbg_wait_kind != T2_WAIT_NONE),
           (unsigned long)t2_dbg_wait_kind,
           Prof_Task2WaitName(t2_dbg_wait_kind),
           (unsigned long)t2_dbg_wait_active_ms,
           (unsigned long)t2_dbg_wait_line,
           (unsigned long)t2_dbg_wait_max_ms,
           (unsigned long)t2_dbg_wait_max_kind,
           Prof_Task2WaitName(t2_dbg_wait_max_kind),
           (unsigned long)t2_dbg_wait_max_line);

    printf("TASK2_MAXW : wait_at_loop_max=%lu/%s wait_line=%lu\n",
           (unsigned long)t2_dbg_loop_max_wait_kind,
           Prof_Task2WaitName(t2_dbg_loop_max_wait_kind),
           (unsigned long)t2_dbg_loop_max_wait_line);
}

void Prof_PrintReport(void)
{
    uint32_t i;

    Prof_UpdateTop();
    prof_print_count++;

    printf("\n------ PROFILER ------\n");
    printf("CFG       : R1_PROFILER_ENABLE=%d print=%lu\n",
           R1_PROFILER_ENABLE,
           (unsigned long)prof_print_count);

    printf("TOP MAX   : [%lu] %s max=%lu ms\n",
           (unsigned long)prof_top_max_id,
           g_prof[prof_top_max_id].name,
           (unsigned long)prof_top_max_ms);

    printf("TOP ACTIVE: [%lu] %s active=%lu ms\n",
           (unsigned long)prof_top_active_id,
           g_prof[prof_top_active_id].name,
           (unsigned long)prof_top_active_ms);

    Prof_Task2PrintDebug();

    printf("ID Name               LastMS  MaxMS  LastUS   MaxUS    Hit  Over  ActMS Re\n");

    for (i = 0U; i < (uint32_t)PROF_COUNT; i++)
    {
        if ((g_prof[i].hit_count > 0U) ||
            (g_prof[i].over_count > 0U) ||
            (g_prof[i].active_ms >= g_prof[i].warn_ms) ||
            (g_prof[i].reenter_count > 0U))
        {
            printf("%2lu %-18s %6lu %6lu %7lu %7lu %6lu %5lu %6lu %2lu\n",
                   (unsigned long)i,
                   g_prof[i].name,
                   (unsigned long)g_prof[i].last_ms,
                   (unsigned long)g_prof[i].max_ms,
                   (unsigned long)g_prof[i].last_us,
                   (unsigned long)g_prof[i].max_us,
                   (unsigned long)g_prof[i].hit_count,
                   (unsigned long)g_prof[i].over_count,
                   (unsigned long)g_prof[i].active_ms,
                   (unsigned long)g_prof[i].reenter_count);
        }
    }
}

#endif /* R1_PROFILER_ENABLE */
