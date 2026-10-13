/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "Joystick.h"
#include "Profiler.h"
#include "R1ActuatorControl.h"
#include "R1MeihuaManual.h"
#include "R1MotionControl.h"
#include "Swerve.h"
#include "USART.h"
#include "dyn.h"
#include "joystickRizal.h"
#include "ARGB.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*
 * Blocking sequence button helper macro.
 *
 * Macro ini dipakai di SequenceTask yang memang berjalan secara blocking.
 * BUTTON_EXPR boleh berupa macro tombol seperti PS_Square, BTN_SILANG, dll.
 *
 * SEQ_WAIT_CLICK:
 *   Menunggu tombol dilepas -> ditekan -> dilepas lagi.
 *   Tujuannya agar satu tekanan tombol hanya melanjutkan satu step sequence
 *   dan tidak tembus ke step berikutnya saat tombol masih ditahan.
 *
 * SEQ_WAIT_PRESS_WITH_BODY:
 *   Menunggu tombol ditekan lalu dilepas, sambil menjalankan BODY_CODE
 *   di dalam loop tunggu. Cocok untuk step yang tetap perlu menjalankan
 *   kontrol manual kecil, misalnya jog pneumatic, selama menunggu tombol next.
 *
 * Catatan:
 *   Ini bukan function, tetapi macro preprocessor.
 *   BODY_CODE akan disalin langsung ke dalam loop oleh preprocessor.
 *   osDelay(1) wajib ada agar task lain tetap mendapat waktu jalan.
 */
uint8_t ModeSpeedRizal = 0;
#define SEQ_BASE_CURRENT()                     \
  do {                                         \
    PROF_BEGIN(PROF_SEQ_BASE_CURRENT);         \
    JoystickHeadingLock_RIZAL(ModeSpeedRizal); \
    PROF_END(PROF_SEQ_BASE_CURRENT);           \
  } while (0)
#define SEQ_BASE_SLOW()                  \
  do {                                   \
    PROF_BEGIN(PROF_SEQ_BASE_SLOW);      \
    JoystickHeadingLock_RIZAL(SlowMode); \
    PROF_END(PROF_SEQ_BASE_SLOW);        \
  } while (0)

/* ================================================================ */
/*  POSISI LIFTER UTAMA — LifterUtama(float heading)                */
/* ================================================================ */
#define LIFTER_BOX_ATAS 4242
#define LIFTER_BOX_MENENGAH 2816
#define LIFTER_BOX_BAWAH 1280
#define LIFTER_BOX_PALING_BAWAH 0
#define LIFTER_TIKTAKTU 3200
#define LIFTER_BOX_LANTAI 600
#define LIFTER_TONGKAT 1880
#define LIFTER_TONGKAT_ASEMBLE 480

/* ================================================================ */
/*  POSISI SLIDER — LifterKFs(float heading)                        */
/* ================================================================ */
// #define SLIDER_MAKSIMAL            4300
// #define SLIDER_MAJU                4200
// #define SLIDER_LANTAI              3100
// #define SLIDER_MUNDUR              0
// #define SLIDER_MAG					600
// #define SLIDER_TIKTAKTU            2000 // Motor Lama
#define SLIDER_MAKSIMAL 1616
#define SLIDER_MAJU 1578
#define SLIDER_LANTAI 1165
#define SLIDER_MUNDUR 0
#define SLIDER_MAG 225
#define SLIDER_TIKTAKTU 752

#define SEQ_WAIT_PRESS_WITH_BODY(BUTTON_EXPR, BODY_CODE) \
  do {                                                   \
    T2_DBG_WAIT_BEGIN(T2_WAIT_PRESS);                    \
    PROF_BEGIN(PROF_SEQ_WAIT_PRESS);                     \
    do {                                                 \
      BODY_CODE                                          \
      osDelay(1);                                        \
    } while ((BUTTON_EXPR) == 0);                        \
                                                         \
    do {                                                 \
      BODY_CODE                                          \
      osDelay(1);                                        \
    } while ((BUTTON_EXPR) != 0);                        \
    PROF_END(PROF_SEQ_WAIT_PRESS);                       \
    T2_DBG_WAIT_END();                                   \
  } while (0)

#define SEQ_WAIT_RELEASE_WITH_BODY(BUTTON_EXPR, BODY_CODE) \
  do {                                                     \
    T2_DBG_WAIT_BEGIN(T2_WAIT_RELEASE);                    \
    PROF_BEGIN(PROF_SEQ_WAIT_RELEASE);                     \
    do {                                                   \
      BODY_CODE                                            \
      osDelay(1);                                          \
    } while ((BUTTON_EXPR) != 0);                          \
    PROF_END(PROF_SEQ_WAIT_RELEASE);                       \
    T2_DBG_WAIT_END();                                     \
  } while (0)

#define SEQ_WAIT_ANY_PRESS_WITH_BODY(BTN_A, BTN_B, BODY_CODE) \
  do {                                                        \
    T2_DBG_WAIT_BEGIN(T2_WAIT_ANY_PRESS);                     \
    PROF_BEGIN(PROF_SEQ_WAIT_ANY_PRESS);                      \
    do {                                                      \
      BODY_CODE                                               \
      osDelay(1);                                             \
    } while (((BTN_A) == 0) && ((BTN_B) == 0));               \
    PROF_END(PROF_SEQ_WAIT_ANY_PRESS);                        \
    T2_DBG_WAIT_END();                                        \
  } while (0)

#define SEQ_DELAY_WITH_BODY(DELAY_MS, BODY_CODE)                       \
  do {                                                                 \
    T2_DBG_WAIT_BEGIN(T2_WAIT_DELAY);                                  \
    PROF_BEGIN(PROF_SEQ_DELAY);                                        \
    uint32_t _seq_delay_start = osKernelGetTickCount();                \
    while ((osKernelGetTickCount() - _seq_delay_start) < (DELAY_MS)) { \
      BODY_CODE                                                        \
      osDelay(1);                                                      \
    }                                                                  \
    PROF_END(PROF_SEQ_DELAY);                                          \
    T2_DBG_WAIT_END();                                                 \
  } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;
DMA_HandleTypeDef hdma_tim8_ch3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for uartSenderTask */
osThreadId_t uartSenderTaskHandle;
const osThreadAttr_t uartSenderTask_attributes = {
  .name = "uartSenderTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OdomTask */
osThreadId_t OdomTaskHandle;
const osThreadAttr_t OdomTask_attributes = {
  .name = "OdomTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ARGBtask */
osThreadId_t ARGBtaskHandle;
const osThreadAttr_t ARGBtask_attributes = {
  .name = "ARGBtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for uartTxQueue */
osMessageQueueId_t uartTxQueueHandle;
const osMessageQueueAttr_t uartTxQueue_attributes = {
  .name = "uartTxQueue"
};
/* Definitions for ARGB_queue */
osMessageQueueId_t ARGB_queueHandle;
const osMessageQueueAttr_t ARGB_queue_attributes = {
  .name = "ARGB_queue"
};
/* Definitions for uartTxDoneSem */
osSemaphoreId_t uartTxDoneSemHandle;
const osSemaphoreAttr_t uartTxDoneSem_attributes = {
  .name = "uartTxDoneSem"
};
/* USER CODE BEGIN PV */

volatile unsigned long ulHighFrequencyTimerTicks;

// DMA Krirm TX USART3
/* ================= UART DMA SENDER CONFIG ================= */
#define UART_SENDER_HUART (&huart3)  // Ganti sesuai UART kamu
#define UART_TX_QUEUE_DEPTH 64U
#define UART_TX_DMA_TIMEOUT_MS 20U
#define UART_TX_LEVEL_70_PERCENT 70U
#define UART_TX_LEVEL_90_PERCENT 90U
/* ================= DMA BUFFER ================= */
static uint8_t uartTxDmaBuffer[UART_TX_FRAME_LEN];
/* ================= DEBUG COUNTERS ================= */
volatile uint32_t g_uart_tx_post_ok_count = 0;
volatile uint32_t g_uart_tx_post_fail_count = 0;
volatile uint32_t g_uart_tx_queue_high70_count = 0;
volatile uint32_t g_uart_tx_queue_high90_count = 0;
volatile uint32_t g_uart_tx_queue_full_count = 0;
volatile uint32_t g_uart_tx_queue_max_used = 0;
volatile uint32_t g_uart_tx_dma_start_count = 0;
volatile uint32_t g_uart_tx_dma_done_count = 0;
volatile uint32_t g_uart_tx_dma_error_count = 0;
volatile uint32_t g_uart_tx_dma_timeout_count = 0;
volatile uint8_t g_uart_tx_dma_busy = 0;
/* Edge flag supaya counter 70/90/full tidak naik terus-menerus */
static uint8_t uart_tx_was_above_70 = 0U;
static uint8_t uart_tx_was_above_90 = 0U;
static uint8_t uart_tx_was_full = 0U;

// Solusi IMU HAL_UART_ERROR_ORE
#define IMU_UART_HANDLE (&huart6)
#define IMU_DMA_RX_BUFFER_SIZE 64U
#define IMU_RING_BUFFER_SIZE 256U
static uint8_t imu_dma_rx_buffer[IMU_DMA_RX_BUFFER_SIZE];
static uint8_t imu_ring_buffer[IMU_RING_BUFFER_SIZE];
static volatile uint16_t imu_ring_write = 0U;
static volatile uint16_t imu_ring_read = 0U;
static volatile uint16_t imu_dma_old_pos = 0U;
volatile float DataYawIMU = 0.0f;
volatile uint8_t g_imu_valid = 0U;
volatile uint32_t g_imu_last_rx_tick = 0U;
volatile uint32_t g_imu_dma_event_count = 0U;
volatile uint32_t g_imu_ring_overflow_count = 0U;
volatile uint32_t g_imu_packet_ok_count = 0U;
volatile uint32_t g_imu_checksum_error_count = 0U;
volatile uint32_t g_imu_uart_error_count = 0U;
volatile uint32_t g_imu_uart_last_error = 0U;

// Swerve
volatile uint32_t CounterDelay = 0;
uint8_t FlagIMU = 0;

float last_sudut_M1 = 0, last_sudut_M2 = 0, last_sudut_M3 = 0,
      last_sudut_M4 = 0;
float LastSpeed1 = 0, LastSpeed2 = 0, LastSpeed3 = 0;
float MaxSpeed = 5000, LimitErrorJ = 250, LimitErrorH = 5, LimitErrorTrace = 10;
float KPj = 10, KDj = 0, KPh = 0, KDh = 0;  //, KIh=0;
float KPTrace = 0, KDTrace = 0;
int cek_nilai = 0;

 float PosX_World = 0, PosY_World = 0, RobotYaw = 0,
      YawOffset = 0;  // data posisi di lapangan
 float PosX_robot = 0, PosY_robot = 0;

 short SpeedEncA = 0, SpeedEncB = 0;
volatile unsigned char togglePosisi;

short IncSudut = 0;
extern volatile uint8_t Target_Keranjang[5];

// Spiline
int titikX[10], titikY[10];
double inc = 0.015, t = 0, titikXS[100], titikYS[100], w[10], headingR[100];

uint16_t indekTitik = 0;

double numX = 0, numY = 0, denum = 0;

double RefX1[100], RefX2[100], RefX3[100], RefX4[100];
double RefY1[100], RefY2[100], RefY3[100], RefY4[100];

uint16_t Index1 = 0, Index2 = 0, Index3 = 0, Index4 = 0;

extern unsigned char ReceivedFromPC[11];
extern uint8_t ReceivingFromPC;
extern int flagputer_roda_slave;

extern unsigned char speed1, speed2, speed3;
extern float sudut1, sudut2, sudut3;

int cek[5];

uint8_t tesTanpaRemote = 0;

#define TONGKAT_ID 1
#define TONGKAT_HOME 460
#define TONGKAT_AMBIL 1789
#define TONGKAT_NAIK 2033
#define TONGKAT_ASEMBEL 945

#define TONGKAT_SERVO_TTT_STEP 30
#define TONGKAT_SERVO_TTT_MIN TONGKAT_ASEMBEL
#define TONGKAT_SERVO_TTT_MAX TONGKAT_NAIK

static int Tongkat_pitch = TONGKAT_AMBIL;

uint8_t sudahAmbilTongkat = 0;

#define LIFTER_TTT_STEP 25  // +25
#define LIFTER_TTT_MIN LIFTER_BOX_PALING_BAWAH
#define LIFTER_TTT_MAX LIFTER_BOX_ATAS

static uint8_t kfsInGripper = 0U;
static uint8_t kfsMagazineFilled = 0U;
static int posisiLifterSaatIni =
    LIFTER_TONGKAT_ASEMBLE;  // LIFTER_BOX_TICTACTOE

#define KFS_ID 2
#define KFS_SPEED_SLOW 267
#define KFS_SPEED_FAST 500
#define KFS_MAG 830
#define KFS_FORREST 1930
#define KFS_LANTAI 3180

#define KFS_SERVO_TTT_STEP 30
#define KFS_SERVO_TTT_MIN KFS_MAG
#define KFS_SERVO_TTT_MAX KFS_LANTAI

static int KFS_pitch = KFS_FORREST;

float R1_DBG_ErrorX;
float R1_DBG_ErrorY;
float R1_DBG_ErrorJ;
float R1_DBG_ErrorH;
float R1_DBG_Sudut;
float R1_DBG_Vx;
float R1_DBG_Vy;
float R1_DBG_Omega;

float R1_DBG_PosX_Robot;
float R1_DBG_PosY_Robot;
float R1_DBG_PosX_World;
float R1_DBG_PosY_World;
float R1_DBG_RobotYaw;
int16_t R1_DBG_SpeedEncA;
int16_t R1_DBG_SpeedEncB;

volatile uint8_t PS_INT = 0;
volatile uint8_t ModeAuto = 0;

uint32_t PB1 = 0;
uint32_t PB2 = 0;
uint32_t PB3 = 0;
uint32_t PB4 = 0;

typedef struct{
	uint32_t mode;
	uint32_t sequens_num;
}argbMode_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM8_Init(void);
void StartTask1(void *argument);
void StartTask02(void *argument);
void Start_uartSenderTask(void *argument);
void Start_ImuTask(void *argument);
void StartT_MonitorTask(void *argument);
void StartOdomTask_RIZAL(void *argument);
void Start_ARGBtask(void *argument);

/* USER CODE BEGIN PFP */
static void UartSender_UpdateQueueDebug(void);
static void ImuRing_PushByte(uint8_t data);
static uint8_t ImuRing_ReadByte(uint8_t* data);
static void ImuDma_CheckNewData(uint16_t dma_pos);
static void ParsingDataIMU(uint8_t c);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    usart_rx_check(); /* Check for data to process */
  }

  if (huart->Instance == USART6) {
    ImuDma_CheckNewData(Size);
  }
}
/*
 * Jika callback ini sudah ada di main.c,
 * jangan buat dua kali. Gabungkan isi if ini ke callback yang lama.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == UART_SENDER_HUART->Instance) {
    g_uart_tx_dma_busy = 0U;
    g_uart_tx_dma_done_count++;

    osSemaphoreRelease(uartTxDoneSemHandle);
  }
}

/*
 * Jika callback ini sudah ada di main.c,
 * jangan buat dua kali. Gabungkan isi if ini ke callback yang lama.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == UART_SENDER_HUART->Instance) {
    g_uart_tx_dma_busy = 0U;
    g_uart_tx_dma_error_count++;

    HAL_UART_AbortTransmit(huart);

    /*
     * Supaya task tidak stuck menunggu semaphore.
     */
    osSemaphoreRelease(uartTxDoneSemHandle);
  }

  if (huart->Instance == USART6) {
    g_imu_uart_error_count++;
    g_imu_uart_last_error = huart->ErrorCode;

    HAL_UART_AbortReceive(huart);

    imu_dma_old_pos = 0U;

    HAL_UARTEx_ReceiveToIdle_DMA(IMU_UART_HANDLE, imu_dma_rx_buffer,
                                 IMU_DMA_RX_BUFFER_SIZE);

    if (IMU_UART_HANDLE->hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(IMU_UART_HANDLE->hdmarx, DMA_IT_HT);
    }
  }
}

void configureTimerForRunTimeStats(void) {
  ulHighFrequencyTimerTicks = 0;
  HAL_TIM_Base_Start_IT(&htim13);
}

unsigned long getRunTimeCounterValue(void) {
  return ulHighFrequencyTimerTicks;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM13_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // kiri
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // belakang

  HAL_TIM_Base_Start_IT(&htim7);  // Timer Sampling Encoder

  // USART
  //	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);//USART RS485
  //	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);//USART Joystick
  //	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);//USART IMU
  //	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);//USART IMU
  //	  HAL_UART_Receive_IT(&huart6,(uint8_t*)Rx_IMU,1);

  HAL_UARTEx_ReceiveToIdle_DMA(IMU_UART_HANDLE, imu_dma_rx_buffer,
                               IMU_DMA_RX_BUFFER_SIZE);
  /*
   * Opsional:
   * Matikan half-transfer interrupt supaya callback tidak terlalu sering.
   * Kalau hdmarx valid.
   */
  //	if (IMU_UART_HANDLE->hdmarx != NULL)
  //	{
  //	    __HAL_DMA_DISABLE_IT(IMU_UART_HANDLE->hdmarx, DMA_IT_HT);
  //	}
  //	SetWorldPosition(0,0,0);
  //	Kinematik(2,0,0);
  //	osDelay(200);
  //	RodaStop();
  //	osDelay(500);
  //	startDMAusart();
  //	MoveServo(TONGKAT_ID, TONGKAT_HOME);
  //	MoveServo(2, KFS_MAG);
  //	LifterUtama(0,0);
  //	LifterKFs(0);
  //	DegKFS(0);
  //	Pneumatic(0,0,0);
  //	PneumaticMotor(0,0);
  //	Pneumatic(1, 0, 0);
  //	HAL_Delay(600);
  //	Pneumatic(0, 1, 0);
  //	HAL_Delay(600);
  //	Pneumatic(0, 0, 1);
  //	HAL_Delay(600);
  //	Pneumatic(0, 0, 0);
  //	HAL_Delay(600);
  //	Pneumatic(1, 1, 1);
  //	HAL_Delay(600);
  //	Pneumatic(0, 0, 0);
  //	HAL_Delay(600);
  PROF_INIT();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uartTxDoneSem */
  uartTxDoneSemHandle = osSemaphoreNew(1, 1, &uartTxDoneSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartTxQueue */
  uartTxQueueHandle = osMessageQueueNew (64, sizeof(UartTxFrame_t), &uartTxQueue_attributes);

  /* creation of ARGB_queue */
  ARGB_queueHandle = osMessageQueueNew (4, sizeof(argbMode_t), &ARGB_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of uartSenderTask */
  uartSenderTaskHandle = osThreadNew(Start_uartSenderTask, NULL, &uartSenderTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(Start_ImuTask, NULL, &ImuTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartT_MonitorTask, NULL, &MonitorTask_attributes);

  /* creation of OdomTask */
  OdomTaskHandle = osThreadNew(StartOdomTask_RIZAL, NULL, &OdomTask_attributes);

  /* creation of ARGBtask */
  ARGBtaskHandle = osThreadNew(Start_ARGBtask, NULL, &ARGBtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 840-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB_1_Pin PB_2_Pin PB_3_Pin PB_4_Pin */
  GPIO_InitStruct.Pin = PB_1_Pin|PB_2_Pin|PB_3_Pin|PB_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void UartSender_UpdateQueueDebug(void) {
  if (uartTxQueueHandle == NULL) {
    return;
  }

  uint32_t used = osMessageQueueGetCount(uartTxQueueHandle);
  uint32_t cap = osMessageQueueGetCapacity(uartTxQueueHandle);

  if (cap == 0U) {
    return;
  }

  uint32_t th70 = (cap * UART_TX_LEVEL_70_PERCENT) / 100U;
  uint32_t th90 = (cap * UART_TX_LEVEL_90_PERCENT) / 100U;

  if (used > g_uart_tx_queue_max_used) {
    g_uart_tx_queue_max_used = used;
  }

  if (used >= th70) {
    if (uart_tx_was_above_70 == 0U) {
      g_uart_tx_queue_high70_count++;
      uart_tx_was_above_70 = 1U;
    }
  } else {
    uart_tx_was_above_70 = 0U;
  }

  if (used >= th90) {
    if (uart_tx_was_above_90 == 0U) {
      g_uart_tx_queue_high90_count++;
      uart_tx_was_above_90 = 1U;
    }
  } else {
    uart_tx_was_above_90 = 0U;
  }

  if (used >= cap) {
    if (uart_tx_was_full == 0U) {
      g_uart_tx_queue_full_count++;
      uart_tx_was_full = 1U;
    }
  } else {
    uart_tx_was_full = 0U;
  }
}

static void ImuRing_PushByte(uint8_t data) {
  uint16_t next = (uint16_t)((imu_ring_write + 1U) % IMU_RING_BUFFER_SIZE);

  if (next == imu_ring_read) {
    /*
     * Ring buffer penuh.
     * Data baru tidak dimasukkan.
     */
    g_imu_ring_overflow_count++;
    return;
  }

  imu_ring_buffer[imu_ring_write] = data;
  imu_ring_write = next;
}

static uint8_t ImuRing_ReadByte(uint8_t* data) {
  if (imu_ring_read == imu_ring_write) {
    return 0U;
  }

  *data = imu_ring_buffer[imu_ring_read];
  imu_ring_read = (uint16_t)((imu_ring_read + 1U) % IMU_RING_BUFFER_SIZE);

  return 1U;
}

/*
 * Dipanggil dari DMA callback.
 * Tugasnya hanya memindahkan byte baru dari DMA buffer ke ring buffer.
 */
static void ImuDma_CheckNewData(uint16_t dma_pos) {
  if (dma_pos == imu_dma_old_pos) {
    return;
  }

  if (dma_pos > IMU_DMA_RX_BUFFER_SIZE) {
    return;
  }

  if (dma_pos > imu_dma_old_pos) {
    for (uint16_t i = imu_dma_old_pos; i < dma_pos; i++) {
      ImuRing_PushByte(imu_dma_rx_buffer[i]);
    }
  } else {
    for (uint16_t i = imu_dma_old_pos; i < IMU_DMA_RX_BUFFER_SIZE; i++) {
      ImuRing_PushByte(imu_dma_rx_buffer[i]);
    }

    for (uint16_t i = 0U; i < dma_pos; i++) {
      ImuRing_PushByte(imu_dma_rx_buffer[i]);
    }
  }

  imu_dma_old_pos = dma_pos;
  g_imu_dma_event_count++;
}

/*
 * Parser tetap byte-by-byte, tapi sekarang jalan di task,
 * bukan di interrupt.
 */
static void ParsingDataIMU(uint8_t c) {
  static uint8_t sBuffer[3];
  static uint8_t sCounter = 0U;
  static uint8_t sflag = 0U;

  if (sflag == 0U) {
    if (c == 0xFFU) {
      sflag = 1U;
    }
  } else if (sflag == 1U) {
    if (c == 0xFEU) {
      sflag = 2U;
      sCounter = 0U;
    } else if (c == 0xFFU) {
      /*
       * Tetap di state header pertama.
       */
      sflag = 1U;
    } else {
      sflag = 0U;
    }
  } else if (sflag == 2U) {
    sBuffer[sCounter] = c;
    sCounter++;

    if (sCounter >= 3U) {
      uint8_t calculatedChecksum = (uint8_t)(sBuffer[0] + sBuffer[1]);

      if (sBuffer[2] == calculatedChecksum) {
        uint16_t azimuth = ((uint16_t)sBuffer[0] << 8) | sBuffer[1];

        /*
         * Float sekarang aman karena jalan di task, bukan interrupt.
         */
        DataYawIMU = 360.0f - ((float)azimuth / 10.0f);

        if (DataYawIMU >= 360.0f) {
          DataYawIMU -= 360.0f;
        }

        if (DataYawIMU < 0.0f) {
          DataYawIMU += 360.0f;
        }

        g_imu_valid = 1U;
        g_imu_last_rx_tick = osKernelGetTickCount();
        g_imu_packet_ok_count++;
      } else {
        g_imu_checksum_error_count++;
      }

      sflag = 0U;
      sCounter = 0U;
    }
  } else {
    sflag = 0U;
    sCounter = 0U;
  }
}

void HudaSeq() {
  if (PS_Cross != 0) {
    //				Kinematik(2,2,0);
    //				MoveServo(TONGKAT_ID,1800);
    //				SetWorldPosition(0,0,0);
    // HAL_Delay(300);
    //
    //				MaxSpeed=2000; LimitErrorJ=700;
    // LimitErrorH=5; 	KPj=40;	KDj=13; 	KPh=1;
    //				JalanKeTitik(-300,500,0,200,0,1000);
    //
    //				MaxSpeed=3500; LimitErrorJ=700;
    // LimitErrorH=5; 	KPj=40;	KDj=13; 	KPh=1;
    //				JalanKeTitik(-1000,400,0,200,0,1000);
    //
    //				MaxSpeed=2000; LimitErrorJ=700;
    // LimitErrorH=5; 	KPj=40;	KDj=13; 	KPh=1;
    //				JalanKeTitik(-2000,400,0,200,0,1000);
    //
    //				RodaStop();
    //
    //				for(int a=0;a<=10;a++)
    // while(PS_Cross==0)
    //				{
    //					JoystickHeadingLock(SlowMode);
    //
    //					if(PS_Square != 0)
    //						{
    //						PneumaticMotor(1, 0);
    //						PneumaticMotor(1, 0);
    //						}
    //
    //				}
    //
    //				MoveServo(TONGKAT_ID,0);
    //				osDelay(500);
    //
    //				MaxSpeed=2000; LimitErrorJ=700;
    // LimitErrorH=5; 	KPj=40;	KDj=13; 	KPh=1;
    //				JalanKeTitik(-2900,1100,0,200,0,1000);
    //
    //				MoveServo(TONGKAT_ID,2650);
    //				LifterUtama(1300);
    //
    //				PneumaticMotor(1, -200);
    //				osDelay(500);
    //				PneumaticMotor(1, 0);
    //
    //				MaxSpeed=2000; LimitErrorJ=700;
    // LimitErrorH=5; 	KPj=40;	KDj=13; 	KPh=1;
    //				JalanKeTitik(-4000,1100,0,200,0,1000);
    //
    //				RodaStop();
    //
    //				for(int a=0;a<=10;a++)
    // while(PS_Cross==0)
    //				{
    //					JoystickHeadingLock(SlowMode);
    //
    //					if(PS_Square != 0)
    //						{
    //						PneumaticMotor(1,
    //-200); 						} else {
    // PneumaticMotor(1, 0);
    //						}
    //
    //					if(PS_Circle != 0)
    //						{
    //						PneumaticMotor(1, 200);
    //						} else {
    //						PneumaticMotor(1, 0);
    //						}
    //
    //				}

    SetWorldPosition(-4000, 1100, 0);
    HAL_Delay(300);
    for (int a = 0; a <= 10; a++)
      while (PS_Cross == 0) {
        if (PS_RXMin != 0) {
          MaxSpeed = 2000;
          LimitErrorJ = 700;
          LimitErrorH = 5;
          KPj = 40;
          KDj = 13;
          KPh = 1;
          JalanKeTitik(-3800, 2000, 0, 200, 0, 1000);
          RodaStop();
          DegKFS(1);
          Pneumatic(1, 0, 0);
        }

        else if (PS_RYMin != 0) {
          MaxSpeed = 2000;
          LimitErrorJ = 700;
          LimitErrorH = 5;
          KPj = 40;
          KDj = 13;
          KPh = 1;
          JalanKeTitik(-2500, 2000, 0, 200, 0, 1000);
          RodaStop();
          DegKFS(1);
          Pneumatic(1, 0, 0);
        }

        else if (PS_RXPlus != 0) {
          MaxSpeed = 2000;
          LimitErrorJ = 700;
          LimitErrorH = 5;
          KPj = 40;
          KDj = 13;
          KPh = 1;
          JalanKeTitik(-1500, 2000, 0, 200, 0, 1000);
          RodaStop();
          DegKFS(1);
          Pneumatic(1, 0, 0);
        }
        JoystickHeadingLock(SlowMode);
      }

    for (int a = 0; a <= 10; a++)
      while (PS_Cross == 0) {
        JoystickHeadingLock(SlowMode);
      }
  }

  JoystickHeadingLock(FastMode);
}

static void Seq_Ttt_AdjustHeightBody(void) {
  // untuk adjust ketinggian lifter
  static uint32_t lastAdjustTick = 0U;
  uint32_t now = osKernelGetTickCount();

  if ((now - lastAdjustTick) >= 50U) {
    if (JOYS_L_ATAS) {
      posisiLifterSaatIni += LIFTER_TTT_STEP;
      if (posisiLifterSaatIni > LIFTER_TTT_MAX)
        posisiLifterSaatIni = LIFTER_TTT_MAX;

      LifterUtama(posisiLifterSaatIni, 1);
      lastAdjustTick = now;
    } else if (JOYS_L_BAWAH) {
      posisiLifterSaatIni -= LIFTER_TTT_STEP;
      if (posisiLifterSaatIni < LIFTER_TTT_MIN)
        posisiLifterSaatIni = LIFTER_TTT_MIN;

      LifterUtama(posisiLifterSaatIni, 1);
      lastAdjustTick = now;
    }
  }
}

static void Seq_Ttt_AdjustPitchKFS(uint16_t pb1, uint16_t pb2) {
  // untuk adjust posisi rotasi gripper kfs
  static uint32_t lastAdjustTick = 0U;
  uint32_t now = osKernelGetTickCount();

  if ((now - lastAdjustTick) >= 50U) {
    if (pb1 != 0) {
      KFS_pitch += KFS_SERVO_TTT_STEP;
      if (KFS_pitch > KFS_SERVO_TTT_MAX)
        KFS_pitch = KFS_SERVO_TTT_MAX;

      MoveServo(KFS_ID, KFS_pitch);
      lastAdjustTick = now;
    } else if (pb2 != 0) {
      KFS_pitch -= KFS_SERVO_TTT_STEP;
      if (KFS_pitch < KFS_SERVO_TTT_MIN)
        KFS_pitch = KFS_SERVO_TTT_MIN;

      MoveServo(KFS_ID, KFS_pitch);
      lastAdjustTick = now;
    }
  }
}

static void Seq_Ttt_AdjustPitchTongkat(uint16_t pb1, uint16_t pb2) {
  // untuk adjust posisi rotasi gripper kfs
  static uint32_t lastAdjustTick = 0U;
  uint32_t now = osKernelGetTickCount();

  if ((now - lastAdjustTick) >= 50U) {
    if (pb1 != 0) {
      Tongkat_pitch += TONGKAT_SERVO_TTT_STEP;
      if (Tongkat_pitch > TONGKAT_SERVO_TTT_MAX)
        Tongkat_pitch = TONGKAT_SERVO_TTT_MAX;

      MoveServo(TONGKAT_ID, Tongkat_pitch);
      lastAdjustTick = now;
    } else if (pb2 != 0) {
      Tongkat_pitch -= TONGKAT_SERVO_TTT_STEP;
      if (Tongkat_pitch < TONGKAT_SERVO_TTT_MIN)
        Tongkat_pitch = TONGKAT_SERVO_TTT_MIN;

      MoveServo(TONGKAT_ID, Tongkat_pitch);
      lastAdjustTick = now;
    }
  }
}

static void Seq_PlaceOneKfs_TicTacToe(void) {
  MoveServo(2, KFS_FORREST);
  KFS_pitch = KFS_FORREST;
  LifterKFs(SLIDER_TIKTAKTU);

  posisiLifterSaatIni = LIFTER_TIKTAKTU;
  LifterUtama(posisiLifterSaatIni, 1);

  /*
   * X pertama sudah dipakai untuk masuk sequence.
   * Macro ini menunggu X dilepas, lalu X ditekan lagi untuk naruh.
   * Selama menunggu, JOYS_L_ATAS/BAWAH bisa adjust lifter utama.
   */

  SEQ_DELAY_WITH_BODY(600, { SEQ_BASE_CURRENT(); });
  SEQ_WAIT_PRESS_WITH_BODY(PS_Cross, {
    SEQ_BASE_CURRENT();
    Seq_Ttt_AdjustHeightBody();

    // atur servo kfs
    Seq_Ttt_AdjustPitchKFS(BTN_LINGKARAN, BTN_SEGITIGA);
  });

  setPn1(1);  // buka gripper / naruh KFS
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });

  KFS_pitch = KFS_FORREST;
  MoveServo(KFS_ID, KFS_pitch);
  LifterKFs(SLIDER_MUNDUR);
  kfsInGripper = 0U;
}

static void Seq_SaveKfsToMagazine(void) {
  setPn1(0);  // gripper tutup
  SEQ_DELAY_WITH_BODY(200, {
    SEQ_BASE_CURRENT();
    if (BTN_KOTAK != 0) {
      goto Adjust;
    }
  });
  MoveServo(2, KFS_MAG);
  SEQ_DELAY_WITH_BODY(800, {
    SEQ_BASE_CURRENT();
    if (BTN_KOTAK != 0) {
      goto Adjust;
    }
  });

  setPn2(0);  // magazine turun
  setPn3(0);  // magazine buka

  LifterKFs(SLIDER_MUNDUR);
  SEQ_WAIT_PRESS_WITH_BODY(PS_Triangle, {
    SEQ_BASE_CURRENT();
    if (BTN_KOTAK != 0) {
      goto Adjust;
    }
  });

  setPn3(1);  // magazine tutup
  SEQ_DELAY_WITH_BODY(400, {
    SEQ_BASE_CURRENT();
    if (BTN_KOTAK != 0) {
      goto Adjust;
    }
  });

  setPn1(1);  // gripper buka
  SEQ_DELAY_WITH_BODY(400, {
    SEQ_BASE_CURRENT();
    if (BTN_KOTAK != 0) {
      goto Adjust;
    }
  });

  setPn2(1);  // magazine naik
  MoveServo(2, KFS_FORREST);
  LifterKFs(SLIDER_MUNDUR);

  kfsMagazineFilled = 1U;
  kfsInGripper = 0U;

  goto NoError;
Adjust:
  Seq_PlaceOneKfs_TicTacToe();
NoError:
  return;
}

static void Seq_TakeKfsFromMagazine(void) {
  //	LifterKFs(SLIDER_LANTAI);
  //	SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_CURRENT(); });

  setPn1(1);  // gripper buka
              //	SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_CURRENT(); });

  MoveServo(2, KFS_MAG);
  LifterKFs(SLIDER_MUNDUR);
  SEQ_DELAY_WITH_BODY(1000, { SEQ_BASE_CURRENT(); });

  setPn2(0);  // magazine turun
  SEQ_DELAY_WITH_BODY(1500, { SEQ_BASE_CURRENT(); });

  setPn3(1);  // magazine buka
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });

  setPn1(0);  // gripper tutup
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });

  setPn3(0);  // magazine buka
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });

  setPn2(1);  // magazine naik
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });

  LifterKFs(SLIDER_TIKTAKTU);
  SEQ_DELAY_WITH_BODY(1000, { SEQ_BASE_CURRENT(); });

  MoveServo(2, KFS_FORREST);

  kfsMagazineFilled = 0U;
  kfsInGripper = 1U;
}

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == 1 || fd == 2) {
    hstatus = HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return -1;
  }
  return -1;
}

void AmbilTongkatDanAssembly(void) {
  // Posisikan untuk ambil tongkat
  MoveServo(TONGKAT_ID, TONGKAT_AMBIL);
  oldPneumatikMotor(0, 0);
  LifterUtama(LIFTER_TONGKAT, 1);

  SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_SLOW(); });
  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, { SEQ_BASE_SLOW(); });

  // grip, naikan tongkat, lalu berhenti
  oldPneumatikMotor(1, 0);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });

  LifterUtama(LIFTER_TONGKAT + 140, 1);
  SEQ_DELAY_WITH_BODY(400, { SEQ_BASE_CURRENT(); });

  //   SEQ_WAIT_PRESS_WITH_BODY(PS_Square, { SEQ_BASE_CURRENT(); });

  // posisikan tongkat horizontal untuk assembly dengan R2, mundurkan
  // tongkat, lalu berhenti
  MoveServo(TONGKAT_ID, TONGKAT_ASEMBEL);
  SEQ_DELAY_WITH_BODY(700, { SEQ_BASE_CURRENT(); });
  oldPneumatikMotor(1, 75);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });
  oldPneumatikMotor(1, 0);

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, {
    SEQ_BASE_CURRENT();
    // kontrol manual untuk menyesuaikan dengan eror lapangan
    if (PS_Circle != 0) {
      oldPneumatikMotor(1, 75);
    } else if (PS_Triangle != 0) {
      oldPneumatikMotor(1, -75);
    } else {
      oldPneumatikMotor(1, 0);
    }
    //
  });

  // turunkanlifter agar tongkt sejajar dengan R2
  LifterUtama(LIFTER_TONGKAT_ASEMBLE, 1);
  posisiLifterSaatIni = LIFTER_TONGKAT_ASEMBLE;

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, {
    SEQ_BASE_SLOW();
    // kontrol manual untuk menyesuaikan dengan eror lapangan
    if (PS_Circle != 0) {
      oldPneumatikMotor(1, 75);
    } else if (PS_Triangle != 0) {
      oldPneumatikMotor(1, -75);
    } else {
      oldPneumatikMotor(1, 0);
    }
    Seq_Ttt_AdjustHeightBody();
    if (PS_Cross != 0) {
      setLED(1);
      SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_SLOW(); });
      setLED(0);
    }
    //						if (PS_Cross != 0) {
    //							// Nyala
    // mAtikan LED, belum ada hardwarenya
    // } else {
    //
    //						}
  });

  // tarik ke atas tongkat dengan spearhead yang sudah di assembly, lalu
  // selesai
  oldPneumatikMotor(1, 0);
  Tongkat_pitch = TONGKAT_ASEMBEL;
  MoveServo(TONGKAT_ID, TONGKAT_ASEMBEL);
  LifterUtama(LIFTER_BOX_MENENGAH, 1);
  SEQ_DELAY_WITH_BODY(500, { SEQ_BASE_CURRENT(); });
}

void AmbilTongkatDanAssemblyDanGerak(void) {
  // Posisikan untuk ambil tongkat
  MoveServo(TONGKAT_ID, TONGKAT_AMBIL);
  oldPneumatikMotor(0, 0);
  LifterUtama(LIFTER_TONGKAT, 1);

  SetWorldPosition_RIZAL(0.0f, 0.0f, 0.0f);
  R1Motion_GoToPointFinalHeadingBase(500.0f, 400.0f, 0.0f,
                                     &R1Motion_ProfileFast);
  R1Motion_GoToPointFinalHeadingBase(1500.0f, 400.0f, 0.0f,
                                     &R1Motion_ProfileFast);
  R1Motion_GoToPointFinalHeadingBase(2100.0f, 400.0f, 0.0f,
                                     &R1Motion_ProfilePrecise);

//  R1Motion_GoToPointFinalHeadingBase(500.0f, 400.0f, 0.0f,
//                                     &R1Motion_ProfileNormal);
//  R1Motion_GoToPointFinalHeadingBase(2060.0f, 400.0f, 0.0f,
//                                     &R1Motion_ProfileNormal);

  SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_SLOW(); });
  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, { SEQ_BASE_SLOW(); });

  // grip, naikan tongkat, lalu berhenti
  oldPneumatikMotor(1, 0);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });

  LifterUtama(LIFTER_TONGKAT + 240, 1);
  SEQ_DELAY_WITH_BODY(400, { SEQ_BASE_CURRENT(); });

  //   SEQ_WAIT_PRESS_WITH_BODY(PS_Square, { SEQ_BASE_CURRENT(); });

  // posisikan tongkat horizontal untuk assembly dengan R2, mundurkan
  // tongkat, lalu berhenti
  MoveServo(TONGKAT_ID, TONGKAT_ASEMBEL);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });
  MoveServo(TONGKAT_ID, TONGKAT_ASEMBEL);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });
  MoveServo(TONGKAT_ID, TONGKAT_ASEMBEL);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });

  oldPneumatikMotor(1, 75);
  SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_CURRENT(); });
  oldPneumatikMotor(1, 0);

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, {
    SEQ_BASE_CURRENT();
    // kontrol manual untuk menyesuaikan dengan eror lapangan
    if (PS_Circle != 0) {
      oldPneumatikMotor(1, 75);
    } else if (PS_Triangle != 0) {
      oldPneumatikMotor(1, -75);
    } else {
      oldPneumatikMotor(1, 0);
    }
    //
  });

  // turunkanlifter agar tongkt sejajar dengan R2
  LifterUtama(LIFTER_TONGKAT_ASEMBLE, 1);
  posisiLifterSaatIni = LIFTER_TONGKAT_ASEMBLE;

  R1Motion_GoToPointFinalHeadingBase(3000.0f, 950.0f, 0.0f,
                                     &R1Motion_ProfileNormal);
  R1Motion_GoToPointFinalHeadingBase(4350.0f, 950.0f, 0.0f,
                                     &R1Motion_ProfileNormal);

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, {
    SEQ_BASE_SLOW();
    // kontrol manual untuk menyesuaikan dengan eror lapangan
    if (PS_Circle != 0) {
      oldPneumatikMotor(1, 75);
    } else if (PS_Triangle != 0) {
      oldPneumatikMotor(1, -75);
    } else {
      oldPneumatikMotor(1, 0);
    }
    Seq_Ttt_AdjustHeightBody();
//    if (PS_Cross != 0) {
//      setLED(1);
//      SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_SLOW(); });
//      setLED(0);
//    }
    //						if (PS_Cross != 0) {
    //							// Nyala
    // mAtikan LED, belum ada hardwarenya
    // } else {
    //
    //						}
  });

        setLED(1);
        SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_SLOW(); });
        setLED(0);
        SEQ_DELAY_WITH_BODY(200, { SEQ_BASE_SLOW(); });
  // tarik ke atas tongkat dengan spearhead yang sudah di assembly, lalu
  // selesai
  oldPneumatikMotor(1, 0);
  Tongkat_pitch = TONGKAT_ASEMBEL;
  MoveServo(TONGKAT_ID, TONGKAT_ASEMBEL);
  LifterUtama(LIFTER_BOX_MENENGAH, 1);
  SEQ_DELAY_WITH_BODY(500, { SEQ_BASE_CURRENT(); });
}

void TongkatZona3(void) {
  SEQ_WAIT_RELEASE_WITH_BODY(PS_Square, { SEQ_BASE_CURRENT(); });
  Tongkat_pitch = TONGKAT_ASEMBEL;
  MoveServo(TONGKAT_ID, Tongkat_pitch);
  oldPneumatikMotor(1, -75);
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });
  oldPneumatikMotor(1, 0);

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, {
    SEQ_BASE_SLOW();
    if (PS_Circle != 0) {
      oldPneumatikMotor(1, 75);
    } else if (PS_Triangle != 0) {
      oldPneumatikMotor(1, -75);
    }

    // Rifan
    else if (PS_L3 != 0) {
      oldPneumatikMotor(0, 0);
    } else {
      oldPneumatikMotor(1, 0);
    }
    Seq_Ttt_AdjustPitchTongkat(JOYS_L_ATAS, JOYS_L_BAWAH);
  });
}

void AmbilKFSdiLantai(void) {
  // mode mengambil kfs yang jatuh di lantai

  // posisikan awal ke atas kfs, gripper, lifter magazine, lifter
  LifterUtama(LIFTER_BOX_LANTAI, 1);
  LifterKFs(SLIDER_LANTAI);
  MoveServo(KFS_ID, KFS_LANTAI);
  SEQ_DELAY_WITH_BODY(800, { SEQ_BASE_CURRENT(); });

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, { SEQ_BASE_CURRENT(); });

  // grip kfs, lalu angkat kfs, karena agar saat rotasi kfs tidak
  // bergesekan dengan lantai
  setPn1(0);  // gripper tutup
  SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });
  posisiLifterSaatIni = LIFTER_TIKTAKTU;
  LifterUtama(posisiLifterSaatIni, 1);
  SEQ_DELAY_WITH_BODY(600, { SEQ_BASE_CURRENT(); });

  SEQ_WAIT_PRESS_WITH_BODY(PS_Square, { SEQ_BASE_CURRENT(); });
  // lalung langsung ke mode penaruhan kfs di tictaktoe
  Seq_PlaceOneKfs_TicTacToe();
}

void ManualControl(void) {
  T2_DBG_STATE(T2_STATE_MANUAL_BRANCH);
  PROF_BEGIN(PROF_T2_MANUAL_BRANCH);

  if (R1MeihuaManual_ProcessJoystickStep() != 0u) {
    PROF_END(PROF_T2_MANUAL_BRANCH);
    return;
  }

  /* ---------------- TONGKAT: Kotak step-by-step ---------------- */
  if (PS_Square != 0 && sudahAmbilTongkat == 0) {
    T2_DBG_STATE(T2_STATE_MANUAL_SQUARE);
    PROF_BEGIN(PROF_T2_MANUAL_SQUARE_TOTAL);
//    AmbilTongkatDanAssembly();
    AmbilTongkatDanAssemblyDanGerak();
    PROF_END(PROF_T2_MANUAL_SQUARE_TOTAL);
    sudahAmbilTongkat = 1;
  } else if (PS_Square != 0 && sudahAmbilTongkat != 0) {
    TongkatZona3();
  }

  /* ---------------- AMBIL KFS / SIMPAN KFS: Segitiga + Bundar
   * ---------------- */
  if (PS_Triangle != 0) {
    T2_DBG_STATE(T2_STATE_MANUAL_TRIANGLE);
    PROF_BEGIN(PROF_T2_MANUAL_TRIANGLE_TOTAL);

    // note kalau sebelum proses pengambilan kfs di meihua forrest untuk
    // menyesuaikan ketinggian lifter posisi awal aktuator
    LifterKFs(SLIDER_MUNDUR);
    MoveServo(2, KFS_FORREST);
    setPn1(1);  // gripper buka
    SEQ_DELAY_WITH_BODY(600, { SEQ_BASE_CURRENT(); });

    SEQ_WAIT_ANY_PRESS_WITH_BODY(PS_Triangle, PS_Square,
                                 { SEQ_BASE_CURRENT(); });
    // pemilihan mode selantunya, bsai untuk ambil di lantai jika KFS jatuh
    // di lantai saat proses masukan KFS di tiktaktoe
    if (PS_Triangle != 0) {
      T2_DBG_STATE(T2_STATE_MANUAL_TRIANGLE_FOREST);
      // mode ambil kfs di meihua forest

      // masukan sloder ke belakang kfs
      LifterKFs(SLIDER_MAJU);
      SEQ_DELAY_WITH_BODY(1300, { SEQ_BASE_CURRENT(); });

      // pemilihan mode untuk memasukan kfs ke magazine atau hanya dipengan
      // oleh gripper
      SEQ_WAIT_ANY_PRESS_WITH_BODY(PS_Triangle, PS_Circle,
                                   { SEQ_BASE_CURRENT(); });

      if (PS_Circle != 0) {
        T2_DBG_STATE(T2_STATE_MANUAL_TRIANGLE_MAGAZINE);
        // mode memasukan kfs di magazine

        Seq_SaveKfsToMagazine();

        SEQ_WAIT_RELEASE_WITH_BODY(PS_Circle, { SEQ_BASE_CURRENT(); });
      } else if (PS_Triangle != 0) {
        T2_DBG_STATE(T2_STATE_MANUAL_TRIANGLE_GRIPPER);
        // mode membawa kfs di gripper

        setPn1(0);  // gripper tutup
        SEQ_DELAY_WITH_BODY(300, { SEQ_BASE_CURRENT(); });

        LifterKFs(SLIDER_MUNDUR);

        kfsInGripper = 1U;

        SEQ_WAIT_RELEASE_WITH_BODY(PS_Triangle, { SEQ_BASE_CURRENT(); });
      }
    } else if (PS_Square != 0) {
      T2_DBG_STATE(T2_STATE_MANUAL_TRIANGLE_FLOOR);
      AmbilKFSdiLantai();
    }

    PROF_END(PROF_T2_MANUAL_TRIANGLE_TOTAL);
  }

  /* ---------------- TARUH TIC TAC TOE: X ---------------- */
  if (PS_Cross != 0) {
    T2_DBG_STATE(T2_STATE_MANUAL_CROSS);
    PROF_BEGIN(PROF_T2_MANUAL_CROSS_TOTAL);
    /*
     * Kalau tidak ada KFS di gripper tapi magazine berisi,
     * ambil dulu dari magazine.
     */
    if ((kfsInGripper == 0U) && (kfsMagazineFilled != 0U)) {
      Seq_TakeKfsFromMagazine();
    }

    /*
     * Kalau ada KFS di gripper, taruh KFS pertama.
     */
    if (kfsInGripper != 0U) {
      Seq_PlaceOneKfs_TicTacToe();
    } else {
      /*
       * Tidak ada KFS untuk ditaruh.
       * Tunggu X dilepas agar tidak repeat.
       */
      SEQ_WAIT_RELEASE_WITH_BODY(PS_Cross, { SEQ_BASE_CURRENT(); });
    }

    /*
     * Kalau setelah naruh KFS pertama ternyata magazine masih berisi,
     * ambil dari magazine lalu siapkan/taruh KFS berikutnya.
     */
    if (kfsMagazineFilled != 0U) {
      LifterKFs(SLIDER_MUNDUR);
      SEQ_DELAY_WITH_BODY(1500, { SEQ_BASE_CURRENT(); });
      Seq_TakeKfsFromMagazine();

      if (kfsInGripper != 0U) {
        Seq_PlaceOneKfs_TicTacToe();
      }
    }
    PROF_END(PROF_T2_MANUAL_CROSS_TOTAL);
  }

  /* ---------------- LIFTER UTAMA NON-BLOCKING: Joy R ---------------- */
  T2_DBG_STATE(T2_STATE_MANUAL_LIFTER_INPUT);
  PROF_BEGIN(PROF_T2_MANUAL_LIFTER_INPUT);
  if (JOYS_R_ATAS) {
    posisiLifterSaatIni = LIFTER_BOX_ATAS;
    LifterUtama(posisiLifterSaatIni, 1);
  } else if (JOYS_R_BAWAH) {
    posisiLifterSaatIni = LIFTER_BOX_BAWAH;
    LifterUtama(posisiLifterSaatIni, 1);
  } else if (JOYS_R_KANAN) {
    posisiLifterSaatIni = LIFTER_BOX_MENENGAH;
    LifterUtama(posisiLifterSaatIni, 1);
  } else if (JOYS_R_KIRI) {
    posisiLifterSaatIni = LIFTER_BOX_ATAS;
    LifterUtama(posisiLifterSaatIni, 0);
    LifterKFs(SLIDER_MAKSIMAL);
  } else if (BTN_R3) {
    posisiLifterSaatIni = LIFTER_BOX_PALING_BAWAH;
    LifterUtama(posisiLifterSaatIni, 0);
    LifterKFs(SLIDER_MAKSIMAL);
  }
  PROF_END(PROF_T2_MANUAL_LIFTER_INPUT);

  /* ---------------- BASE MANUAL ---------------- */
  /*
   * Final mapping:
   * L2 = normal / FastMode
   * R2 atau tidak L2 = SlowMode
   */
  T2_DBG_STATE(T2_STATE_MANUAL_BASE_CURRENT);
  PROF_BEGIN(PROF_T2_MANUAL_BASE_CURRENT);
  SEQ_BASE_CURRENT();
  PROF_END(PROF_T2_MANUAL_BASE_CURRENT);

  if (MISC_SHARE) {
    T2_DBG_STATE(T2_STATE_MISC_SHARE);
    PROF_BEGIN(PROF_T2_MISC_SHARE_TOTAL);
    SEQ_WAIT_RELEASE_WITH_BODY(MISC_SHARE, NULL;);

    /*
     * START ZERO R1 - tombol MISC_SHARE
     *
     * Maka saat tombol ini ditekan:
     * - PosX_World = 0
     * - PosY_World = 0
     * - RobotYaw tidak diubah.
     */
    RodaStop();
    SetWorldPosition_X_and_Y(0.0f, 0.0f);
    osDelay(5);

    PROF_END(PROF_T2_MISC_SHARE_TOTAL);
  }

  PROF_END(PROF_T2_MANUAL_BRANCH);
}

void HudaSemiAutomon(void) {
  static int flag_joy = 0;
  //	static int flag_soft =0;
  //	static int val_spiner = 2000;
  //	static int flag_kunci_semua = 0;

  T2_DBG_STATE(T2_STATE_AUTONOM_BRANCH);
  PROF_BEGIN(PROF_T2_AUTONOM_BRANCH);
  if (PS_Cross != 0) {
    T2_DBG_STATE(T2_STATE_AUTO_CROSS);
    PROF_BEGIN(PROF_T2_AUTO_CROSS_TOTAL);
    Kinematik(2, 2, 0);
    MoveServo(TONGKAT_ID, 1800);
    SetWorldPosition(0, 0, 0);

    MaxSpeed = 2000;
    LimitErrorJ = 700;
    LimitErrorH = 5;
    KPj = 40;
    KDj = 13;
    KPh = 1;
    JalanKeTitik(-300, 500, 0, 200, 0, 1000);

    MaxSpeed = 2000;
    LimitErrorJ = 700;
    LimitErrorH = 5;
    KPj = 40;
    KDj = 13;
    KPh = 1;
    JalanKeTitik(-2000, 400, 0, 200, 0, 1000);

    RodaStop();

    for (int a = 0; a <= 10; a++)
      while (PS_Cross == 0) {
        JoystickHeadingLock(SlowMode);

        if (PS_Square != 0) {
          oldPneumatikMotor(1, 0);
          oldPneumatikMotor(1, 0);
        }

        if (PS_Circle != 0) {
          MoveServo(TONGKAT_ID, 0);
        }
      }

    MaxSpeed = 2000;
    LimitErrorJ = 700;
    LimitErrorH = 5;
    KPj = 40;
    KDj = 13;
    KPh = 1;
    JalanKeTitik(-2900, 900, 0, 200, 0, 1000);
    MoveServo(TONGKAT_ID, 2650);
    LifterUtama(1792, 1);

    oldPneumatikMotor(1, -200);
    osDelay(500);
    oldPneumatikMotor(1, 0);

    MaxSpeed = 2000;
    LimitErrorJ = 700;
    LimitErrorH = 5;
    KPj = 40;
    KDj = 13;
    KPh = 1;
    JalanKeTitik(-3500, 900, 0, 200, 0, 1000);

    RodaStop();

    for (int a = 0; a <= 10; a++)
      while (PS_Cross == 0) {
        JoystickHeadingLock(FastMode);

        if (PS_Square != 0) {
          oldPneumatikMotor(1, -200);
        } else {
          oldPneumatikMotor(1, 0);
        }

        //				if(PS_Circle != 0)
        //					{
        //					MoveServo(TONGKAT_ID,0);
        //					}
      }

    PROF_END(PROF_T2_AUTO_CROSS_TOTAL);
  }
  //	  	if (PS_Triangle != 0) {
  //			SetKecepatanDanSudutRoda(0, 0, 0x80);
  //			flag_joy = 0;
  //			flag_soft = 0;
  //		}
  //
  //		if (PS_Circle != 0) {
  ////			SetKecepatanDanSudutRoda(0, 148, 0x80);
  //
  //			SetKecepatanDanSudutRoda(0, 30, 0x80);
  //			flag_joy = 1;
  //		}
  //
  //		if (PS_Cross != 0) {
  //			SetKecepatanDanSudutRoda(0, 148, 0x80);
  //
  ////			SetKecepatanDanSudutRoda(0, 30, 0x80);
  //			flag_joy = 1;
  //		}
  //
  //
  //		if (PS_Square != 0) {
  //			SetKecepatanDanSudutRoda(0, 30, 0x90);
  //		} else {
  //			SetKecepatanDanSudutRoda(10, 0, 0x90);
  //		}
  //
  //
  ////--------------------------------
  //		if (PS_L2 != 0) {
  //				SetKecepatanSpiner(1500, 1500, 1000, 1);
  //				flag_soft = 1;
  //		} else if (RX > 0) { //kanan
  //			if(flag_soft==1){
  //				val_spiner = 2600;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (RX < 0) { //kiri
  //			if(flag_soft==1){
  //				val_spiner = 2500;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (RY > 0) {//bawah
  //			if(flag_soft==1){
  //				val_spiner = 2800;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (RY < 0) {//atas
  //			if(flag_soft==1){
  //				val_spiner = 2700;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (LX > 0) {//kanan
  //			if(flag_soft==1){
  //				val_spiner = 3000;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (LX < 0) { //kiri
  //			if(flag_soft==1){
  //				val_spiner = 2900;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (LY > 0) { //bawah
  //			if(flag_soft==1){
  //				val_spiner = 3100;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		} else if (LY < 0) { //atas
  //			if(flag_soft==1){
  //				val_spiner = 3100;
  //				SetKecepatanSpiner(val_spiner, val_spiner,
  // 1200, 1); 				flag_joy = 1;
  //			}
  //		}
  //
  //		if (PS_L3 != 0) {
  //			val_spiner += 50;
  //			SetKecepatanSpiner(val_spiner, val_spiner, 1200, 1);
  //			SetKecepatanSpiner(val_spiner, val_spiner, 1200, 1);
  //			flag_joy = 1;
  //			osDelay(500);
  //		}
  //		else if (PS_R3 != 0) {
  //			val_spiner -= 50;
  //			SetKecepatanSpiner(val_spiner, val_spiner, 1200, 1);
  //			SetKecepatanSpiner(val_spiner, val_spiner, 1200, 1);
  //			flag_joy = 1;
  //			osDelay(500);
  //		}
  //
  //
  //		if (PS_R2 != 0) {
  //			SetKecepatanSpiner(0, 0, 0, 0);
  //			flag_soft = 0;
  //		}

  if (PS_Triangle != 0) {
    //	  			SetKecepatanDanSudutRoda(0, 0, 0x80);
    flag_joy = 0;
    //	  			flag_soft = 0;
  }

  if (PS_Cross != 0) {
    //	  			SetKecepatanDanSudutRoda(0, 148, 0x80);

    //			SetKecepatanDanSudutRoda(0, 30, 0x80);
    flag_joy = 1;
  }
  //
  if (flag_joy == 0) {
    T2_DBG_STATE(T2_STATE_AUTO_JOY_FAST);
    JoystickHeadingLock(FastMode);
  }
  if (flag_joy == 1) {
    T2_DBG_STATE(T2_STATE_AUTO_JOY_SLOW);
    JoystickHeadingLock(SlowMode);
  }
  PROF_END(PROF_T2_AUTONOM_BRANCH);
}

void RizalSemiAutomon(void) {
  osDelay(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
 * @brief  Function implementing the Task1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;) {
	PROF_BEGIN(PROF_TASK1_LOOP);
    PROF_BEGIN(PROF_TASK1_JOY_DEADZONE);

	PB1 = HAL_GPIO_ReadPin(PB_1_GPIO_Port, PB_1_Pin) == 0 ? 1 : 0;
	PB2 = HAL_GPIO_ReadPin(PB_2_GPIO_Port, PB_2_Pin) == 0 ? 1 : 0;
	PB3 = HAL_GPIO_ReadPin(PB_3_GPIO_Port, PB_3_Pin) == 0 ? 1 : 0;
	PB4 = HAL_GPIO_ReadPin(PB_4_GPIO_Port, PB_4_Pin) == 0 ? 1 : 0;

    joystick_apply_deadzone(&joysData, 12);
    PROF_END(PROF_TASK1_JOY_DEADZONE);

    if (BTN_R2 != 0) {
      ModeSpeedRizal = FastMode;
    } else if (BTN_L2 != 0) {
      ModeSpeedRizal = SlowMode;
    }

    if (MISC_PS != 0) {
      PS_INT = 1;
    } else {
      PS_INT = 0;
    }

    PROF_END(PROF_TASK1_LOOP);
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  uint8_t isManual = 0;
  uint8_t isBlue = 0;

  osDelay(500);

  T2_DBG_STATE(T2_STATE_INIT);
  PROF_BEGIN(PROF_T2_INIT);

  SetSpeed(KFS_ID, KFS_SPEED_SLOW);
  SetWorldPosition(0, 0, 0);
  Kinematik(2, 0, 0);
  osDelay(200);
  RodaStop();
  osDelay(500);
  startDMAusart();
  MoveServo(TONGKAT_ID, TONGKAT_HOME);
  MoveServo(KFS_ID, KFS_MAG);
  LifterUtama(0, 1);
  LifterKFs(0);
  DegKFS(0);
  Pneumatic(0, 0, 0);
  oldPneumatikMotor(0, 0);
  PROF_END(PROF_T2_INIT);

  T2_DBG_STATE(T2_STATE_MODE_SELECT);
  PROF_BEGIN(PROF_T2_MODE_SELECT);

  SEQ_WAIT_PRESS_WITH_BODY(BTN_KOTAK || BTN_LINGKARAN, {
  	      if (BTN_KOTAK != 0) {
  	    	isBlue = 1;
  	    	R1Motion_SetArenaMode(R1_MOTION_ARENA_BLUE);
  	      }

  	      if (BTN_LINGKARAN != 0) {
  	    	isBlue = 0;
  	    	R1Motion_SetArenaMode(R1_MOTION_ARENA_RED);
  	      }
  	    });


  SEQ_WAIT_PRESS_WITH_BODY(PS_Triangle || PS_Square, {
    if (PS_Triangle != 0) {
      isManual = 1;
    } else if (PS_Square != 0) {
      isManual = 0;
    }
  });



  PROF_END(PROF_T2_MODE_SELECT);

  /* Infinite loop */
  for (;;) {
    T2_DBG_LOOP_BEGIN();
    PROF_BEGIN(PROF_TASK2_LOOP);
    if (isManual == 0) {  // Autonom
      RizalSemiAutomon();
      //   HudaSemiAutomon();
    } else {  // Manual
      ManualControl();
    }
    PROF_END(PROF_TASK2_LOOP);
    T2_DBG_LOOP_END();
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_Start_uartSenderTask */
/**
 * @brief Function implementing the uartSenderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_uartSenderTask */
void Start_uartSenderTask(void *argument)
{
  /* USER CODE BEGIN Start_uartSenderTask */
  UartTxFrame_t frame = {0};

  for (;;) {
    /*
     * Menunggu queue berisi data.
     * Ini bukan polling, jadi task tidak membebani CPU saat queue kosong.
     */
    if (osMessageQueueGet(uartTxQueueHandle, &frame, NULL, osWaitForever) !=
        osOK) {
      continue;
    }

    for (;;) {
      PROF_BEGIN(PROF_UART_SENDER_FRAME);
      memcpy(uartTxDmaBuffer, frame.data, UART_TX_FRAME_LEN);

      /*
       * Bersihkan semaphore lama kalau ada.
       */
      while (osSemaphoreAcquire(uartTxDoneSemHandle, 0U) == osOK) {
        /* drain */
      }

      g_uart_tx_dma_busy = 1U;

      PROF_BEGIN(PROF_UART_DMA_START);
      HAL_StatusTypeDef _prof_uart_dma_status = HAL_UART_Transmit_DMA(
          UART_SENDER_HUART, uartTxDmaBuffer, UART_TX_FRAME_LEN);
      PROF_END(PROF_UART_DMA_START);

      if (_prof_uart_dma_status == HAL_OK) {
        g_uart_tx_dma_start_count++;

        PROF_BEGIN(PROF_UART_WAIT_DONE);
        osStatus_t _prof_uart_wait_status =
            osSemaphoreAcquire(uartTxDoneSemHandle, UART_TX_DMA_TIMEOUT_MS);
        PROF_END(PROF_UART_WAIT_DONE);

        if (_prof_uart_wait_status != osOK) {
          g_uart_tx_dma_timeout_count++;
          g_uart_tx_dma_busy = 0U;

          HAL_UART_AbortTransmit(UART_SENDER_HUART);
        }
      } else {
        g_uart_tx_dma_error_count++;
        g_uart_tx_dma_busy = 0U;

        HAL_UART_AbortTransmit(UART_SENDER_HUART);
        osDelay(1U);
      }

      UartSender_UpdateQueueDebug();

      /*
       * Ambil frame berikutnya tanpa menunggu.
       * Kalau queue kosong, keluar dan kembali tidur.
       */
      if (osMessageQueueGet(uartTxQueueHandle, &frame, NULL, 0U) != osOK) {
        PROF_END(PROF_UART_SENDER_FRAME);
        break;
      }

      PROF_END(PROF_UART_SENDER_FRAME);
    }
  }
  /* USER CODE END Start_uartSenderTask */
}

/* USER CODE BEGIN Header_Start_ImuTask */
/**
 * @brief Function implementing the ImuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_ImuTask */
void Start_ImuTask(void *argument)
{
  /* USER CODE BEGIN Start_ImuTask */
  uint8_t byte = 0;
  /* Infinite loop */
  for (;;) {
    PROF_BEGIN(PROF_IMU_LOOP);
    while (ImuRing_ReadByte(&byte) != 0U) {
      PROF_BEGIN(PROF_IMU_PARSE_BYTE);
      ParsingDataIMU(byte);
      PROF_END(PROF_IMU_PARSE_BYTE);
    }
    PROF_END(PROF_IMU_LOOP);
    osDelay(1);
  }
  /* USER CODE END Start_ImuTask */
}

/* USER CODE BEGIN Header_StartT_MonitorTask */
/**
 * @brief Function implementing the MonitorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartT_MonitorTask */
void StartT_MonitorTask(void *argument)
{
  /* USER CODE BEGIN StartT_MonitorTask */
  TaskStatus_t* pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  unsigned long ulTotalRunTime;
  float runtime_percentage;
  uxArraySize = uxTaskGetNumberOfTasks();
  pxTaskStatusArray =
      pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));  // a little bit scary!

  /* Infinite loop */
  for (;;) {
    osDelay(5000);

    if (pxTaskStatusArray != NULL) {
      uxArraySize =
          uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

      printf("\n\n------++++++------\nTask count = %lu\n", uxArraySize);
      printf("No       Name          P  S   Usage     Count     HW\n");

      for (x = 0; x < uxArraySize; x++) {
        runtime_percentage =
            (float)(100 * (float)pxTaskStatusArray[x].ulRunTimeCounter /
                    (float)ulTotalRunTime);

        printf("Task %2lu: %-12s %2lu %2d %8.4f (%8lu) %4i\n", x,
               pxTaskStatusArray[x].pcTaskName,
               pxTaskStatusArray[x].uxCurrentPriority,
               pxTaskStatusArray[x].eCurrentState, runtime_percentage,
               pxTaskStatusArray[x].ulRunTimeCounter,
               pxTaskStatusArray[x].usStackHighWaterMark);
      }

      //				vPortFree(pxTaskStatusArray);
      PROF_PRINT();

      printf(
          "EncA=%d EncB=%d | XR=%.1f YR=%.1f | XW=%.1f YW=%.1f "
          "Yaw=%.1f | EX=%.1f EY=%.1f EJ=%.1f EH=%.1f Sudut=%.1f "
          "Vx=%.1f Vy=%.1f Om=%.1f\r\n",
          R1_DBG_SpeedEncA, R1_DBG_SpeedEncB, R1_DBG_PosX_Robot,
          R1_DBG_PosY_Robot, PosX_World, PosY_World, RobotYaw, R1_DBG_ErrorX,
          R1_DBG_ErrorY, R1_DBG_ErrorJ, R1_DBG_ErrorH, R1_DBG_Sudut, R1_DBG_Vx,
          R1_DBG_Vy, R1_DBG_Omega);
    } else {
      printf("Unable to allocate stack space\n");
    }
  }
  /* USER CODE END StartT_MonitorTask */
}

/* USER CODE BEGIN Header_StartOdomTask_RIZAL */
/**
 * @brief Function implementing the OdomTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOdomTask_RIZAL */
void StartOdomTask_RIZAL(void *argument)
{
  /* USER CODE BEGIN StartOdomTask_RIZAL */
  uint32_t tick;
  const uint32_t period_ms = 1U;

  while (g_imu_valid != 1)
    osDelay(2);

  SetWorldPosition_RIZAL(0.0f, 0.0f, 0.0f);

  tick = osKernelGetTickCount();

  for (;;) {
    PROF_BEGIN(PROF_ODOM_RIZAL);

    BacaPosisiEncoderDeadwheel60_RIZAL();

    PROF_END(PROF_ODOM_RIZAL);

    tick += period_ms;
    osDelayUntil(tick);
  }
  /* USER CODE END StartOdomTask_RIZAL */
}

/* USER CODE BEGIN Header_Start_ARGBtask */
/**
* @brief Function implementing the ARGBtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ARGBtask */
void Start_ARGBtask(void *argument)
{
  /* USER CODE BEGIN Start_ARGBtask */
	static uint8_t hue = 0;
	uint8_t step_animasi = 0;
	uint8_t state_polisi = 0;

	ARGB_Init();
	ARGB_SetBrightness(105); // Set global brightness to 40%
	osDelay(50);
	/* Infinite loop */
	for (;;) {
//		osDelay(1);
//
//		for (int i = 0; i < NUM_PIXELS; i++) {
//			// Set setiap LED dengan hue yang bergeser
//			ARGB_SetHSV(i, hue + (i * 256 / NUM_PIXELS), 255, 255);
//		}
//		ARGB_Show(); // kirim data ke LED
//		hue++;		 // geser hue tiap frame
//		osDelay(50);

		ARGB_FillRGB(2, 96, 204);
		ARGB_Show();
		osDelay(500);
		ARGB_FillRGB(91, 0, 166);
		ARGB_Show();
		osDelay(500);
	}
  /* USER CODE END Start_ARGBtask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /*
   10ms
   84-1
   10000-1

   100ms
   84-1
   100000-1
   */
  //
  if (htim == &htim7) {
    //      BacaPosisiEncoder();
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
