#include "main.h"
#include "myDEF.h"
#include "USART.h"
#include "Joystick.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

#define MAP(x, in_min, in_max, out_min, out_max)                                                                       \
    (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
#define MAP_FLOAT(x, in_min, in_max, out_min, out_max)                                                                 \
    (((float)(x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

typedef struct {
	uint16_t identifier;
	int8_t LX;
	int8_t LY;
	int8_t RX;
	int8_t RY;
	uint8_t THR;
	uint8_t BRK;
	uint16_t BTN;
	uint8_t MISC;
	uint8_t DPAD;
	uint8_t BAT;
	uint8_t SUM;
} Joystick_p;

typedef struct {
	// +
	int8_t LXright;
	int8_t LYdown;
	int8_t RXright;
	int8_t RYdown;
	// -
	int8_t LXleft;
	int8_t LYup;
	int8_t RXleft;
	int8_t RYup;
} configJoys_t;

#define FRAME_START1 0xFE
#define FRAME_START2 0xCA
#define FRAME_SIZE 14
#define HEADER_SIZE 2
#define TOTAL_FRAME (HEADER_SIZE + FRAME_SIZE)

extern Joystick_p joysData;
#define BUFFERSIZE_PDATA 14
extern uint8_t pData[BUFFERSIZE_PDATA];
extern uint32_t intDMA;
extern uint32_t parseDelta;

extern float RobotYaw;
extern float speed, steer;
extern volatile int8_t LX, LY, RX, RY;

// Program mas huda pakau USART1 dan DMA2 Stream 2
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern configJoys_t configJoys;
#define BUFFERSIZE_RX_DMA 28
extern uint8_t usart_rx_dma_buffer[BUFFERSIZE_RX_DMA];
extern uint8_t debug_rx_dma_buffer[BUFFERSIZE_RX_DMA];

#define JOYS_THRESHOLD 50

// Tombol utama (BTN)
#define BTN_SILANG     ((joysData.BTN & 1) != 0)
#define BTN_LINGKARAN  ((joysData.BTN & 2) != 0)
#define BTN_KOTAK      ((joysData.BTN & 4) != 0)
#define BTN_SEGITIGA   ((joysData.BTN & 8) != 0)
#define BTN_L1         ((joysData.BTN & 16) != 0)
#define BTN_R1         ((joysData.BTN & 32) != 0)
#define BTN_L2         ((joysData.BTN & 64) != 0)
#define BTN_R2         ((joysData.BTN & 128) != 0)
#define BTN_L3         ((joysData.BTN & 256) != 0)
#define BTN_R3         ((joysData.BTN & 512) != 0)

// Analog kanan (Right stick)
#define JOYS_R_ATAS    ((joysData.RY < -JOYS_THRESHOLD))
#define JOYS_R_BAWAH   ((joysData.RY > JOYS_THRESHOLD))
#define JOYS_R_KIRI    ((joysData.RX < -JOYS_THRESHOLD))
#define JOYS_R_KANAN   ((joysData.RX > JOYS_THRESHOLD))

// Analog kiri (Left stick)
#define JOYS_L_ATAS    ((joysData.LY < -JOYS_THRESHOLD))
#define JOYS_L_BAWAH   ((joysData.LY > JOYS_THRESHOLD))
#define JOYS_L_KIRI    ((joysData.LX < -JOYS_THRESHOLD))
#define JOYS_L_KANAN   ((joysData.LX > JOYS_THRESHOLD))

// D-Pad
#define DPAD_ATAS      ((joysData.DPAD & 1) != 0)
#define DPAD_BAWAH     ((joysData.DPAD & 2) != 0)
#define DPAD_KANAN     ((joysData.DPAD & 4) != 0)
#define DPAD_KIRI      ((joysData.DPAD & 8) != 0)

// Tombol Misc (PS, Share, Options)
#define MISC_PS        ((joysData.MISC & 1) != 0)
#define MISC_SHARE     ((joysData.MISC & 2) != 0)
#define MISC_OPTIONS   ((joysData.MISC & 4) != 0)

void joysProcessData(size_t start, size_t end);
void startDMAjoys(void);
void parseDataJoys(void);
uint8_t pop_and_shift(uint8_t *buffer, size_t buffSize);
void readLikeArduino(size_t startPos, size_t endPos, uint8_t forceReset);
void addsThenParse(uint8_t c, uint8_t *forcereset);
void breakpoint(size_t oldPos, size_t pos);
int8_t apply_deadzone(int8_t value, uint8_t deadzone);
void joystick_apply_deadzone(Joystick_p *js, const uint8_t deadzone);
void runJoysRizal(void);
void rizalJoystickHeadingLock(uint8_t Mode);

void startDMAusart(void);
void usart_rx_check(void);
void usart_process_data(const void *data, size_t len);
void usart_send_string(const char *str);

void myHalfDoneTf(DMA_HandleTypeDef *uartDma);
void myAllDoneTf(DMA_HandleTypeDef *uartDma);

void my_DMA_Stream_IRQHandler(void);
void my_USART_IRQHandler(void);
