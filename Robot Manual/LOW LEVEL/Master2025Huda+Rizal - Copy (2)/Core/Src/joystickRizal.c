#include "joystickRizal.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "myDEF.h"
#include "Swerve.h"
#include "Profiler.h"

Joystick_p joysData;
uint8_t pData[BUFFERSIZE_PDATA];
uint8_t sum_debug = 0;

uint32_t intDMA = 0;
uint32_t parseDelta = 0;

configJoys_t configJoys;
uint8_t usart_rx_dma_buffer[BUFFERSIZE_RX_DMA];
uint8_t virtualBlink = 0;
uint8_t debug_rx_dma_buffer[BUFFERSIZE_RX_DMA];
volatile int8_t	LX , LY , RX, RY ;

/* Fungsi untuk ambil buffer[0], lalu geser buffer */
uint8_t pop_and_shift(uint8_t *buffer, size_t buffSize)
{
    if (buffSize == 0)
    {
        return 0; // atau error code
    }

    uint8_t old_data = buffer[0];

    // Geser semua data ke kiri
    for (int i = 0; i < buffSize - 1; i++)
    {
        buffer[i] = buffer[i + 1];
    }

    buffer[buffSize - 1] = 0; // Kosongkan posisi terakhir (opsional)

    return old_data;
}

void readLikeArduino(size_t startPos, size_t endPos, uint8_t forceReset)
{
    PROF_BEGIN(PROF_JOY_READ_LIKE_ARDUINO);
    uint8_t workBuffer[BUFFERSIZE_RX_DMA] = {0};
    memcpy(workBuffer, usart_rx_dma_buffer, BUFFERSIZE_RX_DMA);
    for (size_t var = 0; var < startPos; ++var)
    {
        pop_and_shift(workBuffer, BUFFERSIZE_RX_DMA);
    }
    uint8_t looping = 1;
    size_t pos = 0;
    while (looping)
    {
        uint8_t data = pop_and_shift(workBuffer, BUFFERSIZE_RX_DMA);
        addsThenParse(data, &forceReset);
        pos += 1;
        if (pos >= BUFFERSIZE_RX_DMA)
        {
            looping = 0;
        }

        size_t i = endPos + 1;
        if (endPos != 0 && pos >= i)
        {
            looping = 0;
        }
    }
    PROF_END(PROF_JOY_READ_LIKE_ARDUINO);
}

void addsThenParse(uint8_t c, uint8_t *forcereset)
{
    static uint8_t sCounter = 0, sflag = 0;

    if (*forcereset)
    {
        *forcereset = 0;
        sCounter = 0;
        sflag = 0;
    }

    switch (sflag)
    {
    case 0:
        if (c == FRAME_START1)
        {
            sflag = 1;
            sCounter = 0;
            pData[sCounter] = c;
        }
        break;

    case 1:
        if (c == FRAME_START2)
        {
            sflag = 2;
            pData[1] = c;
            sCounter = 2;
        }
        else
        {
            sflag = 0;
        }
        break;

    case 2:
        pData[sCounter++] = c;
        if (sCounter == 14)
        {
        	uint8_t sum = 0;
        	for (int i = 2; i <= 12; ++i) {
        	    sum += pData[i];
        	}

        	sum_debug=pData[13];

//        	if (pData[13] == sum) {
        	    parseDataJoys();
//        	}
            sflag = 0; // reset state
        }
        break;

    default:
        sflag = 0;
        break;
    }
}

void parseDataJoys(void)
{
    PROF_BEGIN(PROF_JOY_PARSE_PACKET);
    static uint32_t lastParse;
    parseDelta = HAL_GetTick() - lastParse;
    lastParse = HAL_GetTick();
    memcpy((void *)&joysData, pData, FRAME_SIZE);
    PROF_END(PROF_JOY_PARSE_PACKET);

}

void usart_rx_check(void)
{
    PROF_BEGIN(PROF_JOY_DMA_CHECK);
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (pos != old_pos)
    { /* Check change in received data */
//        memcpy(debug_rx_dma_buffer, usart_rx_dma_buffer, BUFFERSIZE_RX_DMA);
        if (pos > old_pos)
        { /* Current position is over previous one */

            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            //			joysProcessData();
            //			joysProcessData(old_pos, pos);
            readLikeArduino(0, 0, 1);
            //			breakpoint(old_pos, pos);

            //			usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        }
        else
        {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */

            //			usart_process_data(&usart_rx_dma_buffer[old_pos],
            //			ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            //			breakpoint(old_pos, pos);
            readLikeArduino(0, 0, 1);
            //			breakpoint(old_pos, pos);
            if (pos > 0)
            {

                //				usart_process_data(&usart_rx_dma_buffer[0], pos);

                readLikeArduino(0, 0, 0);
            }
        }
        old_pos = pos; /* Save current position as old for next transfers */
    }
    PROF_END(PROF_JOY_DMA_CHECK);
}

// void my_DMA_Stream_IRQHandler(void) {
//	/* Check half-transfer complete interrupt */
//	if (LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_1)
//			&& LL_DMA_IsActiveFlag_HT1(DMA2)) {
//		LL_DMA_ClearFlag_HT1(DMA2); /* Clear half-transfer complete flag */
//		usart_rx_check(); /* Check for data to process */
//	}

//	/* Check transfer-complete interrupt */
//	if (LL_DMA_IsEnabledIT_TC(DMA2, LL_DMA_STREAM_1)
//			&& LL_DMA_IsActiveFlag_TC1(DMA2)) {
//		LL_DMA_ClearFlag_TC1(DMA2); /* Clear transfer complete flag */
//		usart_rx_check(); /* Check for data to process */
//	}
//
//

//	/* Implement other events when needed */
//}

///**
// * \brief           USART3 global interrupt handler
// */
// void my_USART_IRQHandler(void) {
//	/* Check for IDLE line interrupt */
//	if (LL_USART_IsEnabledIT_IDLE(USART6)
//			&& LL_USART_IsActiveFlag_IDLE(USART6)) {
//		LL_USART_ClearFlag_IDLE(USART6); /* Clear IDLE line flag */
//		usart_rx_check(); /* Check for data to process */
//	}

//	/* Implement other events when needed */
//}

void startDMAusart(void)
{
    //    LL_USART_Disable(USART6);

    //    LL_USART_InitTypeDef USART_InitStruct = {0};
    //    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    //    /* Peripheral clock enable */
    //    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
    //    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    //    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    //    /**USART6 GPIO Configuration
    //     PA11   ------> USART6_TX
    //     PA12   ------> USART6_RX
    //     */
    //    GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
    //    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    //    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    //    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    //    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    //    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    //    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //    /* USART3 DMA Init */
    //    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_5);
    //    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    //    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    //    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
    //    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    //    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    //    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    //    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    //    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

    //    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART6));
    //    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)usart_rx_dma_buffer);
    //    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, ARRAY_LEN(usart_rx_dma_buffer));

    //    /* Enable HT & TC interrupts */
    //    LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_1);
    //    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);

    //    /* DMA interrupt init */
    //    /* DMA2_Stream1_IRQn interrupt configuration */
    //    NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    //    NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    //    /* USART configuration */
    //    USART_InitStruct.BaudRate = 115200;
    //    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    //    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    //    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    //    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    //    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    //    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    //    LL_USART_Init(USART6, &USART_InitStruct);
    //    LL_USART_ConfigAsyncMode(USART6);
    //    LL_USART_EnableDMAReq_RX(USART6);
    //    LL_USART_EnableIT_IDLE(USART6);

    //    /* USART interrupt */
    //    NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    //    NVIC_EnableIRQ(USART6_IRQn);

    //    /* Enable USART and DMA */
    //    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
    //    LL_USART_Enable(USART6);

    // Program mas huda pakau USART1 dan DMA2 Stream 2

//    HAL_DMA_RegisterCallback(&hdma_usart1_rx, HAL_DMA_XFER_HALFCPLT_CB_ID, myHalfDoneTf);
//    HAL_DMA_RegisterCallback(&hdma_usart1_rx, HAL_DMA_XFER_CPLT_CB_ID, myAllDoneTf);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart_rx_dma_buffer, BUFFERSIZE_RX_DMA);
}
// Fungsi helper untuk menerapkan deadzone
int8_t apply_deadzone(int8_t value, uint8_t deadzone) {
    if (abs(value) < deadzone) {
        return 0;
    }

    // Normalisasi ulang rentang agar tetap -127..127 setelah deadzone
    if (value > 0) {
        return (int8_t)(((int16_t)(value - deadzone) * 127) / (127 - deadzone));
    } else {
        return (int8_t)(((int16_t)(value + deadzone) * 127) / (127 - deadzone));
    }
}

// Fungsi utama untuk memberi deadzone ke semua sumbu analog
void joystick_apply_deadzone(Joystick_p *js, const uint8_t deadzone) {
    LX = apply_deadzone(js->LX, deadzone);
    LY = apply_deadzone(js->LY, deadzone);
    RX = apply_deadzone(js->RX, deadzone);
    RY = apply_deadzone(js->RY, deadzone);


}
