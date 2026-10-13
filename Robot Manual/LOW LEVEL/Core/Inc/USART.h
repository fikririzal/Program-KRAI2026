/*
 * USART.h
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "main.h"
#include "myDEF.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern volatile uint16_t sick_sensor_left_back;
extern volatile uint16_t sick_sensor_right_back;
extern volatile uint16_t sick_sensor_left;
extern volatile uint16_t sick_sensor_right;


extern int LifterKFs(float heading);
extern int LifterUtama(float heading, uint8_t mode_);
extern int PneumaticMotor(int8_t speed, float heading, uint8_t led);
extern int DegKFS(float heading);
extern int Pneumatic(float pn1, float pn2, float pn3);


extern int Blocking(uint8_t val);


extern int SetKecepatanSpiner(float Spin1, float Spin2, float Spin3, uint8_t Flag);
extern void send_pneumatic(short pneumatic);

extern void kirim_data_posisiRobotToPC(signed char id_sendToPC, float speedEA, float speedEB, uint16_t right, uint16_t left, uint8_t data_all_sensor_1);
extern void kirim_data_posisiRobotToPC_baru(uint8_t flag);

extern void sudut_pick_ball(float heading, uint8_t SlaveAddress);
//USART Roda
extern int SetKecepatanDanSudutRoda(signed char speed, float heading, uint8_t SlaveAddress);
extern void SetKpKd(uint8_t Kp, uint8_t Kd, uint8_t SlaveAddress);
extern void RequestSudut(uint8_t SlaveAddress);
extern int SetSlaveTurretControl(uint8_t IdSlave,uint8_t Flag, float Data);
extern void SetSlaveSpinner(uint8_t IdSpin, uint8_t SpinnerAddres,short KecepatanSpinner);
extern void SetSlaveLifterGripper(uint8_t Fungsi,uint8_t Speed);
extern void SetSlaveSudutPutarLiftGripper(uint8_t Speed, float SudutPutar);
extern void SetSlaveServoSystem(uint8_t Fungsi);

extern void RakOnly( uint8_t IdSlave, short data);

extern uint8_t volatile Rx_RS485[1];
extern volatile uint8_t	DataRemote1, DataRemote2,DataRemote3, DataRemote4;
//extern volatile int8_t	LX , LY , RX, RY ;
extern volatile uint8_t  Rx_Remote[1], Rx_IMU[1], Rx_VL[1], Rx_GUI[1];  //  creating a buffer of 10 bytes

extern void ParsingDataJoystick(uint8_t c);
//extern void ParsingDataIMU(uint8_t c);
extern void ParsingDataVL(uint8_t c);
extern void ParsingDataGUI(uint8_t c);
extern void ParsingDataTOF(uint8_t c);

//extern float DataYawIMU, DataPitchIMU, DataRollIMU;
extern uint8_t bufImu_0,bufImu_1,bufImu_2,bufImu_3;

extern uint16_t TOF1,TOF2,TOF3;
extern uint8_t Target1GUI,Target2GUI,Target3GUI,Target4GUI;

extern _Bool FlagBuzzer;
extern _Bool FlagKirimBluetooth;



extern uint8_t DataDMA[7];





#define USART_FLAG_CTS                       ((uint16_t)0x0200)
#define USART_FLAG_LBD                       ((uint16_t)0x0100)
#define USART_FLAG_TXE                       ((uint16_t)0x0080)
#define USART_FLAG_TC                        ((uint16_t)0x0040)
#define USART_FLAG_RXNE                      ((uint16_t)0x0020)
#define USART_FLAG_IDLE                      ((uint16_t)0x0010)
#define USART_FLAG_ORE                       ((uint16_t)0x0008)
#define USART_FLAG_NE                        ((uint16_t)0x0004)
#define USART_FLAG_FE                        ((uint16_t)0x0002)
#define USART_FLAG_PE                        ((uint16_t)0x0001)
#define IS_USART_FLAG(FLAG) (((FLAG) == USART_FLAG_PE) || ((FLAG) == USART_FLAG_TXE) || \
                             ((FLAG) == USART_FLAG_TC) || ((FLAG) == USART_FLAG_RXNE) || \
                             ((FLAG) == USART_FLAG_IDLE) || ((FLAG) == USART_FLAG_LBD) || \
                             ((FLAG) == USART_FLAG_CTS) || ((FLAG) == USART_FLAG_ORE) || \
                             ((FLAG) == USART_FLAG_NE) || ((FLAG) == USART_FLAG_FE))

#define IS_USART_CLEAR_FLAG(FLAG) ((((FLAG) & (uint16_t)0xFC9F) == 0x00) && ((FLAG) != (uint16_t)0x00))

#define IS_USART_BAUDRATE(BAUDRATE) (((BAUDRATE) > 0) && ((BAUDRATE) < 7500001))
#define IS_USART_ADDRESS(ADDRESS) ((ADDRESS) <= 0xF)
#define IS_USART_DATA(DATA) ((DATA) <= 0x1FF)

#endif /* INC_USART_H_ */
