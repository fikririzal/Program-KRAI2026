#ifndef	myDEF
#define	myDEF

#define SetBit(x,y) 	x->BSRR = (1<<y)
#define ClrBit(x,y) 	x->BRR  = (1<<y)
#define Pin(Port,x)	  (Port->IDR&(1<<x))

#define SetA(y) 		GPIOA->BSRR = (1<<y)
#define SetB(y) 		GPIOB->BSRR = (1<<y)
#define SetC(y) 		GPIOC->BSRR = (1<<y)
#define SetD(y) 		GPIOD->BSRR = (1<<y)
#define SetE(y) 		GPIOE->BSRR = (1<<y)
#define SetF(y) 		GPIOF->BSRR = (1<<y)
#define SetG(y) 		GPIOG->BSRR = (1<<y)

#define ClrA(y) 		GPIOA->BRR  = (1<<y)
#define ClrB(y) 		GPIOB->BRR  = (1<<y)
#define ClrC(y) 		GPIOC->BRR  = (1<<y)
#define ClrD(y) 		GPIOD->BRR  = (1<<y)
#define ClrE(y) 		GPIOE->BRR  = (1<<y)
#define ClrF(y) 		GPIOF->BRR  = (1<<y)
#define ClrG(y) 		GPIOG->BRR  = (1<<y)


// mengakses LED pada modul BLDC LED
#define	LedHallA(o)	(o ? SetB(5) : ClrB(5))
#define	LedHallB(o)	(o ? SetB(6) : ClrB(6))
#define	LedHallC(o)	(o ? SetB(7) : ClrB(7))

#define	SetLedHallA		SetBit(GPIOB,5)
#define	SetLedHallB		SetBit(GPIOB,6)
#define	SetLedHallC		SetBit(GPIOB,7)
#define	ClrLedHallA		ClrBit(GPIOB,5)
#define	ClrLedHallB		ClrBit(GPIOB,6)	
#define	ClrLedHallC		ClrBit(GPIOB,7)	

//Enable untuk  motor
#define EnableA(o)	(o ? SetB(12) : ClrB(12))
#define EnableB(o)	(o ? SetB(14) : ClrB(14))
#define EnableC(o)	(o ? SetB(15) : ClrB(15))
	
// konstanta konversi radian dan derajat
#define RAD2DEG     (float) 57.295779513082320876798154814105  
#define DEG2RAD     (float) 0.01745329251994329576923690768489

#ifndef 	int8_t
#define		int8_t		signed char
#endif
#ifndef 	uint8_t
#define		uint8_t		unsigned char
#endif
#ifndef 	int16_t
#define		int16_t		signed short
#endif
#ifndef 	uint16_t
#define		uint16_t	unsigned short
#endif
#ifndef 	int32_t
#define		int32_t		signed long
#endif
#ifndef 	uint32_t
#define		uint32_t	unsigned long
#endif

//AMS5600 PWM INPUT
#define TIMCLOCK 72000000
#define PRESCALAR 72

//#define EeKpKdAddress							0
//#define EeOffsetSudutRodaAddress	8


#define Slave1 0x01
#define Slave2 0x02
#define Slave3 0x03
#define Slave4 0x04

//// pilih salah satu dari 3 opsi di ats untuk mengalamati motor roda putar yang akan di program
#define	RodaId			Slave1 







union   Data16Bit
{   int8_t      d8s[2];     // 2 buah data signed char
    uint8_t     d8u[2];     // 4 buah data unsigned char
    int16_t     d16s;       // 1 buah data signed short integer
    uint16_t    d16u;       // 1 buah data unsigned short integer
}__attribute__ ((__packed__));

union   Data32Bit
{   int8_t      d8s[4];     // 4 buah data signed char
    uint8_t     d8u[4];     // 4 buah data unsigned char
    int16_t     d16s[2];    // 2 buah data signed short integer
    uint16_t    d16u[2];    // 2 buah data unsigned short integer
    int32_t     d32s;       // 1 buah data signed long integer
    uint32_t    d32u;       // 1 buah data unsigned long integer
    float       df;         // 1 buah data float
}__attribute__ ((__packed__));
#endif

