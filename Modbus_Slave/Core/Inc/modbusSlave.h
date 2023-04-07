//modbusSlave.h

#ifndef MODBUSSLAVE_H_
#define MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f4xx_hal.h"

#define SLAVE_ID 9

#define OTHER_FUNCTION 				0x00
#define ILLEGAL_FUNCTION 			0x01
#define ILLEGAL_DATA_ADDRESS 	0x02
#define ILLEGAL_DATA_VALUE 		0x03	
#define RegSize 20
#define CoilSize 5
#define InputSize 5

static uint16_t Holding_Registers_Database[RegSize]={
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   40001-40010
		1234,  5678,  2345,  1786,  12347, 1975,  1071,  1987,  1456,  4567,   // 10-19 40011-40020
};

static const uint16_t Input_Registers_Database[RegSize]={
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   30001-30010
		1234,  5678,  2345,  1786,  12347, 1975,  1071,  1987,  1456,  4567,   // 10-19 30011-30020

};

static uint8_t Coils_Database[CoilSize]={
		  0x9C,    0x65,    0x9C,    0x82,    0x1A // 0-39	1-40
 // 0b10011100 0b01100101 0b10011100 0b10000010 0b00011010
};

static const uint8_t Discrete_Inputs_Database[InputSize]={
		  0x9C,    0x65,    0x9C,    0x82,    0x1A // 0-39	10001-10040
 // 0b10011100 0b01100101 0b10011100 0b10000010 0b00011010
};

uint8_t readCoils (void);
uint8_t readDiscreteInputs (void);
uint8_t readHoldingRegs (void);
uint8_t readInputRegs (void);
uint8_t writeSingleCoil (void);
uint8_t writeSingleReg (void);
uint8_t	writeMultiCoils (void);
uint8_t	writeMultiRegs (void);

void modbus_exception (uint8_t exceptionCode);
void modbus_other (uint8_t otherFunction);

#endif /* MODBUSSLAVE_H_ */
