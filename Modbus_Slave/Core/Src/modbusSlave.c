//modbusSlave.c

#include "modbusSlave.h"
#include "string.h"

extern uint8_t RxData[50];
extern uint8_t TxData[50];
extern UART_HandleTypeDef huart2;

void sendData (uint8_t *data, int size)
{
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc&0xFF;   // CRC LOW
	data[size+1] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart2, data, size+2, 1000);
}

void modbus_exception (uint8_t exceptionCode)
{
	//| SLAVE_ID | FUNCTION_CODE | EXCEPTION CODE | CRC
	//| 1 BYTE   |  1 BYTE       |  1 BYTE        | 2 BYTES 
	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = exceptionCode; //load exception
	sendData(TxData, 3);
}

void modbus_other (uint8_t otherFunction)
{
	//| SLAVE_ID | FUNCTION_CODE | EXCEPTION CODE | CRC
	//| 1 BYTE   |  1 BYTE       |  1 BYTE        | 2 BYTES 
	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = otherFunction; //load other function
	sendData(TxData, 3);
}

uint8_t readCoils (void) //Case 0x01
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Coils Address

	uint16_t numCoils = ((RxData[4]<<8)|RxData[5]);   // number to coils master has requested
	if (numCoils > 2000) {
		modbus_exception(ILLEGAL_DATA_VALUE);
		return 0;
	}
	
	if (startAddr + numCoils - 1 > CoilSize*8) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	memset (TxData, '\0', 256);
	
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils/8) + (numCoils%8>0 ? 1:0);  // Byte count
	
	int indx = 3; 
	
	int starByte = startAddr/8;
	uint16_t bitPosition = startAddr%8;
	int indxPosition = 0;

	// Load the actual data into TxData buffer
	for (int i=0; i<numCoils; i++)   
	{
		TxData[indx] |= ((Coils_Database[starByte] >> bitPosition) &0x01 ) << indxPosition;
		indxPosition++; bitPosition++;
		if (indxPosition > 7){
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7){
			bitPosition = 0;
			starByte++;
		}

	}
	
	if (numCoils%8 != 0) indx++;
	sendData(TxData, indx); 
	return 1; 
	
	// Test with 0b10011100 0b01100101 0b10011100 0b10000010 0b00011010
	// startAddr = 9; numCoils = 10 => startByte = 1; bitPosition = 1;
	// Tx[3] = 0b00110010 
	// Tx[4] = 0b00000010
}

uint8_t readDiscreteInputs (void) //Case 0x02
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Inputs Address

	uint16_t numInputs = ((RxData[4]<<8)|RxData[5]);   // number to Inputs master has requested
	if (numInputs > 2000) {
		modbus_exception(ILLEGAL_DATA_VALUE);
		return 0;
	}
	
	if (startAddr + numInputs - 1 > InputSize*8) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	memset (TxData, '\0', 256);
	
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numInputs/8) + (numInputs%8>0 ? 1:0);  // Byte count
	int indx = 3;  
	
	int starByte = startAddr/8;
	uint16_t bitPosition = startAddr%8;
	int indxPosition = 0;

	// Load the actual data into TxData buffer
	for (int i=0; i<numInputs; i++)   
	{
		TxData[indx] |= ((Discrete_Inputs_Database[starByte] >> bitPosition) &0x01 ) << indxPosition;
		indxPosition++; bitPosition++;
		if (indxPosition > 7){
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7){
			bitPosition = 0;
			starByte++;
		}

	}
	
	if (numInputs%8 != 0) indx++;
	sendData(TxData, indx);  
	return 1; 
}

uint8_t readHoldingRegs (void) //Case 0x03
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address
	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);    // number to registers master has requested
	if (numRegs > 125) {
		modbus_exception(ILLEGAL_DATA_VALUE);
		return 0;
	}
	
	if (startAddr + numRegs - 1 > RegSize) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;   // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // byte count
	int indx = 3;  // number of bytes in TxData Buffer

	for (int i=0; i<numRegs; i++)   // Load data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr])&0xFF;     // extract the lower byte
		startAddr++;
	}

	sendData(TxData, indx); 
	return 1; 
}

uint8_t readInputRegs (void) //Case 0x04
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]); 

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]); 
	if (numRegs > 125) {
		modbus_exception(ILLEGAL_DATA_VALUE);
		return 0;
	}
	
	if (startAddr + numRegs - 1 > RegSize) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	int indx = 3;  // number of bytes in TxData Buffer

	for (int i=0; i<numRegs; i++)    // Load data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr])&0xFF;     // extract the lower byte
		startAddr++;
	}

	sendData(TxData, indx); 
	return 1; 
}

uint8_t writeSingleCoil (void) //Case 0x05
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Coil Address

	if (startAddr + 1 > CoilSize*8) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	int startByte = startAddr/8;
	uint16_t bitPosition = startAddr%8;
	
	if ((RxData[4] == 0xFF) && (RxData[5] == 0x00)) // FF 00 = ON
	{
		Coils_Database[startByte] |= 1<<bitPosition;
	}
	else if ((RxData[4] == 0x00) && (RxData[5] == 0x00)) // 00 00 = OFF
	{
		Coils_Database[startByte] &= ~(1<<bitPosition);
	}
	 
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | START ADDRESS | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE       | 2 BYTES   | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = RxData[2];  // start address HIGH byte
	TxData[3] = RxData[3];  // start address LOW byte
	TxData[4] = RxData[4];  // reg Data HIGH byte
	TxData[5] = RxData[5];  // reg Data LOW bye
	
	sendData(TxData, 6);
	return 1; 
}

uint8_t writeSingleReg (void) //Case 0x06
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	if (startAddr + 1 > RegSize) {
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	Holding_Registers_Database[startAddr]  = ((RxData[4]<<8)|RxData[5]);  
	
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | START ADDRESS | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE       | 2 BYTES   | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = RxData[2];  // start address HIGH byte
	TxData[3] = RxData[3];  // start address LOW byte
	TxData[4] = RxData[4];  // reg Data HIGH byte
	TxData[5] = RxData[5];  // reg Data LOW bye
	
	sendData(TxData, 6);
	return 1;
}

uint8_t writeMultiCoils (void) //Case 0x15
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numCoils = (RxData[4]<<8)|RxData[5];
	if (numCoils > 800){
		modbus_exception(ILLEGAL_DATA_VALUE);
	}
	
	if (startAddr + numCoils - 1 > CoilSize*8){
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	int startByte = startAddr/8;
	uint16_t bitPosition = startAddr%8;
	int indxPosition = 0;

	int indx = 7;
	
	// Load the actual data into TxData buffer
	for (int i=0; i<numCoils; i++)   
	{
		if (((RxData[indx]>>indxPosition)&0x01) == 1)
		{
			Coils_Database[startByte] |= 1<<bitPosition; // replace = 1
		}
		else 
		{
			Coils_Database[startByte] &= ~(1<<bitPosition); // replace = 0
		}			
		bitPosition++; indxPosition++;
		
		if (indxPosition>7)
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition>7)
		{
			bitPosition=0;
			startByte++;
		}
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | START ADDRESS | NUM OF REGS   | CRC       |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE       | 2 BYTES       | 2 BYTES |

	TxData[0] = SLAVE_ID;   // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = RxData[2];  // start address HIGH byte
	TxData[3] = RxData[3];  // start address LOW byte
	TxData[4] = RxData[4];  // reg Data HIGH byte
	TxData[5] = RxData[5];  // reg Data LOW bye
	
	sendData(TxData, 6);
	return 1;
}

uint8_t writeMultiRegs (void) //Case 0x16
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = (RxData[4]<<8)|RxData[5];
	if (numRegs > 100){
		modbus_exception(ILLEGAL_DATA_VALUE);
	}
	
	if (startAddr + numRegs - 1 > RegSize){
		modbus_exception(ILLEGAL_DATA_ADDRESS);
	}	
	
	int indx = 7; 
	for (int i=0; i<numRegs; i++)
	{
		Holding_Registers_Database[startAddr++]  = (RxData[indx]<<8|RxData[indx+1]);
		indx++;
	}
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | START ADDRESS | NUM OF REGS   | CRC       |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE       | 2 BYTES       | 2 BYTES |

	TxData[0] = SLAVE_ID;   // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = RxData[2];  // start address HIGH byte
	TxData[3] = RxData[3];  // start address LOW byte
	TxData[4] = RxData[4];  // reg Data HIGH byte
	TxData[5] = RxData[5];  // reg Data LOW bye
	
	sendData(TxData, 6);
	return 1;
}
