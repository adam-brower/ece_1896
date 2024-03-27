/*
 * sensors.cpp
 *
 *  Created on: Mar 22, 2024
 *      Author: Bronco
 */

#include <PTE7300_I2C.h>

#define MAXIMUM_TRIES 100

// default nodeaddress
#define DEFAULT_NODE_ADDRESS 0x6C

// command register address
#define RAM_ADDR_CMD 0x22
// serial register
#define RAM_ADDR_SERIAL 0x50
// result registers
#define RAM_ADDR_DSP_T 0x2E
#define RAM_ADDR_DSP_S 0x30
#define RAM_ADDR_STATUS	0x36
#define RAM_ADDR_ADC_TC 0x26


PTE7300_I2C::PTE7300_I2C(I2C_HandleTypeDef *hi2c)
{
  _nodeAddress = DEFAULT_NODE_ADDRESS;
  _bUseCRC = false; // CRC flag
  hi2c_pnt = hi2c;
  // This requires I2C to be initialized before this is constructed
}

void PTE7300_I2C::CRC_set(bool tf) {_bUseCRC = tf;}

void PTE7300_I2C::start()
{
	uint16_t CMD = 0x8B93; // START command
	this->writeRegister(RAM_ADDR_CMD, 1, &CMD); // write to CMD register
}

void PTE7300_I2C::sleep()
{
	uint16_t CMD = 0x6C32; // SLEEP command
	this->writeRegister(RAM_ADDR_CMD, 1, &CMD); // write to CMD register
}

void PTE7300_I2C::idle()
{
	uint16_t CMD = 0x7BBA; // IDLE command
	this->writeRegister(RAM_ADDR_CMD, 1, &CMD); // write to CMD register
}

void PTE7300_I2C::reset()
{
	uint16_t CMD = 0xB169; // RESET command
	this->writeRegister(RAM_ADDR_CMD, 1, &CMD); // write to CMD register
}

uint8_t PTE7300_I2C::writeRegister(uint8_t address, unsigned int number, uint16_t* data)
{
	uint8_t status;
	status = this->writeRegisterNoCRC(address, number, data);
	return status;
}

uint8_t PTE7300_I2C::writeRegisterNoCRC(uint8_t address, unsigned int number, uint16_t* data)
{
	uint8_t status;
	uint8_t data_byte[2*number];

	for (int i = 0; i < number; i++)
	{
		data_byte[i] = data[i] & 0x00FF;
		data_byte[i+1] = (data[i] & 0xFF00) >> 8;
	}

	status = HAL_I2C_Mem_Write(hi2c_pnt, (_nodeAddress << 1), address, I2C_MEMADD_SIZE_8BIT, data_byte, 2*number, HAL_MAX_DELAY);
	return status;
}

uint8_t PTE7300_I2C::readRegister(uint8_t address, unsigned int number, uint16_t *buffer)
{
	uint8_t status;
	status = this->readRegisterNoCRC(address, number, buffer);
	return status;
}

uint8_t PTE7300_I2C::readRegisterNoCRC(uint8_t address, unsigned int number, uint16_t *buffer)
{
	uint8_t status;
	uint8_t data_byte[2*number];

	status = HAL_I2C_Mem_Read(hi2c_pnt, (_nodeAddress << 1), address, I2C_MEMADD_SIZE_8BIT, data_byte, 2*number, HAL_MAX_DELAY);

	for (int i = 0; i < number; i++)
	{
		uint16_t lowByte = data_byte[i];
		uint16_t highByte = data_byte[i+1];
		buffer[i] = highByte << 8 | lowByte;
	}
	return status;
}

int16_t PTE7300_I2C::readDSP_S()
{
  uint16_t DSP_S;
  this->readRegister(RAM_ADDR_DSP_S, 1, &DSP_S);
  return (int16_t)(DSP_S); // type-cast to signed integer
}

int PTE7300_I2C::readPressure()
{
	int16_t DSP_reading;
	int pressure_psi;
	DSP_reading = this -> readDSP_S();
	pressure_psi =  5800*float((DSP_reading + 16000))/32768;
	return pressure_psi;
}



