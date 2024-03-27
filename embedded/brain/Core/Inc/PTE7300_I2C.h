/*
 * sensors.h
 *
 *  Created on: Mar 22, 2024
 *      Author: Bronco
 */


#include "main.h"

#ifndef INC_PTE7300_I2C_H_
#define INC_PTE7300_I2C_H_

class PTE7300_I2C
{
  public:
    // constructor
	PTE7300_I2C(I2C_HandleTypeDef *hi2c);

	// low-level functions
	bool		  isConnected(); // check connectivity of device to the I2C-bus
	void		  CRC_set(bool tf);	// use CRC checking on datatransmission on/off (default true)

	// high-level functions
	uint32_t      readSERIAL();
	int16_t       readDSP_T();
	int16_t       readDSP_S();
	uint16_t	  readSTATUS();
	int           readADC_TC();
	int 		  readPressure();
	void		  start();
	void 	      sleep();
	void 		  idle();
	void 		  reset();

  private:
    // class properties
	int _nodeAddress;
	bool _bUseCRC;
	I2C_HandleTypeDef * hi2c_pnt;

	// static functions
	uint8_t  readRegister(uint8_t address, unsigned int number, uint16_t *buffer);
	uint8_t  readRegisterNoCRC(uint8_t address, unsigned int number, uint16_t *buffer);
	uint8_t  readRegisterCRC(uint8_t address, unsigned int number, uint16_t *buffer);
	uint8_t          writeRegisterNoCRC(uint8_t address, unsigned int number, uint16_t *data);
	uint8_t 		  writeRegisterCRC(uint8_t address, unsigned int number, uint16_t *data);
	uint8_t          writeRegister(uint8_t address, unsigned int number, uint16_t *data);
	static char   calc_crc4(unsigned char polynom, unsigned char init, unsigned char* data, unsigned int len);
	static char   calc_crc8(unsigned char polynom, unsigned char init, unsigned char* data, unsigned int len);
};



#endif /* INC_PTE7300_I2C_H_ */
