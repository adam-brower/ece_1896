/*
 * SFM3100.h
 *
 *  Created on: Mar 26, 2024
 *      Author: Bronco
 */

#ifndef INC_SFM3100_H_
#define INC_SFM3100_H_

#include "main.h"

class SFM3100
{
  public:
    // constructor
	SFM3100(ADC_HandleTypeDef *hadc);

	float read_voltage();
	float read_flowRate();

	float flow;
  private:
	ADC_HandleTypeDef * hadc_ptr;
	uint16_t A, B, Offset;
};


#endif /* INC_SFM3100_H_ */
