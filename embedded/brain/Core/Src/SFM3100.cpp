/*
 * SFM3100.cpp
 *
 *  Created on: Mar 26, 2024
 *      Author: Bronco
 */

#include <SFM3100.h>

SFM3100::SFM3100(ADC_HandleTypeDef *hadc)
{
	hadc_ptr = hadc;
	A = 8000;
	B = 100;
	Offset = 67;
}

float SFM3100::read_voltage(){
	uint16_t ADC_value;
	float voltage;
	HAL_ADC_Start(hadc_ptr);
	HAL_ADC_PollForConversion(hadc_ptr, 1);
	ADC_value = HAL_ADC_GetValue(hadc_ptr);

	voltage = 3.3*float(ADC_value)/4096;
	return voltage;
}

float SFM3100::read_flowRate(){
	float voltage, flowRate;
	voltage = this -> read_voltage();
	flowRate = (voltage - 0.67)*80*abs(voltage - 0.67);
	//flowRate = (voltage - (float(Offset)/B))*(A/B)*abs(voltage - (float(Offset)/B));
	return flowRate;
}
