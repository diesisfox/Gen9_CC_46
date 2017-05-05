/*
 * RT_Handler.h
 *
 *  Created on: Jan 8, 2017
 *      Author: frank
 */

#ifndef RT_HANDLER_H_
#define RT_HANDLER_H_

#include "stm32l4xx_hal.h"

void RT_Handler(uint32_t * PreviousWakeTime, uint8_t * motCanErrCount);

#endif /* RT_HANDLER_H_ */
