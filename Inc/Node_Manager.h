/*
 * Node_Manager.h
 *
 *  Created on: Jan 8, 2017
 *      Author: frank
 */

#ifndef NODE_MANAGER_H_
#define NODE_MANAGER_H_

#include "stm32f4xx_hal.h"

void Node_Manager(void);
void resetNode(void * nodeID);
void resetNode_recursive(uint8_t nodeID, uint8_t attempt);


#endif /* NODE_MANAGER_H_ */
