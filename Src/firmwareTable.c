/*
 * firmwareTable.c
 *
 *  Created on: Jan 10, 2017
 *      Author: frank
 */


#include  "../../CAN_ID.h"
#include  "nodeConf.h"

const uint32_t * const acceptedFirmware = (uint32_t[MAX_NODE_NUM]){
		0x00000101,		// nodeID = 0; 	System Architecture
		0x00000001,		// nodeID = 1;	Command Center
		0x00000001,		// nodeID = 2;	Motor Controller
		0x00000001,		// nodeID = 3;	BPS
		0x00000001,		// nodeID = 4;	ADS
		0x00000001,		// nodeID = 5;  DCI
		0x00000001,		// nodeID = 6;	Radio
		0x00000000,		// nodeID = 7;  N/A
		0x00000000,		// nodeID = 8;  N/A
		0x00000000,		// nodeID = 9;  N/A
		0x00000001,		// nodeID = A;	Inertial Navigation unit
		0x00000001,		// nodeID = B;	Pyronometer
		0x00000000,		// nodeID = C;  N/A
		0x00000000,		// nodeID = D;  N/A
		0x00000000,		// nodeID = E;  N/A
		0x00000000,		// nodeID = F;  N/A
};
