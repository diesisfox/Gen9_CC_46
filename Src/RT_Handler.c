/*
 * RT_Handler.c
 *
 *  Created on: Jan 8, 2017
 *      Author: frank
 */

#ifndef DISABLE_RT

//#include "RT_Handler.h"
#include "nodeMiscHelpers.h"
#include "nodeConf.h"
#include "cmsis_os.h"
#include "can.h"
//#include "mcpCAN.h"
#include "../../CAN_ID.h"

extern nodeEntry * nodeTable;
extern controlVars userInput;
extern osMutexId controlVarsMtxHandle;
extern osMutexId * nodeEntryMtxHandle;
extern osMessageQId motCanRxQHandle;
extern const uint32_t * const acceptedFirmware;

inline void RT_Handler(uint32_t * PreviousWakeTime, uint8_t * motCanErrCount){
Can_frame_t tempFrame;
Can_frame_t newFrame;	// To be sent on the mainCAN
newFrame.isExt = 0;
newFrame.isRemote = 0;
newFrame.dlc = CMD_DLC;
newFrame.id = mc_P2P;

for(;;){
	// Motor controller connection state machine
	if((nodeTable[mc_nodeID].nodeConnectionState == CONNECTED) || (nodeTable[mc_nodeID].nodeConnectionState == UNRELIABLE)){
		if(nodeTable[mc_nodeID].nodeConnectionState == CONNECTED) {
			// Only send the diagnostic frame request when the status is connected
			tempFrame.isExt = 1;
			tempFrame.isRemote = 0;
			tempFrame.dlc = Req_DLC;
			tempFrame.id = mitsubaREQ;
			tempFrame.Data[0] = Req_Frm0 | Req_Frm1 | Req_Frm2;
			mcpCan_sendFrame(&tempFrame);	// Send to motorCAN
		}

		// Take the CONTROL_VARS semaphore; there should be no competition
		xSemaphoreTake(controlVarsMtxHandle,0);	// Fail if false;
		// TODO: ADC read control vars and write to userInput struct
		xSemaphoreGive(controlVarsMtxHandle);

		// Send switch positions
		newFrame.id = swPos;
		newFrame.dlc = swPos_DLC;
		bxCan_sendFrame(&newFrame);		// Send to main CAN for logging
		mcpCan_sendFrame(&newFrame);	// Send to motor CAN for control

		// Send brake position
		newFrame.id = brakePos;
		newFrame.dlc = brakePos_DLC;
		bxCan_sendFrame(&newFrame);
		mcpCan_sendFrame(&newFrame);

		// Send accelerator position
		newFrame.id = accelPos;
		newFrame.dlc = accelPos_DLC;
		bxCan_sendFrame(&newFrame);
		mcpCan_sendFrame(&newFrame);

		// Send regen position
		newFrame.id = regenPos;
		newFrame.dlc = regenPos_DLC;
		bxCan_sendFrame(&newFrame);
		mcpCan_sendFrame(&newFrame);

		// Wait for the incoming diagnostic data; but timeout to ensure real time operation
		if(!xQueueReceive(motCanRxQHandle, &tempFrame, MC_Refresh_Interval)){
			(*motCanErrCount)++;
			if (*motCanErrCount > MC_MAX_ERRNUM){
				nodeTable[mc_nodeID].nodeConnectionState = CONN_ERROR;
				continue;
			} else {
				nodeTable[mc_nodeID].nodeConnectionState = UNRELIABLE;
			}
		}
		else {
			tempFrame.isExt = 0;
			nodeTable[mc_nodeID].nodeConnectionState = CONNECTED;
			if (tempFrame.id == mitsubaFr0) {
				// Diagnostic frame 0
				// TODO: Check frame 0 data
				tempFrame.id = mcDiag0;	// Remap the the frame into the mainCAN addresses
				bxCan_sendFrame(&tempFrame);
			} else if (tempFrame.id == mitsubaFr1) {
				// Diagnostic frame 1
				// TODO: Check frame 1 data
				tempFrame.id = mcDiag1;	// Remap the the frame into the mainCAN addresses
				bxCan_sendFrame(&tempFrame);
			} else if (tempFrame.id == mitsubaFr2) {
				// Diagnostic frame 2
				// TODO: Check frame 2 data
				tempFrame.id = mcDiag1;	// Remap the the frame into the mainCAN addresses
				bxCan_sendFrame(&tempFrame);
			}
		}
	}
	else if(nodeTable[mc_nodeID].nodeConnectionState == DISCONNECTED){
		while (!xQueueReceive(motCanRxQHandle, &tempFrame, portMAX_DELAY));	// Keep waiting until the first handshake received
		// Verify handshake
		if(tempFrame.id == mc_FW){
			uint32_t tempFWString;
			bytesToReg(tempFrame.Data, &tempFWString);
			if(tempFWString == acceptedFirmware[mc_nodeID]){
				newFrame.Data[0] = CC_ACK;		// Acknowledge node addition request
			}
			else {
				newFrame.Data[0] = CC_NACK;
			}
		} else {
			newFrame.Data[0] = NODE_RESET;
		}
		mcpCan_sendFrame(&newFrame);
		continue;
	}
	else if(nodeTable[mc_nodeID].nodeConnectionState == CONNECTING){
		if(xQueueReceive(motCanRxQHandle, &tempFrame, MC_RES_TIMEOUT))
		{
			if(tempFrame.id == mc_SW){
				// Update node status word
				xSemaphoreTake(nodeEntryMtxHandle[mc_nodeID], 0);
					bytesToReg(tempFrame.Data, &(nodeTable[mc_nodeID].nodeStatusWord));
				xSemaphoreGive(nodeEntryMtxHandle[mc_nodeID]);

				if((nodeTable[mc_nodeID].nodeStatusWord & SW_STATE_BITS) == ACTIVE){
					// Node verified to be in active state
					nodeTable[mc_nodeID].nodeConnectionState = CONNECTED;
				}
			}
			else {
				// Node is in error condition, reset the controller
				nodeTable[mc_nodeID].nodeConnectionState = DISCONNECTED;
				newFrame.Data[0] = NODE_RESET;
				mcpCan_sendFrame(&newFrame);
			}
		}
		else {
			nodeTable[mc_nodeID].nodeConnectionState = DISCONNECTED;
		}
	}

	// All cases
	osDelayUntil(PreviousWakeTime, MC_Refresh_Interval);
}
}

#endif

