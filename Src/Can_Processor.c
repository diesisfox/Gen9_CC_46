/*
 * Can_Processor.c
 *
 *  Created on: Jan 8, 2017
 *      Author: frank
 */
#include "Can_Processor.h"

extern osMessageQId mainCanRxQHandle;
extern osMutexId * nodeEntryMtxHandle;
extern nodeEntry * nodeTable;
extern osTimerId * nodeTmrHandle;
extern const uint8_t selfNodeID;
extern osMessageQId BadNodesQHandle;
extern const uint32_t * const acceptedFirmware;

void Can_Processor(){
	uint8_t nodeConnectionWait[MAX_NODE_NUM] = {0};
	static Can_frame_t newFrame;
	for(;;){
		// Block until there is something on the mainCanRxQ
		xQueueReceive(mainCanRxQHandle, &newFrame, portMAX_DELAY);
		uint32_t canID = newFrame.id;	// canID container
		uint8_t RxNodeID = (canID & 0xF);
#ifdef __JAMES__
		static uint8_t rxmsg[] = "Got a frame.";
		Serial2_writeBuf(rxmsg);
#endif
		// CAN Processor state machine
		if((canID == p2pOffset) || (canID == selfNodeID + p2pOffset)){
			// Multicast or unicast command received!
			taskENTER_CRITICAL();
				executeCommand(newFrame.Data[0]);
			taskEXIT_CRITICAL();
		} else if (nodeTable[RxNodeID].nodeFirmwareVersion == SW_Sentinel){
			// Illegal node (not defined in the node Table)
			// Shutdown the node
			newFrame.id = RxNodeID + p2pOffset;
			newFrame.Data[0] = NODE_SHUTDOWN;
			newFrame.dlc = CMD_DLC;
			bxCan_sendFrame(&newFrame);
		} else if ((canID & 0xFF0) == swOffset){
			// Status word received
			xSemaphoreTake(nodeEntryMtxHandle[RxNodeID], portMAX_DELAY);
				bytesToReg(newFrame.Data,&(nodeTable[RxNodeID].nodeStatusWord));
			xSemaphoreGive(nodeEntryMtxHandle[RxNodeID]);

			if(nodeTable[RxNodeID].nodeConnectionState == CONNECTED){
				// Regular heartbeat; check the node status and refresh heartbeat when applicable
				if((nodeTable[RxNodeID].nodeStatusWord & SW_STATE_BITS) == ACTIVE){
					xTimerReset(nodeTmrHandle[RxNodeID], portMAX_DELAY);
				} else if ((nodeTable[RxNodeID].nodeStatusWord & SW_STATE_BITS) == SHUTDOWN){
					xTimerStop(nodeTmrHandle[RxNodeID], portMAX_DELAY);
				}
				// Remaining state is HARD error and no intervention is needed
			} else if (nodeTable[RxNodeID].nodeConnectionState == CONNECTING){
				// Node is in handshake process
				if((nodeTable[RxNodeID].nodeStatusWord & SW_STATE_BITS) == ACTIVE){
					// Promote node status when the handshake is completed
					xSemaphoreTake(nodeEntryMtxHandle[RxNodeID], portMAX_DELAY);
						nodeTable[RxNodeID].nodeConnectionState = CONNECTED;
					xSemaphoreGive(nodeEntryMtxHandle[RxNodeID]);
					// Refresh the heartbeat timer so the node doesn't timeout while attempting addition
					xTimerReset(nodeTmrHandle[RxNodeID], portMAX_DELAY);
				} else {
					// Handle the situation where the node is stuck in the CONNECTING state
					(nodeConnectionWait[RxNodeID])++;
					if(nodeConnectionWait[RxNodeID] >= MAX_CONN_WAIT_ATTEMPTS){
						// Node is stuck in CONNECTING state -> shutdown the node
						newFrame.dlc = CMD_DLC;
						newFrame.id = RxNodeID + p2pOffset;
						newFrame.Data[0] = NODE_SHUTDOWN;
						bxCan_sendFrame(&newFrame);

						xSemaphoreTake(nodeEntryMtxHandle[RxNodeID], portMAX_DELAY);
							nodeTable[RxNodeID].nodeConnectionState = DISCONNECTED;
						xSemaphoreGive(nodeEntryMtxHandle[RxNodeID]);
						xTimerStop(nodeTmrHandle[RxNodeID], portMAX_DELAY);
					} else {
						xTimerReset(nodeTmrHandle[RxNodeID], portMAX_DELAY);
					}
				}

			} else {
				// Node behavior is improper; send to NodeManager to reset
				xSemaphoreTake(nodeEntryMtxHandle[RxNodeID], portMAX_DELAY);
					nodeTable[RxNodeID].nodeConnectionState = UNRELIABLE;
				xSemaphoreGive(nodeEntryMtxHandle[RxNodeID]);
				xQueueSend(BadNodesQHandle, &RxNodeID, portMAX_DELAY);
			}
		} else if ((canID & 0xFF0) == fwOffset){
			// Firmware string received
			uint32_t tempFWString;
			bytesToReg(newFrame.Data, &tempFWString);
			newFrame.dlc = CMD_DLC;
			newFrame.id = RxNodeID + p2pOffset;
			if(tempFWString == acceptedFirmware[RxNodeID]){
				// Firmware string matches preset values
				// Send CC_NACK to the node
				newFrame.Data[0] = CC_ACK;
				// Update nodeTable entry
				xSemaphoreTake(nodeEntryMtxHandle[RxNodeID], portMAX_DELAY);
					nodeTable[RxNodeID].nodeConnectionState = CONNECTING;
				xSemaphoreGive(nodeEntryMtxHandle[RxNodeID]);
					//XXX the line below uniquely crashes
				xTimerReset(nodeTmrHandle[RxNodeID], portMAX_DELAY);
				xTimerReset(nodeTmrHandle[RxNodeID], portMAX_DELAY);	// Start the node heartbeat timer
			}
			else {
				// Firmware string mismatch - Wrong firmware loaded into system
				// Send CC_NACK to the node
				newFrame.Data[0] = CC_NACK;
				// Update nodeTable entry
				xSemaphoreTake(nodeEntryMtxHandle[RxNodeID], portMAX_DELAY);
					nodeTable[RxNodeID].nodeConnectionState = DISCONNECTED;
				xSemaphoreGive(nodeEntryMtxHandle[RxNodeID]);
			}
			bxCan_sendFrame(&newFrame);
		}
		// Ignore other cases
	}
}
