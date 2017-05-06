/*
 * Node_Manager.c
 *
 *  Created on: Jan 8, 2017
 *      Author: frank
 */

#include "Node_Manager.h"
#include "cmsis_os.h"
#include "../../CAN_ID.h"
#include "nodeConf.h"
#include "can.h"

extern osMessageQId BadNodesQHandle;
extern nodeEntry * nodeTable;

void Node_Manager(){
	uint8_t badNodeID = 0;
	xQueueReceive(BadNodesQHandle, &badNodeID, portMAX_DELAY);	// Wait for queue
	UBaseType_t currentPriority = uxTaskPriorityGet(NULL);		// Get priority of current Task
	// TODO: Verify the below statement
	xTaskCreate(resetNode, NULL, 512, (void*)(badNodeID), currentPriority, NULL);
//	xTaskCreate(resetNode, NULL, 512, (void*)(&badNodeID), currentPriority, NULL);		// Create a new task to deal with each bad node

}

// Wrapper for the recursive reset node function
void resetNode(void * nodeID){
	uint8_t _nodeID = (uint8_t)(nodeID);
//	uint8_t _nodeID = *((uint8_t*)(nodeID));
//	osDelay(Node_HB_Interval);
	resetNode_recursive(_nodeID, 0);
}

void resetNode_recursive(uint8_t nodeID, uint8_t attempt){
	if(/*attempt >= 0*/attempt <= MAX_NODE_RESET_ATTEMPTS){
		Can_frame_t newFrame;
		newFrame.dlc = 1;
		newFrame.id = 0x040 + nodeID;
		newFrame.isExt = 0;
		newFrame.isRemote = 0;
		newFrame.Data[0] = (attempt<MAX_NODE_RESET_ATTEMPTS)?NODE_RESET:NODE_HRESET;
		bxCan_sendFrame(&newFrame);
		osDelay(2 * HB_Interval);
		if(nodeTable[nodeID].nodeConnectionState == UNRELIABLE){
			// Continue to attempt node restart
			resetNode_recursive(nodeID, attempt + 1);
		}
		else {
			// Node recovery successful
			vTaskDelete(NULL);	// Clean up the task
		}
	}else{
		// Restart failed; give up...
				nodeTable[nodeID].nodeConnectionState = CONN_ERROR;
				vTaskDelete(NULL);	// Clean up
	}
	// Since attempt is monotonically increasing, no other cases need to be handled!
}
