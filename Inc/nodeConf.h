/*
 * nodeConf.h
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 *  Edit this file for all node configurations!
 */

/*
 * Node configuration instructions:
 * 1. Make sure all parameters in this files is set properly according to node specifications
 * 2. Implement additional application-layer parsing in do
 * 		- main.c (loc "XXX 1")
 * 3. Implement flush queues in executeCommand()
 * 		- nodeMiscHelpers.c (loc "XXX 2" and loc "XXX 3")
 * 4. Suspend any application layer tasks in shutdown command
 * 		- nodeMiscHelpers.c (loc "XXX 4")
 *
 *
 * If you have any additional tasks/queues/mutexes/semaphores, you should try to add them through CubeMX if possible.
 * As always, ensure that there is sufficient FLASH and HEAP!
 *
 * The task priorities should ALWAYS be set as the following:
 * PRIORITY |	TASK
 * 	HIGH	  watchdogRefresh (if applicable)
 * 	 ^	 	  FreeRTOS_Timer
 *   |		  Can_Processor
 *   |---------------------------------------
 *   |		  Application Layer	Tasks		|
 *   |---------------------------------------
 *  LOW		  IdleTask
 *
 *	Deviation from this priority list may result in unstable node behavior and/or timing failure!
 */

#ifndef NODECONF_H_
#define NODECONF_H_

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define WD_Interval			16			// Watdog timer refresh interval (soft ms) | MUST BE LESS THAN 26!!!
#define RT_Interval			10			// Refresh interval for driver controls
#define Node_HB_Interval	2*HB_Interval			// Node's maximum heartbeat interval time

#define MAX_NODE_NUM	 		16		// Maximum number of nodes supported on this system
#define MAX_NODERESET_ATTEMPTS	1		// Maximum number of retries CC will attempt before flagging node as HARD ERRORs

//const uint32_t const * acceptedFirmware;

#ifdef FRANK
const uint32_t firmwareString = 0x00000100;	// v00.00.01.0
const uint8_t selfNodeID = 1;	// CC
#define NODE_CONFIGURED
#elif defined (__JAMES__)
static const uint32_t firmwareString = 0x00000100;
static const uint8_t selfNodeID = 6;
uint32_t selfStatusWord = 0x0;
#define NODE_CONFIGURED
#else
// SW_Sentinel will fail the CC firmware check and result in node addition failure!
static const uint32_t firmwareString = 0x00000100;			// Firmware Version string
static const uint8_t selfNodeID = 1;					// The nodeID of this node
extern uint32_t selfStatusWord;
#define NODE_CONFIGURED
#endif

// The node entry does not need a nodeID since it will be stored in an array with nodeID as key
typedef struct {
	uint8_t	 nodeConnectionState;
	uint32_t nodeFirmwareVersion;
	uint32_t nodeStatusWord;
} nodeEntry;

typedef struct {
	uint16_t switchPositions;
	float	 brakePosition;
	float	 accelPosition;
	float	 regenPosition;
} controlVars;

typedef struct{
	uint8_t  nodeID;
	uint8_t	 attempts;
	uint32_t ticks;
} resetParams;

#endif /* NODECONF_H_ */
