/*
 * Queue.h
 *
 *  Created on: Mar 23, 2020
 *      Author: 79029
 */

#ifndef SRC_QUEUE_H_
#define SRC_QUEUE_H_
#include "cmsis_os.h"

template<class T>
class osQueue
{


public:
	osQueue(uint16_t count,const char *name):NameQ(name),queueSize(count)
{
		//xQueueBuffer = new uint8_t[queueSize * sizeof( T )];
}

	void createQueue()
	{
		const osMessageQueueAttr_t attributes = {
				name : NameQ,
				attr_bits : NULL,
				cb_mem : NULL,//,/&xQueueControlBlock,
				cb_size :NULL,//sizeof(xQueueControlBlock),
				mq_mem :NULL,// &xQueueBuffer,
				mq_size :NULL// sizeof(xQueueBuffer)
		};
		xHandle=osMessageQueueNew (queueSize, sizeof(T), &attributes);
	}
	osStatus_t receive(T * val, TickType_t xTicksToWait = osWaitForever)
	{
		return osMessageQueueGet(xHandle, val, NULL, xTicksToWait);
	}

	osStatus_t send(const T & val, TickType_t xTicksToWait = 0U)
	{
		return osMessageQueuePut(xHandle, &val, NULL, xTicksToWait);
	}

	osMessageQueueId_t xHandle;
		const uint16_t queueSize;
		//uint8_t *xQueueBuffer;
		//StaticQueue_t xQueueControlBlock;
		const char *NameQ;
};
// Definitions for defaultTask





class osTask
{
	osThreadId_t TaskHandle;
	//uint32_t TaskBuffer[ size ];
	//StaticTask_t TaskControlBlock;
	osPriority_t priority;
	const char *NameQ;
	const  uint16_t stack_size;
public:
	osTask(const char *name,uint16_t stack,osPriority_t osPriority=osPriorityNormal ):NameQ(name),stack_size(stack)
{
		priority=osPriority;
}


	void start(osThreadFunc_t func)
	{
		const osThreadAttr_t Task_attributes = {
				name:NameQ,
				attr_bits:NULL,//
				cb_mem :NULL,// &TaskControlBlock,
				cb_size :NULL,// sizeof(TaskControlBlock),
				stack_mem :NULL,// &TaskBuffer[0],
				stack_size: stack_size*4,
				priority : (osPriority_t) priority,
				tz_module : NULL,
				reserved:NULL
		};

		TaskHandle = osThreadNew(func, NULL, &Task_attributes);
	}

};


#endif /* SRC_QUEUE_H_ */

