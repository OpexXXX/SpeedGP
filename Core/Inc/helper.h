/*
 * Queue.h
 *
 *  Created on: Mar 23, 2020
 *      Author: 79029
 */

#ifndef SRC_QUEUE_H_
#define SRC_QUEUE_H_
#include "cmsis_os.h"

template<class T, size_t size>
class osQueue
{
	osMessageQueueId_t xHandle;
	uint8_t xQueueBuffer[size * sizeof( T ) ];
	StaticQueue_t xQueueControlBlock;
	const char *NameQ;

public:
	osQueue(const char *name):NameQ(name)
{

}

	void createQueue()
	{
		const osMessageQueueAttr_t attributes = {
				name : NameQ,
				attr_bits : NULL,
				cb_mem : &xQueueControlBlock,
				cb_size :sizeof(xQueueControlBlock),
				mq_mem : &xQueueBuffer,
				mq_size : sizeof(xQueueBuffer)
		};
		xHandle=osMessageQueueNew (size, sizeof(T), &attributes);
	}
	osStatus_t receive(T * val, TickType_t xTicksToWait = osWaitForever)
	{
		return osMessageQueueGet(xHandle, val, NULL, xTicksToWait);
	}

	osStatus_t send(const T & val, TickType_t xTicksToWait = 0U)
	{
		return osMessageQueuePut(xHandle, &val, NULL, xTicksToWait);
	}
};
// Definitions for defaultTask


template<size_t size>
class osTask
{
	osThreadId_t TaskHandle;
	uint32_t TaskBuffer[ size ];
	StaticTask_t TaskControlBlock;
	osPriority_t priority;
	const char *NameQ;

public:
	osTask(const char *name,osPriority_t osPriority=osPriorityNormal ):NameQ(name)
{
		priority=osPriority;
}


	void start(osThreadFunc_t func)
	{
		const osThreadAttr_t Task_attributes = {
				name:NameQ,
				attr_bits:NULL,
				cb_mem : &TaskControlBlock,
				cb_size : sizeof(TaskControlBlock),
				stack_mem : &TaskBuffer[0],
				stack_size: sizeof(TaskBuffer),
				priority : (osPriority_t) priority,
				tz_module : NULL,
				reserved:NULL
		};

		TaskHandle = osThreadNew(func, NULL, &Task_attributes);
	}

};


#endif /* SRC_QUEUE_H_ */

