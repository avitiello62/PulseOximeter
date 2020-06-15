/*
 * ring_buffer.h
 *
 *  Created on: May 8, 2020
 *      Author: mario
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_
#include "main.h"
#define MAX_SIZE (60)


enum Status{RING_BUFFER_OK, RING_BUFFER_NOT_OK};
typedef enum Status Status;

typedef struct _RingBuffer
{
	uint8_t buffer[MAX_SIZE];
	uint8_t head,tail;
	Status status;
	uint8_t size;
}RingBuffer;


void RingBuffer_Init(RingBuffer *txBuf);
Status RingBuffer_Write(RingBuffer *txBuf, uint8_t *pData, uint8_t len);
Status RingBuffer_Read(RingBuffer *txBuf, uint8_t *pData, uint8_t len);
uint8_t RingBuffer_GetDataLength(RingBuffer *txBuf);


#endif /* INC_RING_BUFFER_H_ */




