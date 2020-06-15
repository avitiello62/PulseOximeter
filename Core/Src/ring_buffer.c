#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer *txBuf) {
	txBuf->head = 0;
	txBuf->tail = 0;
	txBuf->status = RING_BUFFER_OK;
	txBuf->size = 0;
}

Status addByte(RingBuffer *txBuf, uint8_t pData) {

	txBuf->buffer[txBuf->tail] = pData;
	txBuf->tail = (txBuf->tail + 1) % MAX_SIZE_RING_BUFFER;
	txBuf->size++;

	if (txBuf->size == MAX_SIZE_RING_BUFFER)
		txBuf->status = RING_BUFFER_OK;

	return txBuf->status;
}

Status getByte(RingBuffer *txBuf, uint8_t *pData) {

	*pData = txBuf->buffer[txBuf->head];
	txBuf->head = (txBuf->head + 1) % MAX_SIZE_RING_BUFFER;
	txBuf->size--;
	if (txBuf->size == -1)
		txBuf->status = RING_BUFFER_NOT_OK;
	return txBuf->status;
}

Status RingBuffer_Write(RingBuffer *txBuf, uint8_t *pData, uint8_t len) {

	for (int i = 0; i < len; i++) {
		if (addByte(txBuf, pData[i]) == RING_BUFFER_NOT_OK)
			return RING_BUFFER_NOT_OK;
	}

	return RING_BUFFER_OK;

}

Status RingBuffer_Read(RingBuffer *txBuf, uint8_t *pData, uint8_t len) {
	if (len > 1) {
		for (int i = 0; i < len; i++) {
			if (getByte(txBuf, &pData[i]) == RING_BUFFER_NOT_OK)
				return RING_BUFFER_NOT_OK;
		}
	} else
		return getByte(txBuf, pData);

	return RING_BUFFER_OK;
}

uint8_t RingBuffer_GetDataLength(RingBuffer *txBuf) {
	return txBuf->size;
}

