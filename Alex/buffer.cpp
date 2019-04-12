/*
 * buffer.c
 * 
 * This file implements a circular buffer
 *
 * Created: 24/2/2018 4:02:12 PM
 *  Author: dcstanc
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "buffer.h"

#ifndef NULL
#define NULL	(void *) 0x0
#endif

// Disable interrupts
void enterAtomic(unsigned char *csreg)
{
	*csreg = SREG;
	cli();
}

// Re-enable interrupts
void exitAtomic(unsigned char csreg)
{
	SREG = csreg;
}

// Initialize the buffer. We must call this before using writeBuffer or readBuffer.
// PRE:
//		Buffer to use is specified in "buffer", size of buffer in characters is specified in "size"
// POST:
//		Buffer is initialized to accept up to "size" characters.
void initBuffer(TBuffer *buffer, unsigned int size)
{
	buffer->buffer = (unsigned char *) calloc(size, sizeof(unsigned char));
	buffer->count=0;
	buffer->front=0;
	buffer->back=0;
	buffer->size=size;
}

// Write to the buffer.
// PRE:
//		Buffer specified in "buffer" argument must be initialized using initBuffer. "data" is character to
//		write to buffer.
// POST:
//		"data" is written to "buffer" and writeBuffer returns BUFFER_OK if space is available.
//		"data" is discarded and writeBuffer returns BUFFER_FULL if space is not available.
//		"data" is discarded and writeBuffer returns BUFFER_INVALID if "buffer" was not initialized using initBuffer.

TBufferResult writeBuffer(TBuffer *buffer, unsigned char data)
{
	enterAtomic(&buffer->csreg);
	if(buffer->buffer == NULL || buffer->size == 0)
	{
		exitAtomic(buffer->csreg);
		return BUFFER_INVALID;
	}
	
	if(buffer->count >= buffer->size)
	{
		exitAtomic(buffer->csreg);
		return BUFFER_FULL;
	}
	
	buffer->buffer[buffer->back] = data;
	buffer->back = (buffer->back + 1) % buffer->size;
	buffer->count++;
	
	exitAtomic(buffer->csreg);
	return BUFFER_OK;
}

// Read from the buffer.
// PRE:
//		Buffer specified in "buffer" argument must be initialized using initBuffer. "data" is a pointer
//		to a variable of type unsigned char.
// POST:
//		Variable pointed to by "data" contains character read from head of the queue, and readBuffer returns
//		BUFFER_OK if data is available for reading.
//		Variable pointed to by "data" is unmodified, and readBuffer returns BUFFER_EMTPY if no data is available.
//		Variable pointed to by "data" is unmodified, and readBuffer returns BUFFER_INVALID if "buffer" was
//		not initialized using initBuffer.

TBufferResult readBuffer(TBuffer *buffer, unsigned char *data)
{
	enterAtomic(&buffer->csreg);
	
	if(buffer->buffer == NULL || buffer->size == 0)
	{
		exitAtomic(buffer->csreg);
		return BUFFER_INVALID;
	}
	
	if(buffer->count == 0)
	{
		exitAtomic(buffer->csreg);
		return BUFFER_EMPTY;
	}
	
	*data = buffer->buffer[buffer->front];
	buffer->front = (buffer->front + 1) % buffer->size;
	buffer->count--;
	
	exitAtomic(buffer->csreg);
	return BUFFER_OK;
}


// Frees buffer
// PRE:
//		"buffer" points to buffer to free
// POST:
//		"buffer" is deallocated.

void freeBuffer(TBuffer *buffer)
{
	if(buffer->buffer != NULL)
	{
		free(buffer->buffer);
		buffer->buffer = NULL;
	}
	
	buffer->size=0;
	buffer->count=0;
}

int dataAvailable(TBuffer *buffer)
{
	return (buffer->count > 0);
}
