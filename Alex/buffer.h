/*
 * buffer.h
 *
 * Created: 24/2/2018 4:03:06 PM
 *  Author: dcstanc
 */ 


#ifndef BUFFER_H_
#define BUFFER_H_

typedef struct tb
{
	// The actual buffer
	unsigned char *buffer;
	
	// Front and back pointers
	unsigned int front, back;
	
	// Our buffer size
	unsigned int size;
	
	// Number of characters in buffer
	unsigned int count;
	
	// This is for atomicity control
	unsigned char csreg;
	
} TBuffer;

typedef enum 
{
	BUFFER_OK,
	BUFFER_FULL,
	BUFFER_EMPTY,
	BUFFER_INVALID
} TBufferResult;

// Intialize the buffer. We must call this before using writeBuffer or readBuffer.
// PRE: 
//		Buffer to use is specified in "buffer", size of buffer in characters is specified in "size"
// POST:
//		Buffer is initialized to accept up to "size" characters.
void initBuffer(TBuffer *buffer, unsigned int size);

// Write to the buffer.
// PRE:
//		Buffer specified in "buffer" argument must be initialized using initBuffer. "data" is character to
//		write to buffer.
// POST:
//		"data" is written to "buffer" and writeBuffer returns BUFFER_OK if space is available.
//		"data" is discarded and writeBuffer returns BUFFER_FULL if space is not available.
//		"data" is discarded and writeBuffer returns BUFFER_INVALID if "buffer" was not initialized using initBuffer.

TBufferResult writeBuffer(TBuffer *buffer, unsigned char data);

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

TBufferResult readBuffer(TBuffer *buffer, unsigned char *data);

// Frees buffer
// PRE:
//		"buffer" points to buffer to free
// POST:
//		"buffer" is deallocated.
		
void freeBuffer(TBuffer *buffer);

int dataAvailable(TBuffer *buffer);
#endif /* BUFFER_H_ */
