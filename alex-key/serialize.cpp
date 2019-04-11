#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "serialize.h"

#define MAGIC_NUMBER				0xFCFDFEFF

/* Data size is 4 + 128 + 4 + 1 = 137 bytes. We pad to 140 bytes as this is the nearest divisible by 4 we have. So 
	 we add 3 bytes */

typedef struct comms
{
	uint32_t magic;
	uint32_t dataSize;
	char buffer[MAX_DATA_SIZE];
	unsigned char checksum;
	char dummy[3];
} TComms;

static char _privateBuffer[PACKET_SIZE];

static TResult assemble(char *outputBuffer, const char *inputBuffer, int len)
{
	// For copying to output buffer
	static int counter=0;

	// If there's leftover bytes from the next transmission
	static int leftoverFlag=0;
	static int leftoverCount=0;
	static char leftoverBuffer[PACKET_SIZE];

	int bytesLeft;
	int i;	

	// Copy in leftovers
	if(leftoverFlag)
	{
		int copyCount;
		if(leftoverCount <= PACKET_SIZE)
		{
			leftoverFlag=0;
			copyCount = leftoverCount;
		}
		else
			copyCount = PACKET_SIZE;

		leftoverCount -= copyCount;

		for(i=0; i<copyCount; i++)
		{
			outputBuffer[counter++] = leftoverBuffer[i];
		}
	}

	if(counter + len >= PACKET_SIZE)
	{
		bytesLeft = (PACKET_SIZE - counter);
		leftoverFlag=1;
		int bytesToCopy = len - bytesLeft;

		// Copy to leftover buffer
		for(i=0; i<bytesToCopy; i++)
		{
			leftoverBuffer[leftoverCount+i] = inputBuffer[bytesLeft + i];
		}
		leftoverCount += bytesToCopy;
	}
	else
		bytesLeft = len;

	for(i=0; i<bytesLeft; i++)
		outputBuffer[counter++] = inputBuffer[i];

	if(counter == PACKET_SIZE)
	{
		counter = 0;
		return PACKET_COMPLETE;
	}
	else
		return PACKET_INCOMPLETE;
}


TResult deserialize(const char *buffer, int len, void *output)
{
	TResult result = assemble(_privateBuffer, buffer, len);

	if(result == PACKET_COMPLETE)
	{
		// Extract out the comms packet
		TComms *comms = (TComms *) _privateBuffer;

		// Check that we have a valid packet
		if(comms->magic != MAGIC_NUMBER)
		{
			printf("BAD MAGIC NUMBER. EXPECTED %x GOT %x\n", MAGIC_NUMBER, comms->magic);
			return PACKET_BAD;
		}

		// Packet is valid. Now let's do the checksum
		unsigned char checksum = 0;

		unsigned int i;

		for(i=0; i<comms->dataSize; i++)
			checksum ^= comms->buffer[i];

		if(checksum != comms->checksum)
			return PACKET_CHECKSUM_BAD;
		else
		{
			memcpy(output, comms->buffer, comms->dataSize);
			return PACKET_OK;
		}
	}
	else
		return result;
}

int serialize(char *buffer, void *dataStructure, size_t size)
{
	TComms comms;

	// We use this to detect for malformed packets
	comms.magic = MAGIC_NUMBER;

	// Copy over the data structure
	memcpy(comms.buffer, dataStructure, size);

	// Now we take a checksum
	unsigned char checksum = 0;

	unsigned i;

	for(i=0; i<size; i++)
		checksum ^= comms.buffer[i];

	comms.checksum = checksum;
	comms.dataSize = size;

	memcpy(buffer, &comms, sizeof(TComms));

	return sizeof(TComms);
}
