#ifndef __SERIALIZE__
#define __SERIALIZE__

#include <stdlib.h>

#define PACKET_SIZE		140
#define MAX_DATA_SIZE				128

typedef enum
{
	PACKET_OK = 0,
	PACKET_BAD = 1,
	PACKET_CHECKSUM_BAD = 2,
	PACKET_INCOMPLETE = 3,
	PACKET_COMPLETE = 4
} TResult;

int serialize(char *buffer, void *dataStructure, size_t size);
TResult deserialize(const char *buffer, int len, void *output);

#endif
