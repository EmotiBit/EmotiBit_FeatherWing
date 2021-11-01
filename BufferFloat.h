#pragma once

//#define DEBUG

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
//#include <cstring>
#include <Arduino.h>

#ifdef DEBUG
#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
#endif

class BufferFloat {
private:
	size_t _capacity; /**< total capacity */
	size_t _size; /**< number of elements in buffer */
	size_t _nOverflow;	// Automatically any data overflow events
	size_t _nClipped;	// Stores any manually reported data clipping events 

public:
	float * data;
	uint32_t timestamp;	// time of the most recent push_back in millis()
	//bool autoResize;

	static const uint8_t SUCCESS = 0;
	static const uint8_t ERROR_BUFFER_OVERFLOW = 1;
	static const uint8_t PUSH_WHILE_GETTING = 2;
	static const uint8_t GET_WHILE_PUSHING = 4;
	static const uint8_t ERROR_SIZE_MAX_EXCEEDED = 8;
	static const uint8_t ERROR_PTR_NULL = 16;

	BufferFloat(size_t capacity = 16);
	~BufferFloat();
	//uint8_t resize(size_t newSize);
	uint8_t push_back(float f, uint32_t * dataTimestamp = nullptr);
	void clear();
	size_t size();
	size_t capacity();
	bool isFull();
	size_t getOverflowCount();
	uint8_t incrOverflowCount(unsigned int n = 1);
	size_t getClippedCount();
	uint8_t incrClippedCount(unsigned int n = 1);
	float average();
};
