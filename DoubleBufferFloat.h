#pragma once

#include "BufferFloat.h"

class DoubleBufferFloat {
private:
	BufferFloat* _buffer1;
	BufferFloat* _buffer2;
	BufferFloat* _inputBuffer;
	BufferFloat* _outputBuffer;
	bool _isPushing = false;
	bool _isGetting = false;
public:



	DoubleBufferFloat(size_t capacity = 64);
	//DoubleBufferFloat(const DoubleBufferFloat &doubleBuffer);
	//DoubleBufferFloat operator=(const DoubleBufferFloat &doubleBuffer);
	~DoubleBufferFloat();
	uint8_t push_back(float f, uint32_t * timestamp = nullptr);
	size_t getData(float ** data, uint32_t * timestamp = nullptr);
	//void setAutoResize(bool b);
	size_t inSize();
	size_t outSize();
	size_t inCapacity();
	size_t outCapacity();
	void resize(size_t capacity);
};

