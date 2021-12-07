#pragma once

#include "BufferFloat.h"

class DoubleBufferFloat {
public:

	enum class BufferSelector
	{
		IN,
		OUT
	};

	DoubleBufferFloat(size_t capacity = 32);
	//DoubleBufferFloat(const DoubleBufferFloat &doubleBuffer);
	//DoubleBufferFloat operator=(const DoubleBufferFloat &doubleBuffer);
	~DoubleBufferFloat();
	uint8_t push_back(float f, uint32_t * timestamp = nullptr);
	size_t getData(float ** data, uint32_t * timestamp = nullptr, bool swapBuffers = true);	// Swaps the input and output buffers by default			
	bool swap(); // Swaps the input and output buffers
																																																//void setAutoResize(bool b);
	size_t size(BufferSelector b);
	size_t inSize();	// deprecated, use size(BufferSelector b)
	size_t outSize();	// deprecated, use size(BufferSelector b)
	size_t capacity(BufferSelector b);
	size_t inCapacity();	// deprecated, use capacity(BufferSelector b)
	size_t outCapacity();	// deprecated, use capacity(BufferSelector b)

	void resize(size_t capacity);

	size_t getOverflowCount(BufferSelector b);
	uint8_t incrOverflowCount(BufferSelector b, unsigned int n = 1);
	size_t getClippedCount(BufferSelector b);
	uint8_t incrClippedCount(BufferSelector b, unsigned int n = 1);

	uint8_t downsample(BufferFloat* b);

private:
	BufferFloat* _buffer1;
	BufferFloat* _buffer2;
	BufferFloat* _inputBuffer;
	BufferFloat* _outputBuffer;
	bool _isPushing = false;
	bool _isGetting = false;

	BufferFloat* _getBuffer(BufferSelector b);
};

