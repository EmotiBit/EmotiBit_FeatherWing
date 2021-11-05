#include "BufferFloat.h"

BufferFloat::BufferFloat(size_t capacity) {
#ifdef DEBUG
	Serial.print("BufferFloat: ");
	Serial.println(capacity);
#endif
	//autoResize = false;
	_size = 0;
	_capacity = capacity;
	//data = (float*) calloc(_capacity, sizeof(float));
	timestamp = 0;
	_nOverflow = 0;
	_nClipped = 0; 
	data = new float[_capacity];
}

// ToDo: Verify copy constructor
//BufferFloat::BufferFloat(const BufferFloat &buffer) {
//	autoResize = buffer.autoResize;
//	_size = buffer._size;
//	_capacity = buffer.capacity;
//	timestamp = buffer.timestamp;
//	data = new float[_capacity];
//	for (uint32_t i = 0; i < buffer._size; i++) {
//		data[i] = buffer.data[i];
//	}
//}

BufferFloat::~BufferFloat() {
	//free(data);
	delete[] data;
}

uint8_t BufferFloat::push_back(float f, uint32_t * dataTimestamp) {
	if (_size >= _capacity) {
		//if (autoResize) {
		//	// ToDo: Add check for free RAM
		//	float * temp = data;
		//	_capacity *= 2;
		//	//data = (float*)calloc(_capacity, sizeof(float));
		//	data = new float[_capacity];
		//	memcpy(data, temp, sizeof(float) * _size);
		//	//free(temp);
		//	delete[] temp;
		//}
		//else {
		//	return ERROR_BUFFER_OVERFLOW;
		//}

		return ERROR_BUFFER_OVERFLOW | incrOverflowCount();
	}
	else {
		if (dataTimestamp == nullptr) {
			timestamp = millis();
		}
		else {
			timestamp = *dataTimestamp;
		}
		data[_size] = f;
		_size++;
		return SUCCESS;
	}
}

void BufferFloat::clear() {
#ifdef DEBUG
	Serial.println("clear()");
#endif // DEBUG
	_size = 0;
	timestamp = 0;
	_nOverflow = 0;
	_nClipped = 0; 
}

size_t BufferFloat::size() {
	return _size;
}

size_t BufferFloat::capacity() {
	return _capacity;
}

bool BufferFloat::isFull()
{
	if (_size >= _capacity) return true;
	else return false;
}

size_t BufferFloat::getOverflowCount()
{
	return _nOverflow;
}

uint8_t BufferFloat::incrOverflowCount(unsigned int n)
{
	if (_nOverflow <= SIZE_MAX - n)
	{
		_nOverflow += n;
		return SUCCESS;
	}
	else
	{
		_nOverflow = SIZE_MAX;
		return ERROR_SIZE_MAX_EXCEEDED;
	}
}

size_t BufferFloat::getClippedCount()
{
	return _nClipped;
}

uint8_t BufferFloat::incrClippedCount(unsigned int n)
{
	if (_nClipped <= SIZE_MAX - n)
	{
		_nClipped += n;
		return SUCCESS;
	}
	else
	{
		_nClipped = SIZE_MAX;
		return ERROR_SIZE_MAX_EXCEEDED;
	}
}

float BufferFloat::average() {
	float f = 0;
	for (int i = 0; i < size(); i++) {
		f += (float) data[i];
	}
	f /= (float) size();
	return f;
}