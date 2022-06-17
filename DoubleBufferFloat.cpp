#include "DoubleBufferFloat.h"

DoubleBufferFloat::DoubleBufferFloat(size_t capacity) {
	_buffer1 = new BufferFloat(capacity);
	_buffer2 = new BufferFloat(capacity);
	_inputBuffer = _buffer1;
	_outputBuffer = _buffer2;
}

// ToDo: Verify copy constructor
//DoubleBufferFloat::DoubleBufferFloat(const DoubleBufferFloat &doubleBuffer) {
//	*_buffer1 = *doubleBuffer._buffer1;
//	*_buffer2 = *doubleBuffer._buffer2;
//	_inputBuffer = _buffer1;
//	_outputBuffer = _buffer2;
//}

// ToDo: Create assignment operator
//DoubleBufferFloat DoubleBufferFloat::operator=(const DoubleBufferFloat &doubleBuffer) {
//	*_buffer1 = *doubleBuffer._buffer1;
//	*_buffer2 = *doubleBuffer._buffer2;
//	_inputBuffer = _buffer1;
//	_outputBuffer = _buffer2;
//}

DoubleBufferFloat::~DoubleBufferFloat() {
	_inputBuffer = nullptr;
	_outputBuffer = nullptr;
	delete _buffer1;
	delete _buffer2;
}

uint8_t DoubleBufferFloat::push_back(float f, uint32_t * timestamp) {
	_isPushing = true;
	if (_inputBuffer != nullptr) {
		return _inputBuffer->push_back(f, timestamp);// | (_isGetting) ? BufferFloat::PUSH_WHILE_GETTING : BufferFloat::SUCCESS;
	}
	_isPushing = false;
	return BufferFloat::ERROR_PTR_NULL;
}

size_t DoubleBufferFloat::getData(float ** data, uint32_t * timestamp, bool swapBuffers) 
{
#ifdef DEBUG
	Serial.println("getData()");
#endif // DEBUG
	_isGetting = true;

	// ToDo: Verify this is interrupt safe

	if (_outputBuffer != nullptr) 
	{
		if (swapBuffers)
		{
			swap();
		}
		(*data) = _outputBuffer->data;
		if (timestamp != nullptr) {
			(*timestamp) = _outputBuffer->timestamp;
		}
		return _outputBuffer->size();
	}
	_isGetting = false;
	return BufferFloat::ERROR_PTR_NULL;
}

bool DoubleBufferFloat::swap()
{
	// ToDo: add a mutex to handle multithreaded CPUs
	
	if (_outputBuffer != nullptr)
	{
		_outputBuffer->clear();
		if (_inputBuffer == _buffer1) {
#ifdef DEBUG
			Serial.println("_inputBuffer == &_buffer2");
#endif // DEBUG
			_inputBuffer = _buffer2;
			_outputBuffer = _buffer1;
		}
		else {
#ifdef DEBUG
			Serial.println("_inputBuffer != &_buffer1");
#endif // DEBUG
			_inputBuffer = _buffer1;
			_outputBuffer = _buffer2;
		}
		return true;
	}
	return false;
}

//void DoubleBufferFloat::setAutoResize(bool b) {
//	_buffer1->autoResize = b;
//	_buffer2->autoResize = b;
//}

size_t DoubleBufferFloat::size(BufferSelector b)
{
	if (_getBuffer(b) == nullptr) return 0;
	else return _getBuffer(b)->size();
}


size_t DoubleBufferFloat::inSize() {
	if (_inputBuffer == nullptr) return 0;
	else {
		return _inputBuffer->size();
	}
}

size_t DoubleBufferFloat::outSize() {
	if (_outputBuffer == nullptr) return 0;
	else {
		return _outputBuffer->size();
	}
}

size_t DoubleBufferFloat::capacity(BufferSelector b)
{
	if (_getBuffer(b) == nullptr) return 0;
	else return _getBuffer(b)->capacity();
}

size_t DoubleBufferFloat::inCapacity() {
	if (_inputBuffer == nullptr) return 0;
	else {
		return _inputBuffer->capacity();
	}
}

size_t DoubleBufferFloat::outCapacity() {
	if (_outputBuffer == nullptr) return 0;
	else {
		return _outputBuffer->capacity();
	}
}

void DoubleBufferFloat::resize(size_t capacity) {
	_inputBuffer = nullptr;
	_outputBuffer = nullptr;
	delete _buffer1;
	delete _buffer2;

	_buffer1 = new BufferFloat(capacity);
	_buffer2 = new BufferFloat(capacity);
	_inputBuffer = _buffer1;
	_outputBuffer = _buffer2;
}

BufferFloat* DoubleBufferFloat::_getBuffer(BufferSelector b)
{
	if (b == BufferSelector::IN)
	{
		return _inputBuffer;
	}
	else
	{
		return _outputBuffer;
	}
}

size_t DoubleBufferFloat::getOverflowCount(BufferSelector b)
{
	if (_getBuffer(b) == nullptr) return 0;
	else return _getBuffer(b)->getOverflowCount();
}

uint8_t DoubleBufferFloat::incrOverflowCount(BufferSelector b, unsigned int n)
{
	if (_getBuffer(b) == nullptr) return BufferFloat::ERROR_PTR_NULL;
	else return _getBuffer(b)->incrOverflowCount(n);
}

size_t DoubleBufferFloat::getClippedCount(BufferSelector b)
{
	if (_getBuffer(b) == nullptr) return 0;
	else return _getBuffer(b)->getClippedCount();
}

uint8_t DoubleBufferFloat::incrClippedCount(BufferSelector b, unsigned int n)
{
	if (_getBuffer(b) == nullptr) return BufferFloat::ERROR_PTR_NULL; 
	else return _getBuffer(b)->incrClippedCount(n);
}

uint8_t DoubleBufferFloat::downsample(BufferFloat* b)
{
	uint8_t status = 0;
	
	if (b == nullptr) return BufferFloat::ERROR_PTR_NULL;

	// Perform data averaging
	float f = b->average();

	// Add to data double buffer
	status = status | push_back(f, &(b->timestamp));

	// Check for clipping
	if (b->getClippedCount() > 0)
	{
		status = status | incrClippedCount(DoubleBufferFloat::BufferSelector::IN);
	}

	// Check for overflow
	if (b->getOverflowCount() > 0)
	{
		status = status | BufferFloat::ERROR_BUFFER_OVERFLOW;
	}
	return status;
}
