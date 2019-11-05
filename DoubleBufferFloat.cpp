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
}

size_t DoubleBufferFloat::getData(float ** data, uint32_t * timestamp) {
#ifdef DEBUG
	Serial.println("getData()");
#endif // DEBUG
	_isGetting = true;

	// ToDo: Verify this is interrupt safe

	if (_inputBuffer != nullptr && _outputBuffer != nullptr) {
		_outputBuffer->clear();
		if (_inputBuffer == _buffer1) {
#ifdef DEBUG
			Serial.println("_inputBuffer == &_buffer1");
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
		(*data) = _outputBuffer->data;
		if (timestamp != nullptr) {
			(*timestamp) = _outputBuffer->timestamp;
		}
		return _outputBuffer->size();
	}
	_isGetting = false;
}

//void DoubleBufferFloat::setAutoResize(bool b) {
//	_buffer1->autoResize = b;
//	_buffer2->autoResize = b;
//}

size_t DoubleBufferFloat::inSize() {
	if (_inputBuffer != nullptr) {
		_inputBuffer->size();
	}
}

size_t DoubleBufferFloat::outSize() {
	if (_outputBuffer != nullptr) {
		_outputBuffer->size();
	}
}

size_t DoubleBufferFloat::inCapacity() {
	if (_inputBuffer != nullptr) {
		_inputBuffer->capacity();
	}
}

size_t DoubleBufferFloat::outCapacity() {
	if (_outputBuffer != nullptr) {
		_outputBuffer->capacity();
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
