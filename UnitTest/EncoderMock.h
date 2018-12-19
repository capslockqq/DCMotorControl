#pragma once
#include "gmock/gmock.h"
#include "../DC-Motor-PID-Control/IEncoder.h"
class EncoderMock : public IEncoder {
public:
	MOCK_METHOD0(GetPosition, float());
};
