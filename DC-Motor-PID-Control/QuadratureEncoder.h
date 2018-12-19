#pragma once
#include "IEncoder.h"
class QuadratureEncoder : public IEncoder
{
public:
	QuadratureEncoder();
	float GetPosition();
	~QuadratureEncoder();
};

