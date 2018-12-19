#pragma once
#include "IDCMotor.h"
class DCMotor12Volts : public IDCMotor
{
public:
	DCMotor12Volts();
	void SetSpeed(float voltage);
	~DCMotor12Volts();
};

