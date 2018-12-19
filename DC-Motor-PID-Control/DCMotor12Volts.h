#pragma once
#include "IDCMotor.h"
class DCMotor12Volts : public IDCMotor
{
public:
	DCMotor12Volts();
	void SetSpeed(float voltage, Direction dir);
	~DCMotor12Volts();
};

