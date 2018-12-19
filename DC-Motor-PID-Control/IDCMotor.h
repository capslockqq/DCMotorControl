#pragma once
typedef enum {
	left,
	right,
}Direction;
class IDCMotor{
public:
	virtual void SetSpeed(float voltage, Direction dir) = 0;
};