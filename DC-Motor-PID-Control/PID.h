#pragma once
#include "IEncoder.h"
#include "IDCMotor.h"
typedef struct {
	float kp;
	float ki;
	float kd;
	float Max;
	float Min;
	float IntegratorLimit;
}PIDVal;

class PID
{
public:
	PID(PIDVal *val, IEncoder *encoder, IDCMotor *dcMotor);
	void SetPIDValues(PIDVal *val);
	void GetPIDValues(PIDVal *val);
	float GetIntegral();
	float GetCorrection();
	float GetMeasurement();
	void Control(float setpoint);
	~PID();
private:
	float Correction(float setpoint);
	float _correction = 0;
	float _integral = 0;
	float _previousError = 0;
	bool _firstTime = true;
	float _measurement;
	Direction _dir;
	PIDVal _val = { 0 };
	IEncoder *_encoder;
	IDCMotor *_dcMotor;
};

