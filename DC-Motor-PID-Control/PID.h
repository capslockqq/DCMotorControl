#pragma once

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
	PID();
	PID(PIDVal *val);
	void SetPIDValues(PIDVal *val);
	void GetPIDValues(PIDVal *val);
	float Correction(float setpoint, float error);
	~PID();
private:
	float _correction;
	float _integral = 0;
	float _previousError = 0;
	PIDVal _val;
};

