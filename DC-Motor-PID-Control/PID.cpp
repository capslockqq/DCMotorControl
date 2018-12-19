#include "PID.h"

PID::PID()
{
	PIDVal filler{ 0,0,0,0,0,0 };
	SetPIDValues(&filler);
}

PID::PID(PIDVal *val)
{
	SetPIDValues(val);
}

void PID::SetPIDValues(PIDVal * val)
{
	_val.kp = val->kp >= 0 ? val->kp : 0;
	_val.ki = val->ki >= 0 ? val->ki : 0;
	_val.kd = val->kd >= 0 ? val->kd : 0;

	_val.Max = val->Max;
	_val.Min = val->Min;
	_val.IntegratorLimit = val->IntegratorLimit;
}

void PID::GetPIDValues(PIDVal *val) {
	val->kp = _val.kp;
	val->ki = _val.ki;
	val->kd = _val.kd;

	val->Max = _val.Max;
	val->Min = _val.Max;
}

void PID::Correction(float setpoint, float measurement)
{
	float error = setpoint - measurement;
	float derivative = error - _previousError;
	_integral += error;
	_integral = _integral > _val.IntegratorLimit ? _val.IntegratorLimit : _integral;//Integral windup protection
	
	float tmpCorrection = error * _val.kp + _integral*_val.ki + derivative*_val.kd;
	
	if (tmpCorrection > _val.Max) {
		tmpCorrection = _val.Max;
	}
	else if (tmpCorrection < _val.Min) {
		tmpCorrection = _val.Min;
	}
	_correction = tmpCorrection;
	_previousError = error;


}




PID::~PID()
{
}
