#include "pch.h"
#include "../DC-Motor-PID-Control/PID.cpp"
#include "../DC-Motor-PID-Control/QuadratureEncoder.cpp"
#include "../DC-Motor-PID-Control/DCMotor12Volts.cpp"

bool operator==(const PIDVal& lhs, const PIDVal& rhs)
{
	return lhs.kp == rhs.kp;
}

TEST(PIDTest, GetPIDValues) {
	PIDVal values{ 1,1,2,4,4,100 };
	PID Controller = PID(&values);

	PIDVal dummy{ 0,0,0,0,0 };
	Controller.GetPIDValues(&dummy); //Placing PIDValues in dummy struct
	EXPECT_EQ(dummy, values);
}