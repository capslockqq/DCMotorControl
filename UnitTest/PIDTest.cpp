#include "EncoderMock.h"
#include "DCMotorMock.h"
#include "gmock/gmock.h"
#include "../DC-Motor-PID-Control/PID.cpp"
#include "../DC-Motor-PID-Control/QuadratureEncoder.cpp"
#include "../DC-Motor-PID-Control/DCMotor12Volts.cpp"

bool operator==(const PIDVal& lhs, const PIDVal& rhs)
{
	return lhs.kp == rhs.kp && lhs.ki == rhs.ki && lhs.kd == rhs.kd && lhs.Max == rhs.Max && lhs.Min == rhs.Min && lhs.IntegratorLimit == rhs.IntegratorLimit;
}

class PIDTestFixture : public ::testing::Test {
protected:
	// You can remove any or all of the following functions if its body
	// is empty.

	PIDTestFixture() {
		// You can do set-up work for each test here.
	}

	virtual ~PIDTestFixture() {
		// You can do clean-up work that doesn't throw exceptions here.
	}

	// If the constructor and destructor are not enough for setting up
	// and cleaning up each test, you can define the following methods:
	virtual void SetUp() {
		dummy = { 0 };
	}

	virtual void TearDown() {
		// Code here will be called immediately after each test (right
		// before the destructor).
	}


	// Objects declared here can be used by all tests in the test case for Project1.
	EncoderMock encoder;
	DCMotorMock dcMotor;
	PIDVal values;
	PIDVal dummy{ 0, 0, 0, 0, 0, 0 };
	Direction dir;
};



TEST_F(PIDTestFixture, GetPIDValuesFromValidValues) {
	values = { 1,1,2,4,4,100 };
	PID Controller = PID(&values, &encoder, &dcMotor);

	Controller.GetPIDValues(&dummy); //Placing PIDValues in dummy struct
	EXPECT_EQ(dummy, values);
}
TEST_F(PIDTestFixture, SetPIDValuesWithInvalidNumbers) {
	values = { -1,-1,-2,0,0,0 };
	PID Controller = PID(&values, &encoder, &dcMotor);
	PIDVal expected{ 0,0,0,0,0,0 };
	Controller.GetPIDValues(&dummy); //Placing PIDValues in dummy struct
	EXPECT_EQ(dummy, expected);
}
using ::testing::Return;

TEST_F(PIDTestFixture, CalculateCorrectionOnlyKp) {
	values = { 2.5, 0, 0, 300, -300, 0 };
	PID Controller = PID(&values, &encoder, &dcMotor);
	float setpoint = 30;
	float error = 0;

	EXPECT_CALL(encoder, GetPosition())
		.WillOnce(Return(100))
		.WillOnce(Return(60))
		.WillOnce(Return(20));
	error = (setpoint - 100)*values.kp;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
	error = (setpoint - 60)*values.kp;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
	error = (setpoint - 20)*values.kp;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
}

TEST_F(PIDTestFixture, CalculateCorrectionKpAndKi) {
	values = { 0.6, 2, 0, 3000, -3000, 1000 };
	PID Controller = PID(&values, &encoder, &dcMotor);
	float setpoint = 40;
	float error = 0;
	float integral = 0;
	EXPECT_CALL(encoder, GetPosition()) // Fake data from sensor
		.WillOnce(Return(120))
		.WillOnce(Return(100))
		.WillOnce(Return(44));
	integral += setpoint - 120;
	error = (setpoint - 120)*values.kp+integral*values.ki;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
	integral += setpoint - 100;
	error = (setpoint - 100)*values.kp + integral * values.ki;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
	integral += setpoint - 44;
	error = (setpoint - 44)*values.kp + integral * values.ki;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
}

TEST_F(PIDTestFixture, CalculateCorrectionKpKiAndKd) {
	values = { 0.6, 2, 23, 3000, -3000, 1000 };
	PID Controller = PID(&values, &encoder, &dcMotor);
	float setpoint = 40;
	float error = 0;
	float integral = 0;
	float derivative = 0;
	EXPECT_CALL(encoder, GetPosition()) // Fake data from sensor
		.WillOnce(Return(120))
		.WillOnce(Return(100))
		.WillOnce(Return(44));

	integral += setpoint - 120;
	error = (setpoint - 120)*values.kp + integral * values.ki + derivative*values.kd;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
	integral += setpoint - 100;
	derivative = (setpoint - 100)-(setpoint - 120);
	error = (setpoint - 100)*values.kp + integral * values.ki + derivative * values.kd;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());	
	integral += setpoint - 44;
	derivative = (setpoint - 44)-(setpoint - 100);
	error = (setpoint - 44)*values.kp + integral * values.ki + derivative * values.kd;
	Controller.Control(setpoint);
	EXPECT_EQ(error, Controller.GetCorrection());
}

TEST_F(PIDTestFixture, ExceedMinimumAndMaximumValues) {
	values = { 1, 0, 0, 10, -10, 1000 };
	PID Controller = PID(&values, &encoder, &dcMotor);
	EXPECT_CALL(encoder, GetPosition()) // Fake data from sensor
		.WillRepeatedly(Return(0));
	Controller.Control(11);
	EXPECT_EQ(Controller.GetCorrection(), values.Max);
	Controller.Control(-11);
	EXPECT_EQ(Controller.GetCorrection(), values.Min);
	Controller.Control(-10);
	EXPECT_EQ(Controller.GetCorrection(), values.Min);
	Controller.Control(10);
	EXPECT_EQ(Controller.GetCorrection(), values.Max);
	Controller.Control(-9);
	EXPECT_EQ(Controller.GetCorrection(), -9);
	Controller.Control(9);
	EXPECT_EQ(Controller.GetCorrection(), 9);
}

TEST_F(PIDTestFixture, IntegralWindupProtection) {

	values = { 0, 1, 0, 10, -10, 5 };
	PID Controller = PID(&values, &encoder, &dcMotor);
	float integral = 0;
	float setpoint = 2;
	float error = 0;
	
	EXPECT_CALL(encoder, GetPosition()) // Fake data from sensor
		.WillRepeatedly(Return(0));
	integral += setpoint; //Since the sensor data keeps being 0
	Controller.Control(setpoint);
	Controller.GetPIDValues(&dummy);
	EXPECT_LT(Controller.GetIntegral(), dummy.IntegratorLimit);
	Controller.Control(setpoint);
	Controller.Control(setpoint);
	Controller.GetPIDValues(&dummy);
	EXPECT_EQ(Controller.GetIntegral(), dummy.IntegratorLimit);
	Controller.Control(-10);
	EXPECT_EQ(Controller.GetIntegral(), -dummy.IntegratorLimit);
}
