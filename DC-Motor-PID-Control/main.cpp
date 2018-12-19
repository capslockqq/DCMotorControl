// DC-Motor-PID-Control.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "PID.h"

int main()
{
    std::cout << "Hello World!\n"; 
	PIDVal values{ -3,-5,1,4,4, 100};
	PID Controller = PID(&values);

	PIDVal dummy{ 0,0,0,0,0 };

	Controller.GetPIDValues(&dummy);

	std::cout << dummy.kp << std::endl;
	std::cout << dummy.ki << std::endl;
	std::cout << dummy.kd << std::endl;
	std::cout << dummy.Max << std::endl;
	std::cout << dummy.Min << std::endl;
	std::cout << dummy.IntegratorLimit << std::endl;

}
