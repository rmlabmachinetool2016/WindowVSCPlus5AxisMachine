// 5AxisMachineMotorTest.cpp : main project file.

#include "stdafx.h"
#include "5AxisMotorTest.h"
#include <string>
using std::string; 
using namespace My5AxisMachineMotorTest;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
//	string mystring;
	// Enabling Windows XP visual effects before any controls are created
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 

	// Create the main window and run it
	Application::Run(gcnew MotorTestForm());
	return 0;
}
