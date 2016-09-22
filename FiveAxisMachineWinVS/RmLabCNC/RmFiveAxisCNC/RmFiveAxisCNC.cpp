// RmFiveAxisCNC.cpp : main project file.

#include "stdafx.h"
#include "FiveAxisCNCForm.h"

using namespace RmFiveAxisCNC;

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// Enabling Windows XP visual effects before any controls are created
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false); 

	// Create the main window and run it
	Application::Run(gcnew FiveAxisCNCForm());
	return 0;
}
