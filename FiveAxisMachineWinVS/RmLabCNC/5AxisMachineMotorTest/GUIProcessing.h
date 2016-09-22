#pragma once
namespace My5AxisMachineMotorTest{
namespace GUIMainProcess{
ref class GUIProcessing
{
public:
	GUIProcessing(void);
	virtual ~GUIProcessing(void);
	bool bConnectStatus;
	double SAMPLING_TIME;
	double time;
	double maxTime;
	unsigned int step;
};
}}
