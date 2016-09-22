#pragma once
#include "StdAfx.h"
#include "FiveAxisCNC.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
using namespace boost::numeric::ublas; 
// #include <string>
// using std::string;
namespace RmLabCNC{
static vector<double>  vec_AbsolutePosition,vec_PartOrigin;
//public ref class FiveAxisIOModule
ref class FiveAxisIOModule
{
public:
	static const double PI = 3.1415926535897932384626433832795;
//	string mystring;
//	string  AioDeviceName, CntDeviceName;
//	String	AioDeviceName,CntDeviceName;
// 	String^	AioDeviceNameX,^ CntDeviceNameX;
// 	StringBuilder	^ErrorStringX;
	// parameter of reference
// 	static const double RADIUS			= 0.01;					// m
// 	static const double OMG				= (2*PI) / 1 * 1.00;					// rad/sec
/*	static const double SPEEDZ			= 0.001;	// m/sec*/
	static const double INIT_POS_X		= 0.0,					// initial position(m,rad)
		INIT_POS_Y1		= 0.0,//0.01,//-0.01,	//“ñŽŸŒ³‚ÌŽž‚Í‚±‚ê
		INIT_POS_Y2		= INIT_POS_Y1,
		INIT_POS_Z		= 0.0,//-0.01,//-0.010,
		INIT_ANGLE_C	= 0.0,
		INIT_ANGLE_A1	= 0.0,
		INIT_ANGLE_A2	= INIT_ANGLE_A1;
// hardware data
// 	static const double RESONATE_LINER_ENC_X		= (1.0/10000000.0);		// m/pulse
// 	static const double RESONATE_LINER_ENC_Y1		= (1.0/10000000.0);		// m/pulse
// 	static const double RESONATE_LINER_ENC_Y2		= (1.0/10000000.0);		// m/pulse
// 	static const double RESONATE_LINER_ENC_Z		= (1.0/10000000.0);		// m/pulse

	static const double RESONATE_LINER_ENC_X		= (1.0/10000.0);		// mm/pulse
	static const double RESONATE_LINER_ENC_Y1		= (1.0/10000.0);		// mm/pulse
	static const double RESONATE_LINER_ENC_Y2		= (1.0/10000.0);		// mm/pulse
	static const double RESONATE_LINER_ENC_Z		= (1.0/10000.0);		// mm/pulse
	static const double RESONATE_ROTATION_ENC_C		= (360/1562500.0);	//(2.0*PI/1562500.0);	// rad/pulse
	static const double RESONATE_ROTATION_ENC_A1	= (360/3125000.0);//(2.0*PI/3125000.0);	// rad/pulse
	static const double RESONATE_ROTATION_ENC_A2	= (360/3125000.0);//(2.0*PI/3125000.0);	// rad/pulse
	static const double MAX_FORCE_X					= 36.0;					// N
	static const double MAX_FORCE_Y1				= 36.0;					// N
	static const double MAX_FORCE_Y2				= 36.0;					// N
	static const double MAX_FORCE_Z					= 36.0;					// N
	static const double MAX_TORQUE_C				= 2.0;					// N*m
	static const double MAX_TORQUE_A1				= 5.0;					// N*m
	static const double MAX_TORQUE_A2				= 5.0;					// N*m
	
	static const short NUM_LINEAR_ACTUATOR	= 4;					// number of linear motor
	static const short NUM_ROTATION_ACTUATOR	= 3;					// number of motor
	static const short NUM_ACTUATOR			= 7;//NUM_ROTATION_ACTUATOR + NUM_LINEAR_ACTUATOR;
	// number of actuator
//	static const short NUM_COUNTER			= 7; // number of counter ch. Already defined
	
	static const int DA_MAX_VOLT			= 10;
	static const int DA_MIN_VOLT			= -10;//-DA_MAX_VOLT;

// 	double MAX_FORCE_X, MAX_FORCE_Y1, MAX_FORCE_Y2, MAX_FORCE_Z, MAX_TORQUE_C, MAX_TORQUE_A1, MAX_TORQUE_A2;	
// 	short NUM_LINEAR_ACTUATOR, NUM_ROTATION_ACTUATOR, NUM_ACTUATOR;
// 	int DA_MAX_VOLT,DA_MIN_VOLT;
	double SAMPLING_TIME;				// sec

	short  CntId,AioId;
	System::String^	AioDeviceName,^ CntDeviceName;
	long AioRet,AioRet2,CntRet,CntRet2;	
	FiveAxisIOModule(void);
	virtual ~FiveAxisIOModule(void);
	void ConnectToCNC(System::String^ &errorX);
	void DisconnectToCNC(System::String^ &Disconnecterror);
	void EmergencyStopCNC(System::String^ &EmergencyStopError);
	void OutputAllMotor(vector<double>& OutputForce);
	void StopAllMotor();
	void OutputOneMotor(short m_iMotorNumber, double OutputForce);
	void StartCounter(System::IntPtr pTimer, System::String^ &StartCounterError, int m_hwnd);
	void StopCounter(System::String^ &StopCounterError);
	void ResetCounterToOrigin(void);
	vector<double> GetAbsolutePosition(void);
	vector<double> GetAbsPosition(void);
	void SetPartOrigin(void);
	vector<double> GetMachiningPosition(void);

	

private:
//	std::string	mystringX;
//   string title[200];
 //    string mystring;
// 	System::String const^ mystringCst;
// 	System::String^ mystring;
	System::String^ mystring;
};
}
