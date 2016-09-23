#include "StdAfx.h"
#include "FiveAxisIOModule.h"
// #include <string>
// using std::string;
using namespace RmLabCNC;
FiveAxisIOModule::FiveAxisIOModule(void)
{
// 	DA_MAX_VOLT			= 10;
// 	DA_MIN_VOLT			= -DA_MAX_VOLT;
// 
// 	NUM_LINEAR_ACTUATOR	= 4;					// number of linear motor
// 	NUM_ROTATION_ACTUATOR	= 3;					// number of motor
// 	NUM_ACTUATOR			= NUM_ROTATION_ACTUATOR + NUM_LINEAR_ACTUATOR;
// 
// 	MAX_FORCE_X					= 36.0;					// N
// 	MAX_FORCE_Y1				= 36.0;					// N
// 	MAX_FORCE_Y2				= 36.0;					// N
// 	MAX_FORCE_Z					= 36.0;					// N
// 	MAX_TORQUE_C				= 2.0;					// N*m
// 	MAX_TORQUE_A1				= 5.0;					// N*m
// 	MAX_TORQUE_A2				= 5.0;					// N*m
	SAMPLING_TIME				= 0.002;				// sec
	AioId = 0;
	CntId = 0;
	AioDeviceName = "AIO000";// "AIO000"; "AIO001";
	CntDeviceName = "CNT001";
	vec_AbsolutePosition.resize(NUM_COUNTER);
	vec_PartOrigin.resize(NUM_COUNTER);
	vec_AbsolutePosition.clear();
	vec_PartOrigin.clear();
}


FiveAxisIOModule::~FiveAxisIOModule(void)
{
}
void FiveAxisIOModule::ConnectToCNC(System::String^ &Connecterror)
{
//string	AioDeviceName,CntDeviceName, ErrorString;
//	string mystring;
	StringBuilder	^ErrorString;
	pin_ptr<short>pAioId = &static_cast<short>(AioId);
	pin_ptr<short>pCntId = &static_cast<short>(CntId);



	// 初期化処理
//	ErrorString->Append("OK");
// 	AioDeviceName = "AIO000";
// 	CntDeviceName = "CNT000";

	AioRet = CAioCLI::AioInit(AioDeviceName, pAioId);
	Connecterror = "OK";
	if (AioRet != 0){
		AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
		Connecterror = String::Format("AioInitError= {0:d}: {1}",AioRet,ErrorString);
		return;
//		return String::Format("CntSelectChannelSignal= {0:d}: {1}",CntRet,ErrorString);
	}
	
	CntRet = CCntCLI::CntInit(CntDeviceName, pCntId);
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		Connecterror = String::Format("CntInitError= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
	// ＤＡボードのプロセスリセット
	AioRet = CAioCLI::AioResetProcess ( AioId );
	if (AioRet != 0){
		AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
		Connecterror = String::Format("AioResetProcess= {0:d}: {1}",AioRet,ErrorString);
		return;
	}

	// デバイスのリセット
	AioRet = CAioCLI::AioResetDevice(AioId);
	if (AioRet != 0){
		AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
		Connecterror = String::Format("AioResetDevice= {0:d}: {1}",AioRet,ErrorString);
		return;
	}
	CntRet = CCntCLI::CntResetDevice(CntId);
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		Connecterror = String::Format("CntResetDevice= {0:d}: {1}",CntRet,ErrorString);
		return;
	}

	// 出力レンジの設定
	AioRet = CAioCLI::AioSetAoRangeAll(AioId, PM10);
	if (AioRet != 0){
		AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
		Connecterror = String::Format("AioSetAoRangeAll= {0:d}: {1}",AioRet,ErrorString);
		return;
	}
	// カウンタ入力シグナルが差動であると指定
	for(short i=0;i<NUM_COUNTER;i++)
	{
		CntRet = CCntCLI::CntSelectChannelSignal(CntId,i,CNT_SIGTYPE_LINERECEIVER);
		if (CntRet != 0){
			CntRet2 =  CCntCLI::CntGetErrorString(CntRet, ErrorString);
			Connecterror = String::Format("CntSelectChannelSignal= {0:d}: {1}",CntRet,ErrorString);
			return;
		}
		CntRet = CCntCLI::CntSetOperationMode(CntId,i,CNT_MODE_2PHASE,CNT_MUL_X4,CNT_CLR_ASYNC);
		if (CntRet != 0){
			CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
			Connecterror = String::Format("CntSetOperationMode= {0:d}: {1}",CntRet,ErrorString);
			return;
		}
		CntRet = CCntCLI::CntSetZMode(CntId,i,CNT_ZPHASE_NOT_USE);
		if (CntRet != 0){
			CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
			Connecterror = String::Format("CntSetZMode= {0:d}: {1}",CntRet,ErrorString);
			return;
		}
		CntRet = CCntCLI::CntSetCountDirection (CntId,i,CNT_DIR_UP);
		if (CntRet != 0){
			CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
			Connecterror = String::Format("CntSetCountDirection= {0:d}: {1}",CntRet,ErrorString);
			return;
		}
	}

	return;
}

void FiveAxisIOModule::DisconnectToCNC(System::String^ &Disconnecterror)
{
	StringBuilder	^ErrorString;
	pin_ptr<short>pAioId = &static_cast<short>(AioId);
	pin_ptr<short>pCntId = &static_cast<short>(CntId);

	short ChNo[7] = {0,1,2,3,4,5,6};

	// 初期化処理
	//	ErrorString->Append("OK");
// 	AioDeviceName = "AIO000";
// 	CntDeviceName = "CNT000";

	Disconnecterror = "OK";

	CntRet = CCntCLI::CntOutputDOBit ( CntId , 0 , 0 , 0 );
	CntRet = CCntCLI::CntStopNotifyTimer ( CntId );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		Disconnecterror = String::Format("CntStopNotifyTimer= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
	// Stoping counter board and DA board
	CntRet = CCntCLI::CntStopCount ( CntId , ChNo , NUM_COUNTER );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		Disconnecterror = String::Format("CntStopCount= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
	for(short i=0;i<NUM_ACTUATOR-3;i++)// 3 Axis CNC have 3 motor and 1 spind motor
	{
		AioRet = CAioCLI::AioSingleAoEx(AioId,i,0);
		if (AioRet != 0){
			AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
			Disconnecterror = String::Format("AioSingleAoEx= {0:d}: {1}",CntRet,ErrorString);
			return;
		}
	}
	// ＤＡボードのクローズ
	AioRet = CAioCLI::AioExit(AioId);
	if (AioRet != 0){
		AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
		Disconnecterror = String::Format("AioExit= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
	// カウンタボードのクローズ
	CntRet = CCntCLI::CntExit(CntId);
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		Disconnecterror = String::Format("CntExit= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
}
void FiveAxisIOModule::EmergencyStopCNC(System::String^ &EmergencyStopError)
{
	StringBuilder	^ErrorString;
	pin_ptr<short>pAioId = &static_cast<short>(AioId);
	pin_ptr<short>pCntId = &static_cast<short>(CntId);

	short ChNo[7] = {0,1,2,3,4,5,6};
	EmergencyStopError = "OK";
	// Send to all actuator 0 Voltage
	for(short i=0;i<NUM_ACTUATOR-3;i++)
	{
		AioRet = CAioCLI::AioSingleAoEx(AioId,i,0);
		if (AioRet != 0){
			AioRet2 = CAioCLI::AioGetErrorString(AioRet, ErrorString);
			EmergencyStopError = String::Format("AioSingleAoEx= {0:d}: {1}",CntRet,ErrorString);
			return;
		}
	}

	CntRet = CCntCLI::CntOutputDOBit ( CntId , 0 , 0 , 0 );
	CntRet = CCntCLI::CntStopNotifyTimer ( CntId );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		EmergencyStopError = String::Format("CntStopNotifyTimer= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
	// Stoping counter board and DA board
	CntRet = CCntCLI::CntStopCount ( CntId , ChNo , NUM_COUNTER );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		EmergencyStopError = String::Format("CntStopCount= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
}
void FiveAxisIOModule::OutputAllMotor(vector<double>& OutputForce)
{
	int i;
	vector<double> force(OutputForce);
	float outputVolt[NUM_ACTUATOR];
	double max_force[NUM_LINEAR_ACTUATOR] = {MAX_FORCE_X,MAX_FORCE_Y1,MAX_FORCE_Y2,MAX_FORCE_Z};
	double max_torque[NUM_ROTATION_ACTUATOR] = {MAX_TORQUE_C,MAX_TORQUE_A1,MAX_TORQUE_A2};

	force(2) =  force(3);// Change value of Z axis to Y2 axis
	force(0) = -force(0);
//	force(1) = -force(1);
	for(i=0;i<NUM_LINEAR_ACTUATOR;i++)
	{
		if(force(i) > max_force[i])
			force(i) = max_force[i];
		else if(force(i) < -max_force[i])
			force(i) = -max_force[i];
		outputVolt[i] = static_cast<float>(force(i)/max_force[i]*DA_MAX_VOLT);
	}
	for(i=0;i<NUM_ROTATION_ACTUATOR;i++)
	{
		if(force(i+NUM_LINEAR_ACTUATOR) > max_torque[i])
			force(i+NUM_LINEAR_ACTUATOR) = max_torque[i];
		else if(force(i+NUM_LINEAR_ACTUATOR) < -max_torque[i])
			force(i+NUM_LINEAR_ACTUATOR) = -max_torque[i];
		outputVolt[i+NUM_LINEAR_ACTUATOR] = static_cast<float>(force(i+NUM_LINEAR_ACTUATOR)/max_torque[i]*DA_MAX_VOLT);
	}

	OutputForce = force;

	for(i=0;i<NUM_ACTUATOR-4;i++) // Just control X, Y, Z motor
		long AioRet = CAioCLI::AioSingleAoEx ( AioId , i , outputVolt[i] );
}
void FiveAxisIOModule::StopAllMotor()
{
	int i;
	for(i=0;i<NUM_ACTUATOR-3;i++)
		long AioRet = CAioCLI::AioSingleAoEx ( AioId , i , 0.0 );
}
void FiveAxisIOModule::OutputOneMotor(short m_iMotorNumber, double OutputForce)
{
	double force;
	float outputVolt;
	double max_force[NUM_LINEAR_ACTUATOR] = {MAX_FORCE_X,MAX_FORCE_Y1,MAX_FORCE_Y2,MAX_FORCE_Z};
	double max_torque[NUM_ROTATION_ACTUATOR] = {MAX_TORQUE_C,MAX_TORQUE_A1,MAX_TORQUE_A2};

	force = OutputForce;
	if (m_iMotorNumber< NUM_LINEAR_ACTUATOR)
	{
		if (force>max_force[m_iMotorNumber])
		{
			force = max_force[m_iMotorNumber];
		} 
		else
		{
			if (force< -max_force[m_iMotorNumber])
			{
				force = -max_force[m_iMotorNumber];
			} 
		}
		outputVolt = static_cast<float>(force/max_force[m_iMotorNumber]*DA_MAX_VOLT);
	} 
	else
	{
		if(force> max_torque[m_iMotorNumber-NUM_LINEAR_ACTUATOR])
			force= max_torque[m_iMotorNumber-NUM_LINEAR_ACTUATOR];
		else if(force < -max_torque[m_iMotorNumber-NUM_LINEAR_ACTUATOR])
			force = -max_torque[m_iMotorNumber-NUM_LINEAR_ACTUATOR];
		outputVolt = static_cast<float>(force/max_torque[m_iMotorNumber-NUM_LINEAR_ACTUATOR]*DA_MAX_VOLT);
	}

	OutputForce = force;
	long AioRet = CAioCLI::AioSingleAoEx ( AioId , m_iMotorNumber , outputVolt );
}
void FiveAxisIOModule::ResetCounterToOrigin()
{
	 System::String^ ResetCounterToOriginError;
	 StringBuilder	^ErrorString;
	 short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};

	CntRet = CCntCLI::CntZeroClearCount ( CntId , ChNo , NUM_COUNTER );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		ResetCounterToOriginError = String::Format("CntZeroClearCount= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
	unsigned long PresetData[NUM_COUNTER];
	double pos_pre[NUM_COUNTER] = {INIT_POS_X,INIT_POS_Y1,INIT_POS_Y2,INIT_POS_Z,INIT_ANGLE_C,INIT_ANGLE_A1,INIT_ANGLE_A2};
	double resonate[NUM_COUNTER] = {RESONATE_LINER_ENC_X,RESONATE_LINER_ENC_Y1,RESONATE_LINER_ENC_Y2,RESONATE_LINER_ENC_Z,RESONATE_ROTATION_ENC_C,RESONATE_ROTATION_ENC_A1,RESONATE_ROTATION_ENC_A2};
	long dir[NUM_COUNTER]={1,1,1,1,1,1,1};
	for(unsigned int i=0;i<NUM_COUNTER;i++){
		pos_pre[i] = pos_pre[i]*dir[i]/resonate[i];
		PresetData[i] = static_cast<unsigned long>(pos_pre[i]);
	}
	CntRet = CCntCLI::CntPreset( CntId , ChNo , NUM_COUNTER , PresetData );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		ResetCounterToOriginError =ResetCounterToOriginError+  String::Format("ResetCounterToOriginError= {0:d}: {1}",CntRet,ErrorString);
		return;
	}
}

void FiveAxisIOModule::StartCounter(System::IntPtr pTimer, System::String^ &StartCounterError, int m_hwnd)
	{
		short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};
		StringBuilder	^ErrorString;


		CntRet = CCntCLI::CntStartCount ( CntId , ChNo , NUM_COUNTER );
		if (CntRet != 0){
			CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
			StartCounterError = String::Format("CntStopCount= {0:d}: {1}",CntRet,StartCounterError);
			return;
		}
		CntRet = CCntCLI::CntZeroClearCount ( CntId , ChNo , NUM_COUNTER );
		if (CntRet != 0){
			CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
			StartCounterError = String::Format("CntZeroClearCount= {0:d}: {1}",CntRet,StartCounterError);
			return;
		}
		unsigned long PresetData[NUM_COUNTER];
		double pos_pre[NUM_COUNTER] = {INIT_POS_X,INIT_POS_Y1,INIT_POS_Y2,INIT_POS_Z,INIT_ANGLE_C,INIT_ANGLE_A1,INIT_ANGLE_A2};
		double resonate[NUM_COUNTER] = {RESONATE_LINER_ENC_X,RESONATE_LINER_ENC_Y1,RESONATE_LINER_ENC_Y2,RESONATE_LINER_ENC_Z,RESONATE_ROTATION_ENC_C,RESONATE_ROTATION_ENC_A1,RESONATE_ROTATION_ENC_A2};
		long dir[NUM_COUNTER]={1,1,1,1,1,1,1};
		for(unsigned int i=0;i<NUM_COUNTER;i++){
			pos_pre[i] = pos_pre[i]*dir[i]/resonate[i];
			PresetData[i] = static_cast<unsigned long>(pos_pre[i]);
		}
		CntRet = CCntCLI::CntPreset( CntId , ChNo , NUM_COUNTER , PresetData );
		if (CntRet != 0){
			CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
			StartCounterError = String::Format("CntPreset= {0:d}: {1}",CntRet,StartCounterError);
			return;
		}

// 		// reset power tester
// 				CntRet = CCntCLI::CntOutputDOBit ( CntId , 1 , 0 , 1 );
// 				System::Threading::Thread::Sleep(200);
// 				CntRet = CCntCLI::CntOutputDOBit ( CntId , 1 , 0 , 0 );
//		pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
		CntRet	= CCntCLI::CntTimerCallbackProc(CntId, pTimer, nullptr);
		CntRet = CCntCLI::CntNotifyTimer ( CntId , static_cast<unsigned long>(SAMPLING_TIME*1000),0);
// 		// start power tester
// 		CntRet = CCntCLI::CntOutputDOBit ( CntId , 0 , 0 , 1 );

	}
void FiveAxisIOModule::StopCounter(System::String^ &StopCounterError)
{
	short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};
	StringBuilder	^ErrorString;

//	CntRet = CCntCLI::CntOutputDOBit ( CntId , 0 , 0 , 0 );
	CntRet = CCntCLI::CntStopNotifyTimer ( CntId );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		StopCounterError = String::Format("CntStopNotifyTimer= {0:d}: {1}",CntRet,ErrorString);
		return;
	}

	// Stoping counter board and DA board
	CntRet = CCntCLI::CntStopCount ( CntId , ChNo , NUM_COUNTER );
	if (CntRet != 0){
		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
		StopCounterError = String::Format("CntStopCount= {0:d}: {1}",CntRet,ErrorString);
		return;
	}


// 	// Stoping counter board and DA board
// 	CntRet = CCntCLI::CntStopCount ( CntId , ChNo , NUM_COUNTER );
// 	if (CntRet != 0){
// 		CntRet2 = CCntCLI::CntGetErrorString(CntRet, ErrorString);
// 		StopCounterError = String::Format("CntStopCount= {0:d}: {1}",CntRet,ErrorString);
// 		return;
// 	}
}
vector<double> FiveAxisIOModule::GetAbsPosition()
{
	vector<double> vec_AbsPosition;
	vec_AbsPosition.resize(NUM_COUNTER);
	vec_AbsPosition.clear();
	vec_AbsPosition(0)= 0.5;
	vec_AbsPosition(1)= 1.5;
	return vec_AbsPosition;
}
vector<double> FiveAxisIOModule::GetAbsolutePosition()
{
	int i;
	vector<long> cnt(NUM_COUNTER);
	unsigned long CntDat[NUM_COUNTER];

	CNCPOSITION  vec_CNCPosition;

	static vector<double> oldpos,oldvel;
	long CntRet,dir[NUM_COUNTER]={1,1,1,-1,-1,1,1};
	
	short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};
//	double cut[NUM_COUNTER] = {CUTOFF_FREC_X,CUTOFF_FREC_Y1,CUTOFF_FREC_Y2,CUTOFF_FREC_Z,CUTOFF_FREC_C,CUTOFF_FREC_A1,CUTOFF_FREC_A2};
	
	// reading values from encorder
	CntRet = CCntCLI::CntReadCount ( CntId , ChNo , NUM_COUNTER , CntDat );

	// convert right-hand system
	for(i=0;i<NUM_COUNTER;i++)
		cnt(i) = static_cast<long>(CntDat[i] * dir[i]);

	// convert position(mm)
	vec_AbsolutePosition(0) = cnt(3) * RESONATE_LINER_ENC_X;//  3 Axis X
	vec_AbsolutePosition(1) = cnt(4) * RESONATE_LINER_ENC_Y1;// 3 Axis Y
	vec_AbsolutePosition(2) = cnt(2) * RESONATE_LINER_ENC_Y2;
	vec_AbsolutePosition(3) = cnt(5) * RESONATE_LINER_ENC_Z;//  3 Axis Z
	// convert position(rad
	vec_AbsolutePosition(4) = cnt(4) * RESONATE_ROTATION_ENC_C;// Wrong in the direct of motor C
	vec_AbsolutePosition(5) = cnt(5) * RESONATE_ROTATION_ENC_A1;
	vec_AbsolutePosition(6) = cnt(6) * RESONATE_ROTATION_ENC_A2;

// 	const DWORD ALL_CntStart=8388607;	// Chiのカウントスタート値 2^23
// #define	ENC2RAD	(2.0*PI/(500.0*4.8*4.0))/
// 	#define	CNT_ZERO (8388607.0)
// 	// convert position(mm)
// 	vec_CNCPosition.X = cnt(0) * RESONATE_LINER_ENC_X;
// 	vec_CNCPosition.Y1 = cnt(1) * RESONATE_LINER_ENC_Y1;
// 	vec_CNCPosition.Y2= cnt(2) * RESONATE_LINER_ENC_Y2;
// 	vec_CNCPosition.Z = cnt(3) * RESONATE_LINER_ENC_Z;
// 	// convert position(rad
// 	vec_CNCPosition.C = cnt(4) * RESONATE_ROTATION_ENC_C;
// 	vec_CNCPosition.A1= cnt(5) * RESONATE_ROTATION_ENC_A1;
// 	vec_CNCPosition.A2 = cnt(6) * RESONATE_ROTATION_ENC_A2;
// 
// 	vec_CNCPosition.Y = vec_CNCPosition.Y1;
// 	vec_CNCPosition.A = vec_CNCPosition.A1;


	return vec_AbsolutePosition;// vec_AbsolutePosition  vec_CNCPosition
}
vector<double> FiveAxisIOModule::GetMachiningPosition()
{
	return GetAbsolutePosition() - vec_PartOrigin;// vec_AbsolutePosition  vec_CNCPosition
}
void FiveAxisIOModule::SetPartOrigin()
{
	vec_PartOrigin = GetAbsolutePosition();
}
// void FiveAxisIOModule::StopOneMotor(short AioId,short m_iMotorNumber)
// {
// 	long AioRet = CAioCLI::AioSingleAoEx ( AioId , m_iMotorNumber , 0.0 );
// }

