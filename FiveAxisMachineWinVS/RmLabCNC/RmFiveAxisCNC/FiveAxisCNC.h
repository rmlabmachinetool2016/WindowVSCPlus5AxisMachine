#ifndef _FIVEAXISCNC_H_
#define _FIVEAXISCNC_H_
#pragma once
#include "StdAfx.h"
#include "FiveAxisControlModule.h"
#include "FiveAxisIOModule.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
using namespace boost::numeric::ublas;
using namespace System::IO;
namespace RmLabCNC{
	ref struct CNCPOSITION
	{
	public:
		double X;
		double Y;
		double Y1;
		double Y2;
		double Z;
		double C;
		double A;
		double A1;
		double A2;
	};
	//x y1 y2 z c a1 a2	::	x y1 z c a1
	//0 1  2  3 4 5  6	::	0 1  2 3 4
	static const short LINEAR_MOTOR_X					= 0;					// Control by AIO port 0
	static const short LINEAR_MOTOR_Y1					= 1;					// Control by AIO port 1
	static const short LINEAR_MOTOR_Y2					= 2;					// Control by AIO port 2
	static const short LINEAR_MOTOR_Z					= 3;					// Control by AIO port 3
	static const short LINEAR_MOTOR_C					= 4;					// Control by AIO port 4
	static const short LINEAR_MOTOR_A1					= 5;					// Control by AIO port 5
	static const short LINEAR_MOTOR_A2					= 6;					// Control by AIO port 6
	static const double DOUBLE_TOLERANCE                = 0.000001;
	static const double PI = 3.1415926535897932384626433832795;

	static const short NUMBERAXIS					= 5;
	static const short MTSIZE					= 4;
	static const short SLMCCSIZE				= 3;
	static const short NUM_COUNTER			= 7;
	static const short NUM_DIRECT			= 2;
	static const short NUM_PEAK			= 10;
	static const short NUM_SINFRIC_DATA			= 20;
	static const short NUM_ECCENTRICFRIC_DATA			= 4;
	static const short NUM_STRIBECKFRIC_DATA			= 5;

	static const short EIGHTCURVE = 1;
	static const short TRIFOLIUM = 2;
	static const short LISSAJOUS = 3;
	static const short TUNINGPROCESS = 4;
	static const short CURVE3D = 5;
	static const short MAXCONTROLFORCE = 36;// For Five Axis Case 500; For 3 axis case// N

	static identity_matrix<double> mt_Identity;
	matrix<double> mt_Test(5,5);
	static matrix<double>
		matrix_temporary,
		matrix_weight_M,matrix_inverse_weight_M,matrix_estimated_weight_M,matrix_inverse_estimated_weight_M,
		matrix_previous_rotation_RT,matrix_rotation_RT,matrix_rotation_R,
		matrix_previous_rotation_dotRT,matrix_rotation_dotRT,
		matrix_rotation_ddotRT,
		matrix_TC_gain_Kp,matrix_TC_gain_Kd,
		matrix_SLMCC_gain_lambda,matrix_SLMCC_gain_A,matrix_SLMCC_sign_L,
		matrix_disturbance_gain_Kd,
		matrix_sign_real_velocity_dotq,
		matrix_viscous_friction_c,matrix_estimated_viscous_friction_c,matrix_nominal_viscous_friction_cn,
		matrix_number_peak,matrix_peak_height,matrix_peak_position,matrix_peak_width,
		matrix_sinusoidal_friction,matrix_stribeck_friction,

		mt_M, mt_Kc, mt_Fbr,
		 
		mt_Translate,mt_Ie,mt_rotR, mt_rotR_1, mt_rotR_2,
		mt_rotRT, mt_rotRT_1, mt_rotRT_2,mt_Kvw,mt_Kpw,mt_Kiw,mt_Kvl,mt_Kpl,mt_Kil,
		mt_MachineCordinate, mt_ToolCordinate, mt_WorkPieceCordinate,
		mt_ContourWorkPieceCordinate,mt_ContourMachineCordinate,
		
		mt_NextrotR, mt_NextrotR_1;
	static vector<double> 
		vector_temporary,vector_switching_force,vector_control_force_fu,vector_estimated_control_force_fu,
		vector_real_position_q,
		vector_previous_real_position_q,vector_real_velocity_dotq,

		vector_desired_position_qd,vector_previous_desired_position_qd,
		vector_desired_velocity_dotqd,vector_previous_desired_velocity_dotqd,
		vector_desired_acceleration_ddotqd,
		vector_SLMCC_gain_k,
		vector_tracking_error_ew,vector_tracking_error_dotew,
		vector_contour_error_el,vector_contour_error_dotel,
		vector_estimated_friction_ff, 
		vector_coulomb_friction_fcl,vector_estimated_coulomb_friction_fcl,vector_nominal_coulomb_friction_fncl,

		vector_gravitational_force_g,vector_estimated_gravitational_force_g,
		vector_disturbance_estimator_de,vector_estimated_disturbance_d,
		vector_sliding_surface_s,

		vector_next_desired_position_data,


		vec_el,vec_el_1,vec_el_2,vec_ew,vec_ew_1,vec_ew_2,
		 vec_Pre_refr_1,vec_Pre_refr, vec_refr,vec_refr_1,vec_refr_2,
		 vec_Pre_realx, vec_Pre_realx_1, vec_realx,vec_realx_1,vec_realx_2,
		 vec_OutputControl,vec_PredictedOutput, vec_Fc,
		 vec_KcFW, vec_FcFW, vec_FbrFW, vec_FvsFW, vec_KcBW, vec_FcBW, vec_FbrBW, vec_FvsBW,

		 vec_CNCVMAX, vec_CNCPMAX,
		 vec_Nextrefr_1,vec_Nextrefr_2,
		 vec_VelChange,vec_StartAcc,vec_EndAcc,vec_NextStartAcc,vec_NextEndAcc;
// void InitStaticVariable()
// 	{
// 		mt_Identity.resize(MTSIZE);
// 
// 		mt_M.resize(NUM_COUNTER,NUM_COUNTER);mt_Kc.resize(NUM_COUNTER,NUM_COUNTER);
// 		mt_Fc.resize(NUM_COUNTER,NUM_COUNTER);mt_Fbr.resize(NUM_COUNTER,NUM_COUNTER);
// 
// 		mt_KcFW.resize(NUM_COUNTER,NUM_DIRECT);mt_FcFW.resize(NUM_COUNTER,NUM_DIRECT);
// 		mt_FbrFW.resize(NUM_COUNTER,NUM_DIRECT);mt_FvsFW.resize(NUM_COUNTER,NUM_DIRECT);
// 		mt_KcBW.resize(NUM_COUNTER,NUM_DIRECT);mt_FcBW.resize(NUM_COUNTER,NUM_DIRECT);
// 		mt_FbrBW.resize(NUM_COUNTER,NUM_DIRECT);mt_FvsBW.resize(NUM_COUNTER,NUM_DIRECT);
// 
// 		mt_Translate.resize(MTSIZE,MTSIZE);
// 		mt_rotR.resize(MTSIZE,MTSIZE);mt_rotR_1.resize(MTSIZE,MTSIZE);
// 		mt_rotR_2.resize(MTSIZE,MTSIZE);
// 		mt_Kvw.resize(NUM_COUNTER,NUM_COUNTER);mt_Kpw.resize(NUM_COUNTER,NUM_COUNTER);mt_Kiw.resize(NUM_COUNTER,NUM_COUNTER);
// 		mt_Kvl.resize(NUM_COUNTER,NUM_COUNTER);mt_Kpl.resize(NUM_COUNTER,NUM_COUNTER);mt_Kil.resize(NUM_COUNTER,NUM_COUNTER);
// 
// 
// 		mt_MachineCordinate.resize(MTSIZE,MTSIZE);
// 		mt_ToolCordinate.resize(MTSIZE,MTSIZE);
// 		//	mt_ToolWorkPieceRelation.resize(MTSIZE,MTSIZE);
// 		mt_WorkPieceCordinate.resize(MTSIZE,MTSIZE);
// 		mt_ContourWorkPieceCordinate.resize(MTSIZE,MTSIZE);
// 		mt_ContourMachineCordinate.resize(MTSIZE,MTSIZE);
// 
// 		mt_M.clear();mt_Kc.clear();mt_Fc.clear();mt_Fbr.clear();
// 		mt_KcFW.clear();mt_FcFW.clear();mt_FbrFW.clear();mt_FvsFW.clear();
// 		mt_KcBW.clear();mt_FcFW.clear();mt_FbrBW.clear();mt_FvsBW.clear();
// 		mt_Translate.clear();
// 		mt_rotR.clear();mt_rotR_1.clear();mt_rotR_2.clear();
// 
// 		vec_el.resize(NUM_COUNTER);vec_el_1.resize(NUM_COUNTER);vec_el_2.resize(NUM_COUNTER);
// 		vec_ew.resize(NUM_COUNTER);vec_ew_1.resize(NUM_COUNTER);vec_ew_2.resize(NUM_COUNTER);
// 		vec_Pre_refr.resize(NUM_COUNTER);vec_Pre_refr_1.resize(NUM_COUNTER);
// 		vec_refr.resize(NUM_COUNTER);vec_refr_1.resize(NUM_COUNTER);vec_refr_2.resize(NUM_COUNTER);
// 		vec_Pre_realx.resize(NUM_COUNTER); vec_Pre_realx_1.resize(NUM_COUNTER);
// 		vec_realx.resize(NUM_COUNTER);vec_realx_1.resize(NUM_COUNTER);vec_realx_2.resize(NUM_COUNTER);
// 		vec_OutputControl.resize(NUM_COUNTER);vec_PredictedOutput.resize(NUM_COUNTER);
// 
// 		vec_AbsolutePosition.resize(NUM_COUNTER);
// 		vec_AbsolutePosition.clear();
// 
// 		vec_el.clear();vec_el_1.clear();vec_el_2.clear();vec_ew.clear();vec_ew_1.clear();vec_ew_2.clear();
// 		vec_Pre_refr.clear();vec_Pre_refr_1.clear();vec_refr.clear();vec_refr_1.clear();vec_refr_2.clear();
// 		vec_Pre_realx.clear();vec_Pre_realx_1.clear();vec_realx.clear();vec_realx_1.clear();vec_realx_2.clear();
// 		vec_OutputControl.clear();vec_PredictedOutput.clear();
// 
// 	}

// 	RMMatrix  mt_M,mt_C,mt_rotR, mt_rotR_1, mt_rotR_2,mt_rotRT, mt_rotRT_1, mt_rotRT_2,
// 		mt_Kvl,mt_Kpl,mt_Kil,mt_KvlBest,mt_KplBest,
// 		mt_Kvw,mt_Kpw,mt_Translate,mt_Ie,mt_Identity,
// 		mt_XcFrictionModel,mt_YcFrictionModel;
// 	// Create two vectors of length N
// 	RMVector vec_refr,vec_refr_1,vec_refr_2,
// 		vec_Pre_refr,vec_Pre_refr_1,vec_Pre_refr_2,
// 		vec_realx,vec_realx_1,vec_realx_2, vec_realxRef,
// 		vec_Pre_realx,vec_Pre_realx_1,vec_Pre_realx_2,vec_FrictionForce,
// 		vec_eSup,vec_errorVoltSup,vec_el,vec_elSum,vec_el_1,vec_el_2,vec_ew,vec_ew_1,vec_ew_2,
// 		vec_u, vec_FrictionModelPointXc,vec_FrictionModelPointYc,vec_Tangential,vec_AccVoltSup,vec_VelVoltSup;
ref class FiveAxisCNC
{
public:
	FiveAxisCNC(void);
	~FiveAxisCNC(void);
//	 matrix<double> *mt_M, *mt_Kc;
//	 mt_M = new matrix<double>; //*mt_M, *mt_Kc;

	FiveAxisControlModule ControlModule;
	FiveAxisIOModule IOModule;
	CNCPOSITION m_CNCRefPos,m_CNCPosManualStep,m_CNCTableOrigin,m_CNCRealPos,
				m_CNCStartRefVel,m_NextCNCStartRefVel,m_CNCEndRefVel,m_NextCNCEndRefVel ;
	double SAMPLING_TIME;
// 	m_fexpTnow = m_fexpTnow+ m_fSampTime;
// 	if (m_fexpTnow>(m_fexpRun
	double m_fexpTnow,m_fexpRunT,m_fexpRunTPre,m_fexpTnowReal,m_fSampTime,
		   m_fSampTimeRef, m_fAbso_ec, m_fexpTnowCounter,
		   m_fNextexpRunT,
		   m_fAccFirstTime,m_fNextAccFirstTime, m_fAccEndTime,
		   m_fCNCPosOneX,m_fCNCPosTwoX,m_fCNCPosThreeX,
		   m_fCNCPosOneY,m_fCNCPosTwoY,m_fCNCPosThreeY,
		   m_fCNCPosOneZ,m_fCNCPosTwoZ,m_fCNCPosThreeZ,
		   m_fCNCStartVelX, m_fCNCMidVelX,  m_fCNCEndVelX,  m_fCNCStartAccX,  m_fCNCEndAccX,
		   m_fCNCStartVelY, m_fCNCMidVelY,  m_fCNCEndVelY,   m_fCNCStartAccY,  m_fCNCEndAccY,
		   m_fCNCStartVelZ, m_fCNCMidVelZ,  m_fCNCEndVelZ,   m_fCNCStartAccZ,  m_fCNCEndAccZ,


	     m_fCNCAngleOne,m_fCNCAngleTwo,m_fCNCAngleThree,
		m_fCNCStartAngleVel, m_fCNCMidAngleVel, m_fCNCEndAngleVel,  m_fCNCStartAngleAcc,  m_fCNCEndAngleAcc,

		 m_fNextCNCPosOneX, m_fNextCNCPosTwoX, m_fNextCNCPosThreeX, 
		 m_fNextCNCPosOneY, m_fNextCNCPosTwoY, m_fNextCNCPosThreeY,
		 m_fNextCNCPosOneZ, m_fNextCNCPosTwoZ, m_fNextCNCPosThreeZ,
		 m_fNextCNCStartVelX,  m_fNextCNCMidVelX,   m_fNextCNCEndVelX,  m_fNextCNCStartAccX,   m_fNextCNCEndAccX,
		 m_fNextCNCStartVelY,  m_fNextCNCMidVelY,   m_fNextCNCEndVelY,   m_fNextCNCStartAccY,   m_fNextCNCEndAccY,
		 m_fNextCNCStartVelZ,  m_fNextCNCMidVelZ,   m_fNextCNCEndVelZ,   m_fNextCNCStartAccZ,   m_fNextCNCEndAccZ,

		 m_fNextCNCAngleOne, m_fNextCNCAngleTwo, m_fNextCNCAngleThree,
		 m_fNextCNCStartAngleVel,  m_fNextCNCMidAngleVel, m_fNExtCNCEndAngleVel,    m_fNextCNCStartAngleAcc,   m_fNextCNCEndAngleAcc;

	bool m_bAutoCornerSmooth,m_bMathematicalCurve;
/* Read write file variable */
	FileStream^ DataAnalysisFileStream;
	BinaryWriter^ DataAnalysisBinaryWriter;
	StreamReader^ GcodeFileStream;
	bool m_bGcodeFINISH,m_bPathFinish, m_bOriginSetup,
		m_bNextGcodeFINISH;
	double m_fCNCStartX,m_fCNCStartY,m_fCNCStartZ,
		m_fNextCNCStartX,m_fNextCNCStartY,m_fNextCNCStartZ;

	double m_fCNCEndX,m_fCNCEndY,m_fCNCEndZ,m_fCNCEndA,m_fCNCEndC,
		   m_fNextCNCEndX,m_fNextCNCEndY,m_fNextCNCEndZ,m_fNextCNCEndA,m_fNextCNCEndC,
		   m_fCNCI,m_fCNCJ,m_fCNCRadius, m_fGcodeF, m_fCNCFeedRate,
		   m_fNextCNCI,m_fNextCNCJ,m_fNextCNCRadius, m_fNextGcodeF, m_fNextCNCFeedRate,
		   m_fCNCVMIN,m_fCNCPH, m_fCNCPE, m_fNextCNCPH, m_fNextCNCPE,
		   m_fCNCPHX, m_fCNCPEX, m_fNextCNCPHX, m_fNextCNCPEX,
		   m_fCNCPHY, m_fCNCPEY, m_fNextCNCPHY, m_fNextCNCPEY;


	double m_fLengthX,m_fLengthY,m_fLengthZ, m_fNextLengthX,m_fNextLengthY,m_fNextLengthZ,m_fLengthMovement,m_fNextLengthMovement;
	double m_fCNCStartAngle,m_fCNCEndAngle,m_fOmg, m_fNextCNCStartAngle,m_fNextCNCEndAngle,m_fNextOmg,
			m_fStartddotOmg,m_fEndddotOmg, m_fNextStartddotOmg,
			m_fStartddotAcc,m_fEndddotAcc, m_fNextStartddotAcc,
			m_fMaxAcc;
	unsigned int m_iMoveType,m_iNextMoveType, m_iGcodeM, m_iNextGcodeM,
		m_iSelectedControllerType,m_iSelectedFrictionModel,m_iSelectedDisturbanceObserver,m_iSelectedCharacteristicCurveModel;

	// Mathematical curve data
	unsigned int m_iMathCurveNumber, m_iTuningMotor;
	double m_fMath_a,m_fMath_n,m_fMath_c,m_fMath_b,m_fMath_Time,m_fMath_Repeat,m_fMath_Theta,
		m_fMath_Thetadot,m_fMath_r,m_fMath_w;
	double m_frotR_Pre_theta,m_frotR_theta,m_frotR_theta_1;

	//end  Mathematical curve data

	System::String^ m_strDebugString;
//
	// Check about finish machining contour in Gcode file 
	bool Finish(void);
	void SetMachineOrigin(void);
	
	void CounterTimerCallback(void);
	vector<double>  GetCurrentReference(void);
	vector<double>  GetRealPosition(void);
	void UpdateRealPosition(void);
	void SetTableOrigin(vector<double> Vec_realPosition);
	void SetRealPosition(vector<double> Vec_realPosition);
	void SetRefPosition(vector<double> Vec_refPosition);
	// Get next reference point at counter timer call for feedback control
	void GetNextRefPoint(void);
	// Continue to read next ref contour in Gcode file 
	void IndependentControl2D(void);
	void ContouringControl3DFiveAxis(void);
	void ThreeAxisMachineController(void);
	void ThreeAxisMachineControllerInRegulation(void);
	void IndependentControl3DFiveAxis(void);
	void ThreeAxisMachineSlidingModeContouringController(void);
	void ThreeAxisMachinePDTrackingController(void);
	void ThreeAxisMachinePDContouringController(void);
	
	void EstimateFrictionValue(void);
	double gaussian(double x, double pos, double wid);

	void CoefficientFilter(void);
	void ResetReferenceData(void);
	void ResetRealData(void);
	void CalulateReferenceData(void);
	void CalculateRealPostisionData(void);

	void InitVariableName(System::String^ strName,double fValue);
	void InitControlVariable();
	void rmsscanf(System::String^ scanString,System::String^ &strName,double &fValue);
	void InitGlobalVariable(System::String^ FileGlobalVariable);
	void InitControllerParameters(System::String^ FileControllerParameters);
	void OpenGcodeFile(System::String^ FileGcodeProgram);
	void OpenBinaryFile(System::String^ FileDataAnalysis);
	void SaveDataToBinaryFile();
	void GetNextRealtimeRef();  //*********************************Real Time********************//
	double GetGcodeVariableValue(System::String^  gcodeString,unsigned int &indexStart);
	bool AnalyseGcodeLine(System::String^ strLineRead) ;
	bool AnalyseNextGcodeLine(System::String^ strGcodeLine); 
	void GetNextGCodeLine(void);
	bool OutOfLimitedAcceleration(void) ;
	double CalculateAccEndTime(void);
	double CalculateNextAccFirstTime(void) ;
	double Rmsign(double refNumber); 
	void GetAccelerationTime(void); 
	void GetNextPointRefInRegulation();
	void GetNextPointRefInGCodePath();		

	void SendOutputControl();
	void SendOutputToVirtualSystem();
	System::String^ DebugDataString();
	void SpinStart(double OutputForce);
    void SpinStop();

	void CloseBinaryFile();
	void CloseGcodeFile();
	void StopFiveAxisCNC();

	double GetStaticVariable(System::String^ strName);
	double TestMatrix(int mt_index);
	void SetMatrix(int mt_index, double mt_value);

};
}
#endif