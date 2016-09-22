#ifndef _FIVEAXISCNC_H_
#define _FIVEAXISCNC_H_
#pragma once
#include "StdAfx.h"
#include "FiveAxisControlModule.h"
#include "FiveAxisIOModule.h"
#include "SimulationModel.h"
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

	static const short NUMBERAXIS		 = 7;
	static const short MTSIZE			 = 4;
	static const short NUM_COUNTER		 = 7;
	static const short PEAKLIMIT		 = 8;
	static const short GMSELEMENT		 = 10;
	static const short NFRICTIONDATA	 = 4;
	static const short NUM_DIRECT		 = 2;
	static const short NUMSTATE		 = 3; // Kalman filter
	static const short NUMMEASURED		 = 2; // Kalman filter

	static const short EIGHTCURVE = 1;
	static const short TRIFOLIUM = 2;
	static const short LISSAJOUS = 3;
	static const short TUNINGPROCESS = 4;

	static const double MAX_FORCE_X					= 36.0;					// N
	static const double MAX_FORCE_Y1				= 36.0;					// N
	static const double MAX_FORCE_Y2				= 36.0;					// N
	static const double MAX_FORCE_Z					= 36.0;					// N
	static const double MAX_TORQUE_C				= 2.0;					// N*m
	static const double MAX_TORQUE_A1				= 5.0;					// N*m
	static const double MAX_TORQUE_A2				= 5.0;					// N*m

	static identity_matrix<double> mt_Identity;
	matrix<double> mt_Test(5,5);
	static matrix<double> mt_M,mt_1_M, mt_Kc, mt_Fbr,
		 
		mt_Translate,mt_Ie,mt_rotR, mt_rotR_1, mt_rotR_2,
		mt_rotRT, mt_rotRT_1, mt_rotRT_2,mt_Kvw,mt_Kpw,mt_Kiw,mt_Kvl,mt_Kpl,mt_Kil,
		mt_MachineCordinate, mt_ToolCordinate, mt_WorkPieceCordinate,
		mt_ContourWorkPieceCordinate,mt_ContourMachineCordinate,
		
		mt_NextrotR, mt_NextrotR_1,
		   // Disturbance,Friction, observed disturbance variable
		mt_DOBKd,mt_DOBKv,mt_PeakHeight,mt_PeakPos,mt_PeakWid,
		mt_StaticFricData,mt_GMSForceF,mt_GMSForceW,mt_GMSStiffK,

		mt_KalmanIX,mt_KalmanKobsX,mt_KalmanAX,mt_KalmanCX   //Kalman filter
		;
	static vector<double> vec_el,vec_el_1,vec_el_2,vec_ew,vec_ew_1,vec_ew_2,
		 vec_Pre_refr_1,vec_Pre_refr, vec_refr,vec_refr_1,vec_refr_2,
		 vec_Pre_realx, vec_Pre_realx_1, vec_realx,vec_realx_1,vec_Selectx_1,vec_realx_2,
		 vec_OutputControl,vec_PredictedOutput, vec_Fc,
		 vec_KcFW, vec_FcFW, vec_FbrFW, vec_FvsFW, vec_KcBW, vec_FcBW, vec_FbrBW, vec_FvsBW,

		 vec_CNCVMAX, vec_CNCPMAX,
		 vec_Nextrefr_1,vec_Nextrefr_2,
		 vec_VelChange,vec_StartAcc,vec_EndAcc,vec_NextStartAcc,vec_NextEndAcc,
   // Disturbance,Friction, observed disturbance variable
		 vec_PredictedFriction,vec_PredictedDisturb,vec_DOBd,vec_SlidingVel,
		 vec_NominalFc,vec_NominalFv,
		 vec_DOBEstimatedx_1,vec_DOBew_1,vec_NumberPeak,
		 vec_GMSElement,
		 vec_KalmanMesured,vec_KalmanState,vec_KalmanBX   //Kalman filter
		 ;



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
	SimulationModel SimulModule;
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
	unsigned int m_iMoveType,m_iNextMoveType, m_iGcodeM, m_iNextGcodeM;

	// Mathematical curve data
	unsigned int m_iMathCurveNumber, m_iTuningMotor;
	double m_fMath_a,m_fMath_n,m_fMath_c,m_fMath_b,m_fMath_Time,m_fMath_Repeat,m_fMath_Theta,
		m_fMath_Thetadot,m_fMath_r,m_fMath_w;
	double m_frotR_Pre_theta,m_frotR_theta,m_frotR_theta_1;

	//end  Mathematical curve data
	// Disturbance,Friction, observed disturbance variable
	unsigned int m_iFrictionType, m_iDOBModel;
	//end Disturbance,Friction, observed disturbance variable
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
	void IndependentControl3DFiveAxis(void);
	void CoefficientFilter(void);
	// Friction, disturbance calculate
	void CalculatePredictedFriction(void);
	void CalculatePredictedDisturbance(void);
	void UpdateDisturbanceObserverParameter(void);
	void CalculateDisturbanceObserve(void);
	double gaussian(double x, double pos, double wid);

	void ResetReferenceData(void);
	void ResetRealData(void);
	void CalulateReferenceData(void);
	void CalculateRealPostisionData(void);

	void InitVariableName(System::String^ strName,double fValue);
	void InitNonlinearFrictionData(System::String^ fileNLF);
	void InitGMSModelData(System::String^ fileGMS);
	void InitKalmanFilterModelData(System::String^ fileKalmanFilter);
	void InitControlVariable();
	void rmsscanf(System::String^ scanString,System::String^ &strName,double &fValue);
	void InitGlobalVariable(System::String^ FileGlobalVariable,System::String^ FileNonlinearFriction,System::String^ FileGMSModel);
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
	vector<double> GetOutputControl();

	void CloseBinaryFile();
	void CloseGcodeFile();
	void StopFiveAxisCNC();

	double GetStaticVariable(System::String^ strName);
	double TestMatrix(int mt_index);
	void SetMatrix(int mt_index, double mt_value);

};
}
#endif