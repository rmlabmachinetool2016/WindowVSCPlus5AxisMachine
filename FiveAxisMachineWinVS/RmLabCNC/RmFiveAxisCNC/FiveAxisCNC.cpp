#include "StdAfx.h"
#include "FiveAxisCNC.h"
using namespace System::IO;
using namespace RmLabCNC;
FiveAxisCNC::FiveAxisCNC(void)
{
// SimulationModelParameter setup
// 	mt_Simul1_M.resize(NUMBERAXIS,NUMBERAXIS);
// 
// 	vec_Simulx.resize(NUMBERAXIS);
// 	vec_SimulPrex.resize(NUMBERAXIS);
// 	vec_Simulx_1.resize(NUMBERAXIS);
// 	vec_SimulPrex_1.resize(NUMBERAXIS);
// 	vec_SimulKc.resize(NUMBERAXIS);
// 	vec_SimulFc.resize(NUMBERAXIS);
// 	vec_InertiaForce.resize(NUMBERAXIS);
// 
// 	vec_Simulx.clear();
// 	vec_SimulPrex.clear();
// 	vec_Simulx_1.clear();
// 	vec_SimulPrex_1.clear();
// 	vec_SimulKc.clear();
// 	vec_SimulFc.clear();
// 	vec_InertiaForce.clear();
// 	mt_Simul1_M.clear();


 //   RmLabCNC::InitStaticVariable();
	mt_Identity.resize(MTSIZE);

	mt_M.resize(NUM_COUNTER,NUM_COUNTER);
	mt_1_M.resize(NUM_COUNTER,NUM_COUNTER);
	mt_Kc.resize(NUM_COUNTER,NUM_COUNTER);
	mt_Fbr.resize(NUM_COUNTER,NUM_COUNTER);

	vec_Fc.resize(NUM_COUNTER);
	vec_KcFW.resize(NUM_COUNTER);vec_FcFW.resize(NUM_COUNTER);
	vec_FbrFW.resize(NUM_COUNTER);vec_FvsFW.resize(NUM_COUNTER);
	vec_KcBW.resize(NUM_COUNTER);vec_FcBW.resize(NUM_COUNTER);
	vec_FbrBW.resize(NUM_COUNTER);vec_FvsBW.resize(NUM_COUNTER);
	// Disturbance,Friction, observed disturbance variable
	vec_Selectx_1.resize(NUMBERAXIS);
	vec_PredictedFriction.resize(NUM_COUNTER);
	vec_NominalFc.resize(NUM_COUNTER);
	vec_NominalFv.resize(NUM_COUNTER);

	vec_PredictedDisturb.resize(NUM_COUNTER);
	vec_DOBd.resize(NUM_COUNTER);
	vec_DOBEstimatedx_1.resize(NUM_COUNTER);
	vec_DOBew_1.resize(NUM_COUNTER);
	vec_NumberPeak.resize(NUMBERAXIS*2);
	vec_GMSElement.resize(NUMBERAXIS);
	vec_SlidingVel.resize(NUMBERAXIS);

	vec_KalmanMesured.resize(NUMMEASURED); //Kalman filter
	vec_KalmanState.resize(NUMSTATE);
	vec_KalmanBX.resize(NUMSTATE);
// Matrix setting

	mt_Translate.resize(MTSIZE,MTSIZE);
	mt_rotR.resize(MTSIZE,MTSIZE);mt_rotR_1.resize(MTSIZE,MTSIZE);
	mt_rotR_2.resize(MTSIZE,MTSIZE);
	mt_Kvw.resize(NUM_COUNTER,NUM_COUNTER);mt_Kpw.resize(NUM_COUNTER,NUM_COUNTER);mt_Kiw.resize(NUM_COUNTER,NUM_COUNTER);
	mt_Kvl.resize(NUM_COUNTER,NUM_COUNTER);mt_Kpl.resize(NUM_COUNTER,NUM_COUNTER);mt_Kil.resize(NUM_COUNTER,NUM_COUNTER);



	mt_MachineCordinate.resize(MTSIZE,MTSIZE);
	mt_ToolCordinate.resize(MTSIZE,MTSIZE);
//	mt_ToolWorkPieceRelation.resize(MTSIZE,MTSIZE);
	mt_WorkPieceCordinate.resize(MTSIZE,MTSIZE);
	mt_ContourWorkPieceCordinate.resize(MTSIZE,MTSIZE);
	mt_ContourMachineCordinate.resize(MTSIZE,MTSIZE);
	// Disturbance,Friction, observed disturbance variable
	mt_DOBKd.resize(NUM_COUNTER,NUM_COUNTER);      // Disturbance observer
	mt_DOBKv.resize(NUM_COUNTER,NUM_COUNTER);      // Disturbance observer
    mt_PeakHeight.resize(NUM_COUNTER*2,PEAKLIMIT); // Nonlinear friction
	mt_PeakPos.resize(NUM_COUNTER*2,PEAKLIMIT);
	mt_PeakWid.resize(NUM_COUNTER*2,PEAKLIMIT);
	mt_GMSForceF.resize(NUMBERAXIS,GMSELEMENT);
	mt_GMSForceW.resize(NUMBERAXIS,GMSELEMENT);
	mt_GMSStiffK.resize(NUMBERAXIS,GMSELEMENT);

	mt_StaticFricData.resize(NUM_COUNTER*2,NFRICTIONDATA); // Linear friction

	mt_KalmanIX.resize(NUMSTATE,NUMSTATE); //Kalman filter
	mt_KalmanKobsX.resize(NUMSTATE,NUMMEASURED); 
	mt_KalmanAX.resize(NUMSTATE,NUMSTATE);  
	mt_KalmanCX.resize(NUMMEASURED,NUMSTATE);    // (2,3)


// Confirm Reset value
	mt_M.clear();mt_1_M.clear();mt_Kc.clear();vec_Fc.clear();mt_Fbr.clear();
	vec_KcFW.clear();vec_FcFW.clear();vec_FbrFW.clear();vec_FvsFW.clear();
	vec_KcBW.clear();vec_FcFW.clear();vec_FbrBW.clear();vec_FvsBW.clear();
	mt_Translate.clear();
	mt_rotR.clear();mt_rotR_1.clear();mt_rotR_2.clear();

	vec_el.resize(NUM_COUNTER);vec_el_1.resize(NUM_COUNTER);vec_el_2.resize(NUM_COUNTER);
	vec_ew.resize(NUM_COUNTER);vec_ew_1.resize(NUM_COUNTER);vec_ew_2.resize(NUM_COUNTER);
	vec_Pre_refr.resize(NUM_COUNTER);vec_Pre_refr_1.resize(NUM_COUNTER);
	vec_refr.resize(NUM_COUNTER);vec_refr_1.resize(NUM_COUNTER);vec_refr_2.resize(NUM_COUNTER);
	vec_Pre_realx.resize(NUM_COUNTER); vec_Pre_realx_1.resize(NUM_COUNTER);
	vec_realx.resize(NUM_COUNTER);vec_realx_1.resize(NUM_COUNTER);vec_realx_2.resize(NUM_COUNTER);
	vec_OutputControl.resize(NUM_COUNTER);vec_PredictedOutput.resize(NUM_COUNTER);

	vec_AbsolutePosition.resize(NUM_COUNTER);
	vec_AbsolutePosition.clear();

	vec_el.clear();vec_el_1.clear();vec_el_2.clear();vec_ew.clear();vec_ew_1.clear();vec_ew_2.clear();
	vec_Pre_refr.clear();vec_Pre_refr_1.clear();vec_refr.clear();vec_refr_1.clear();vec_refr_2.clear();
	vec_Pre_realx.clear();vec_Pre_realx_1.clear();vec_realx.clear();vec_realx_1.clear();vec_realx_2.clear();
	vec_OutputControl.clear();vec_PredictedOutput.clear();

	vec_CNCVMAX.resize(NUM_COUNTER);vec_CNCPMAX.resize(NUM_COUNTER);
	vec_CNCVMAX.clear();vec_CNCPMAX.clear();


	vec_Nextrefr_1.resize(NUM_COUNTER);vec_Nextrefr_2.resize(NUM_COUNTER);
	mt_NextrotR.resize(MTSIZE,MTSIZE);mt_NextrotR_1.resize(MTSIZE,MTSIZE);
	vec_VelChange.resize(NUM_COUNTER); vec_StartAcc.resize(NUM_COUNTER);
	vec_EndAcc.resize(NUM_COUNTER);vec_NextStartAcc.resize(NUM_COUNTER);
	vec_NextEndAcc.resize(NUM_COUNTER);
	vec_Nextrefr_1.clear();vec_Nextrefr_2.clear();
	mt_NextrotR.clear();mt_NextrotR_1.clear();
	vec_VelChange.clear();vec_StartAcc.clear();
	vec_EndAcc.clear();vec_NextStartAcc.clear();
	vec_NextEndAcc.clear();

	// Disturbance,Friction, observed disturbance variable
	vec_Selectx_1.clear();
	vec_PredictedFriction.clear();
	vec_NominalFc.clear();
	vec_NominalFv.clear();
	vec_PredictedDisturb.clear();
	vec_DOBd.clear();
	vec_DOBEstimatedx_1.clear();
	vec_DOBew_1.clear();
	vec_NumberPeak.clear();
	mt_DOBKd.clear();
	mt_DOBKv.clear();
	mt_PeakHeight.clear();
	mt_PeakPos.clear();
	mt_PeakWid.clear();
	mt_StaticFricData.clear();

	vec_KalmanMesured.clear(); //Kalman filter
	vec_KalmanState.clear();
	vec_KalmanBX.clear();
	mt_KalmanIX.clear();//Kalman filter
	mt_KalmanKobsX.clear(); 
	mt_KalmanAX.clear();
	mt_KalmanCX.clear();   // (2,3)

	m_bGcodeFINISH = FALSE;
	m_bNextGcodeFINISH = FALSE;
	m_bOriginSetup = FALSE;
//	 mt_M->resize( 1, 5) ;
//	  mt_Kc->resize( 1, 5) ;
// 	  matrix<double> mt_M;//(3, 3),mt_Kc(3, 3);//m (3, 3),m1 (3, 3),m2 (3, 3);
// 	  //	vector<double> RefPosition(3),RealPosition(3);
// 	  if (!FiveAxisCNC) {
// 		  throw gcnew OutOfMemoryException();
// 	  }
}
FiveAxisCNC::~FiveAxisCNC(void)
{
}
bool FiveAxisCNC::Finish(void)
{
	return m_bGcodeFINISH;
}
void FiveAxisCNC::SetMachineOrigin()
{
	IOModule.ResetCounterToOrigin();
	m_bOriginSetup = TRUE;
}
void FiveAxisCNC::CounterTimerCallback()
{
}
vector<double> FiveAxisCNC::GetCurrentReference()
{
	return vec_refr;
}
vector<double> FiveAxisCNC::GetRealPosition()
{
   return vec_realx;
}
void FiveAxisCNC::UpdateRealPosition()
{
	vec_realx = IOModule.GetAbsPosition();
	m_strDebugString = "vec_realx(0)"+ System::Convert::ToString(vec_realx(0));
}
void FiveAxisCNC::SetRealPosition(vector<double> Vec_realPosition)
{
	vec_Pre_realx = vec_realx;
	vec_realx = Vec_realPosition;
	m_CNCRealPos.X = vec_realx(0);
	m_CNCRealPos.Y1 = vec_realx(1);
	m_CNCRealPos.Y2 = vec_realx(2);
	m_CNCRealPos.Y = m_CNCRealPos.Y1;
	m_CNCRealPos.Z = vec_realx(3);
	m_CNCRealPos.C = vec_realx(4);
	m_CNCRealPos.A1 = vec_realx(5);
	m_CNCRealPos.A2 = vec_realx(6);
	m_CNCRealPos.A1 = m_CNCRealPos.A1 ;

// 	vec_Pre_realx = vec_realx;
// 	vec_realx = vec_refr;
//    vec_realx = vec_AbsolutePosition;

// 	vec_refr(0)= 0.4;
// 	vec_refr(1)= 2.3;
// 	vec_realx(0) = Vec_realPosition.X;
// 	vec_realx(1) = Vec_realPosition.Y1;
// 	vec_realx(2) = Vec_realPosition.Y2;
// 	vec_realx(3) = Vec_realPosition.Z;
// 	vec_realx(4) = Vec_realPosition.C;
// 	vec_realx(5) = Vec_realPosition.A1;
// 	vec_realx(6) = Vec_realPosition.A2;
}
void FiveAxisCNC::SetRefPosition(vector<double> Vec_refPosition)
{
	//		theta = m_fOmg*m_fexpTnow;
	vec_Pre_refr =vec_refr;
	vec_refr = Vec_refPosition;
	m_CNCRefPos.X = vec_refr(0);
	m_CNCRefPos.Y1 = vec_refr(1);
	m_CNCRefPos.Y2 = vec_refr(2);
	m_CNCRefPos.Y = m_CNCRefPos.Y1;
	m_CNCRefPos.Z = vec_refr(3);
	m_CNCRefPos.C = vec_refr(4);
	m_CNCRefPos.A1 = vec_refr(5);
	m_CNCRefPos.A2 = vec_refr(6);
	m_CNCRefPos.A  = m_CNCRefPos.A1;

}
void FiveAxisCNC::SetTableOrigin(vector<double> Vec_realPosition)
{
	m_CNCTableOrigin.X = vec_realx(0);
	m_CNCTableOrigin.Y1 = vec_realx(1);
	m_CNCTableOrigin.Y2 = vec_realx(2);
	m_CNCTableOrigin.Z = vec_realx(3);
	m_CNCTableOrigin.C = vec_realx(4);
	m_CNCTableOrigin.A1 = vec_realx(5);
	m_CNCTableOrigin.A2 = vec_realx(6);

}
// Get next reference point at counter timer call for feedback control
void FiveAxisCNC::GetNextRefPoint(void)
{
	//	throw(gcnew System::NotImplementedException);
}
// Continue to read next ref contour in Gcode file 
void FiveAxisCNC::IndependentControl2D(void)
{

}
void FiveAxisCNC::ContouringControl3DFiveAxis(void)
{
	
}
void FiveAxisCNC::IndependentControl3DFiveAxis(void)
{
	int i=0;
	CalulateReferenceData();// Calcule vec_refr1, vec_refr2, 
	CalculateRealPostisionData();// Calcule vec_realx1, vec_realx2, 

	vec_Selectx_1= vec_realx_1;//vec_refr_1;  // Select velocity to calculate friction magnitude

	vec_ew = vec_realx-vec_refr;
	vec_ew_1 = vec_realx_1-vec_refr_1;
//	CoefficientFilter();

	CalculatePredictedFriction();
	CalculatePredictedDisturbance();// Calculate Disturbance first to eliminate previous friction value

	vec_PredictedOutput = prod(mt_M,vec_refr_2)+vec_PredictedFriction+vec_DOBd;
 	vec_OutputControl = vec_PredictedOutput +prod(mt_M,-prod(mt_Kvw,vec_ew_1)-prod(mt_Kpw,vec_ew));
	UpdateDisturbanceObserverParameter();

	vec_OutputControl(3) = -vec_OutputControl(3);
	vec_OutputControl(1) = -vec_OutputControl(1);
// no control motor Y2, A2 
	vec_OutputControl(2) = 0.0;
	vec_OutputControl(6) = 0.0;


	vec_PredictedOutput = vec_PredictedFriction+vec_DOBd;


//   	vec_OutputControl(1) = mt_KalmanKobsX(0,0);
//   	vec_OutputControl(3) = vec_DOBd(0);
// 		prod(mt_M,vec_refr_2-prod(mt_Kvw,vec_ew_1)-prod(mt_Kpw,vec_ew))
//  		+prod(mt_Kc,vec_realx_1);


// 	m_strDebugString = "vec_refr_2(0)"+ System::Convert::ToString(RmLabCNC::vec_refr_2(0))
// 		+"vec_refr_2(1)"+ System::Convert::ToString(RmLabCNC::vec_refr_2(1))
// 		+"vec_realx_1(0)"+ System::Convert::ToString(RmLabCNC::vec_realx_1(0))
// 		+"vec_realx_1(1)"+ System::Convert::ToString(RmLabCNC::vec_realx_1(1));
	double max_force[NUMBERAXIS] = {MAX_FORCE_X,MAX_FORCE_Y1,MAX_FORCE_Y2,MAX_FORCE_Z,MAX_TORQUE_C,MAX_TORQUE_A1,MAX_TORQUE_A2};
	for(i=0;i<NUMBERAXIS;i++)
	{
		if(vec_OutputControl(i) > max_force[i])
			vec_OutputControl(i) = max_force[i];
		else if(vec_OutputControl(i) < -max_force[i])
			vec_OutputControl(i) = -max_force[i];
	}

	


}
void FiveAxisCNC::CoefficientFilter(void)
{
	for (int i=0;i<NUM_COUNTER;i++)
	{
		if (vec_refr_1(i)>0.0)
		{
			mt_Kc(i,i) = vec_KcFW(i);
			vec_Fc(i) = vec_FcFW(i);
		}else
		{
			mt_Kc(i,i) = vec_KcBW(i);
			vec_Fc(i) = - vec_FcBW(i);////*Adtention/////////*
		}
	}

}
double FiveAxisCNC::gaussian(double x, double pos, double wid)
{
	//%  gaussian(x,pos,wid) = gaussian peak centered on pos, half-width=wid
	//%  x may be scalar, vector, or matrix, pos and wid both scalar
	//%  T. C. O'Haver, 1988
	//% Example: gaussian([1 2 3],1,2) gives result [0.5000    1.0000    0.5000]
	return exp(-((x-pos)/(0.6005612*wid))*((x-pos)/(0.6005612*wid)));
}
void FiveAxisCNC::CalculatePredictedFriction(void)
{
	int i=0,j=0;
//	vec_Selectx_1
	switch (m_iFrictionType)
	{
	case 0: // No friction compensation
		for (i=0;i<NUMBERAXIS;i++)
		{
			vec_PredictedFriction(i) = 0;
		}
		break;
	case 1:     // Static friction (Stribek, Couloumb, viscous ) 

//		mt_StaticFricData(NUM_COUNTER*2,NFRICTIONDATA)  (2,4) [(Forward,backward), (Coulomb, Stribek, StribekVelocity, Viscousity)]

		for (i=0; i<NUMBERAXIS; i++)
		{
			if (vec_Selectx_1(i)>=0)
			{
				vec_PredictedFriction(i) =  mt_StaticFricData(i*2,0)+
											(mt_StaticFricData(i*2,1)-mt_StaticFricData(i*2,0))*exp(-mt_StaticFricData(i*2,2)*mt_StaticFricData(i*2,2)*vec_Selectx_1(i)*vec_Selectx_1(i))
											+mt_StaticFricData(i*2,3)*vec_Selectx_1(i);
			} 
			else
			{
				vec_PredictedFriction(i) =  -mt_StaticFricData(i*2+1,0)-
					(mt_StaticFricData(i*2+1,1)-mt_StaticFricData(i*2+1,0))*exp(-mt_StaticFricData(i*2+1,2)*mt_StaticFricData(i*2+1,2)*vec_Selectx_1(i)*vec_Selectx_1(i))
					+mt_StaticFricData(i*2+1,3)*vec_Selectx_1(i);
			}
		}

		break;
	case 2:     // Dynamic friction (General maxwell slip model) 
		for (i=0; i<NUMBERAXIS; i++)
		{
			vec_PredictedFriction(i) = 0.0;
			if (fabs(vec_Selectx_1(i))>vec_SlidingVel(i))// Sliding regime  Use static friction model
			{
				if (vec_Selectx_1(i)>=0)
				{
					vec_PredictedFriction(i) =  mt_StaticFricData(i*2,0)+
						(mt_StaticFricData(i*2,1)-mt_StaticFricData(i*2,0))*exp(-mt_StaticFricData(i*2,2)*mt_StaticFricData(i*2,2)*vec_Selectx_1(i)*vec_Selectx_1(i))
						+mt_StaticFricData(i*2,3)*vec_Selectx_1(i);
				} 
				else
				{
					vec_PredictedFriction(i) =  -mt_StaticFricData(i*2+1,0)-
						(mt_StaticFricData(i*2+1,1)-mt_StaticFricData(i*2+1,0))*exp(-mt_StaticFricData(i*2+1,2)*mt_StaticFricData(i*2+1,2)*vec_Selectx_1(i)*vec_Selectx_1(i))
						+mt_StaticFricData(i*2+1,3)*vec_Selectx_1(i);
				}
			}else  // PreSliding regime  Use General maxwell slip model
			{
				if (vec_Selectx_1(i)>=0)
				{
					for (j=0;j<vec_GMSElement(i);j++)///vec_GMSElement(i) =  (Number element on axis i).
					{
						if (mt_GMSForceF(i,j)<mt_GMSForceW(i,j))
						{
							mt_GMSForceF(i,j) = mt_GMSForceF(i,j)+m_fSampTime*mt_GMSStiffK(i,j)*vec_Selectx_1(i);
						}
						vec_PredictedFriction(i) = vec_PredictedFriction(i)+ mt_GMSForceF(i,j);
					}
				} 
				else
				{
					for (j=0;j<vec_GMSElement(i);j++)///mt_GMSModel  (Axis 1..i,Number element, Element 1...
					{
						if (mt_GMSForceF(i,j)> (-mt_GMSForceW(i,j)))
						{
							mt_GMSForceF(i,j) = mt_GMSForceF(i,j)+m_fSampTime*mt_GMSStiffK(i,j)*vec_Selectx_1(i);
						}
						vec_PredictedFriction(i) = vec_PredictedFriction(i)+ mt_GMSForceF(i,j);
					}
				}

			}
		}
		break;
	case 3:     // Static nonlinear friction (Couloumb, viscous, nonlinear gaussians )  mt_PeakHeight,mt_PeakPos,mt_PeakWid,mt_NumberPeak,

		for (i=0;i<NUMBERAXIS;i++)
		{
			
			if (vec_Selectx_1(i)>=0)
			{
				vec_PredictedFriction(i) = vec_NominalFc(i)+vec_NominalFv(i)*vec_Selectx_1(i);
				for (j=0;j<vec_NumberPeak(i*2);j++)
				{
					vec_PredictedFriction(i) = vec_PredictedFriction(i)+ mt_PeakHeight(i*2,j)*gaussian(vec_Selectx_1(i),mt_PeakPos(i*2,j),mt_PeakWid(i*2,j));
				}
			}
			else
			{
				vec_PredictedFriction(i) = -vec_NominalFc(i)+vec_NominalFv(i)*vec_Selectx_1(i);
				for (j=0;j<vec_NumberPeak(i*2+1);j++)
				{
					vec_PredictedFriction(i) = vec_PredictedFriction(i)+ mt_PeakHeight(i*2+1,j)*gaussian(vec_Selectx_1(i),mt_PeakPos(i*2+1,j),mt_PeakWid(i*2+1,j));
				}
			}

		}

		break;
	}
}
void FiveAxisCNC::CalculatePredictedDisturbance(void)
{
	int i=0;
	switch (m_iDOBModel) // Calculate disturbance base on different disturbance observer 
	{
	case 0:  // No disturbance observer
		for (i=0;i<NUMBERAXIS;i++)
		{
			vec_DOBd(i) = 0;
		}
		break;
	case 1:  // General disturbance observer
		vec_DOBew_1 = vec_DOBEstimatedx_1 - vec_realx_1;
		vec_DOBd = vec_DOBd + ((float)m_fSampTime) * prod(mt_DOBKd,vec_DOBew_1);
//		vec_DOBd = vec_DOBd + (m_fSampTime) * prod(mt_DOBKd,vec_DOBew_1);
		break;
	case 2:  // Disturbance observer use Kalman filter
// 		for (i=1;i<NUMBERAXIS;i++)
// 		{
// 			vec_DOBd(i) = 0;
// 		}
//		vec_KalmanState(0)= vec_KalmanEsX(i);
		vec_KalmanMesured(0)= vec_realx(0);
		vec_KalmanMesured(1)= vec_realx_1(0);
		vec_KalmanMesured(2)= vec_DOBd(0);

// 		vec_KalmanState = prod(prod((mt_KalmanIX-prod(mt_KalmanKobsX,mt_KalmanCX)),mt_KalmanAX),vec_KalmanState)+
// 						 prod((mt_KalmanIX-prod(mt_KalmanKobsX,mt_KalmanCX)),vec_KalmanState)*vec_OutputControl(0);vec_KalmanBX

		vec_KalmanState = prod(prod((mt_KalmanIX-prod(mt_KalmanKobsX,mt_KalmanCX)),mt_KalmanAX),vec_KalmanState)+
							prod((mt_KalmanIX-prod(mt_KalmanKobsX,mt_KalmanCX)),vec_KalmanBX)*(vec_OutputControl(0)-vec_PredictedFriction(0))+
							prod(mt_KalmanKobsX,vec_KalmanMesured);	

		vec_DOBd(0) = -vec_KalmanState(2);

		break;
	}

}
void FiveAxisCNC::UpdateDisturbanceObserverParameter(void)
{
	switch (m_iDOBModel) // Calculate disturbance base on different disturbance observer 
	{
	case 0:  // No disturbance observer
		break;
	case 1:  // General disturbance observer
		vec_DOBEstimatedx_1 = vec_realx_1+((float)m_fSampTime)*(prod(mt_DOBKv,vec_DOBew_1)+prod(mt_1_M,vec_OutputControl-vec_PredictedFriction-vec_DOBd));
		break;
	case 2:  // Disturbance observer use Kalman filter
		break;
	}

}
void FiveAxisCNC::CalculateDisturbanceObserve(void)
{

}
void FiveAxisCNC::ResetReferenceData(void)
{
	vec_refr.clear();vec_Pre_refr.clear();
	vec_refr_1.clear();vec_Pre_refr_1.clear();
}
void FiveAxisCNC::ResetRealData(void)
{
	vec_realx.clear();vec_Pre_realx.clear();
	vec_realx_1.clear();vec_Pre_realx_1.clear();
	vec_realx_2.clear();;
}
void FiveAxisCNC::CalulateReferenceData(void)
{
	vec_Pre_refr_1 = vec_refr_1;
	vec_refr_1 = (vec_refr - vec_Pre_refr)/(float)m_fSampTime;//  mm/s
	vec_refr_2 = (vec_refr_1 - vec_Pre_refr_1)/(float)m_fSampTime;///m_fSampTime; 

// 	switch (m_iMoveType) 
// 	{
// 
// 	case 0:
// 		//	m_iMoveType =(int)GetGcodeVariableValue(strRead, index);
// 		break;
// 
// 	case 1:
// 		// Already calculated in ReadNextGcodeLine
// 		break;
// 	case 2:
// 		// Reference Velocity
// 		vec_Pre_refr_1 = vec_refr_1;
// 		vec_refr_1(0) = (-m_fOmg)*(m_CNCRefPos.Y- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
// 		vec_refr_1(1) = (m_fOmg)*(m_CNCRefPos.X-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
// 		// Reference acceleration
// 		vec_refr_2(0) = (-m_fOmg)*m_fOmg*(m_CNCRefPos.X-m_fCNCI);//-m_fexpR*m_fOmg*m_fOmg*cos(theta);
// 		vec_refr_2(1) = (-m_fOmg)*m_fOmg*(m_CNCRefPos.Y- m_fCNCJ);//-m_fexpR*m_fOmg*m_fOmg*sin(theta)
// 		// 		vec_Pre_refr_1 = vec_refr_1;
// 		// 		vec_refr_1 = (vec_refr - vec_Pre_refr)/m_fSampTime;//  mm/s
// 		// 		vec_refr_2 = (vec_refr_1 - vec_Pre_refr_1)/m_fSampTime;//  mm/s
// 		//		vec_Pre_refr_2 = (vec_refr_1 - vec_Pre_refr_1)/m_fSampTime;
// 
// 		break;
// 	case 3:
// 		// Reference Velocity
// 		vec_Pre_refr_1 = vec_refr_1;
// 		vec_refr_1(0) = (-m_fOmg)*(m_CNCRefPos.Y- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
// 		vec_refr_1(1) = (m_fOmg)*(m_CNCRefPos.X-m_fCNCI) ;//m_fexpR*m_fOmg*Cos(theta);		mm/s
// 		// Reference acceleration
// 		vec_refr_2(0) = (-m_fOmg)*m_fOmg*(m_CNCRefPos.X-m_fCNCI);//-m_fexpR*m_fOmg*m_fOmg*cos(theta);
// 		vec_refr_2(1) = (-m_fOmg)*m_fOmg*(m_CNCRefPos.Y- m_fCNCJ);//-m_fexpR*m_fOmg*m_fOmg*sin(theta);
// // 		vec_Pre_refr_1 = vec_refr_1;
// // 		vec_refr_1 = (vec_refr - vec_Pre_refr)/m_fSampTime;//  mm/s
// // 		vec_refr_2 = (vec_refr_1 - vec_Pre_refr_1)/m_fSampTime;//  mm/s
// //		vec_Pre_refr_2 = (vec_refr_1 - vec_Pre_refr_1)/m_fSampTime;
// 		break;
// 	default:
// 		;//;
// 	}

}
void FiveAxisCNC::CalculateRealPostisionData(void)
{
	vec_Pre_realx_1 = vec_realx_1;
	vec_realx_1 = (vec_realx - vec_Pre_realx)/(float)m_fSampTime;//  mm/s
	vec_realx_2 = (vec_realx_1 - vec_Pre_realx_1)/(float)m_fSampTime;///m_fSampTime; 
}
void FiveAxisCNC::InitNonlinearFrictionData(System::String^ fileNLF)
{
	int i=0,j=0;

	try
	{
		FileStream^ BinaryDataFileStream = gcnew FileStream(fileNLF, FileMode::Open);
		BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);

//	while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
		{
			
			for (i=0;i<NUMBERAXIS;i++)
			{
				vec_NominalFc(i) = BinaryDataReader->ReadDouble(); // Read nominal Couloumb Friction
				vec_NominalFv(i) = BinaryDataReader->ReadDouble(); // Read nominal viscous coefficient

				vec_NumberPeak(i*2)=BinaryDataReader->ReadByte();
				for (j=0;j<((int)vec_NumberPeak(i*2));j++)
				{
					mt_PeakHeight(i*2,j) = BinaryDataReader->ReadDouble();
					mt_PeakPos(i*2,j) = BinaryDataReader->ReadDouble();
					mt_PeakWid(i*2,j) = BinaryDataReader->ReadDouble();
				}
				vec_NumberPeak(i*2+1)=BinaryDataReader->ReadByte();
				for (j=0;j<((int)vec_NumberPeak(i*2+1));j++)
				{
					mt_PeakHeight(i*2+1,j) = BinaryDataReader->ReadDouble();
					mt_PeakPos(i*2+1,j) = BinaryDataReader->ReadDouble();
					mt_PeakWid(i*2+1,j) = BinaryDataReader->ReadDouble();
				}
			}
				
		}
		//			Console::WriteLine(BinaryDataReader->ReadInt32().ToString());
		BinaryDataFileStream->Close( );


	}
	catch (Exception^ e)
	{
		//		if (dynamic_cast<FileNotFoundException^>(e))
		//			Console::WriteLine("File '{0}' not found", fileName);
		//		else
		//			Console::WriteLine("Exception: ({0})", e);
		//		return -1;

	}
}
void FiveAxisCNC::InitGMSModelData(System::String^ fileGMS)
{
	int i=0,j=0;
	try
	{
		FileStream^ BinaryDataFileStream = gcnew FileStream(fileGMS, FileMode::Open);
		BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);

		//	while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
		{

			for (i=0;i<NUMBERAXIS;i++)
			{
				vec_SlidingVel(i)=BinaryDataReader->ReadDouble(); // Siliding regime velocity
				vec_GMSElement(i)=BinaryDataReader->ReadByte(); // Number element of GMS model for axis i
				for (j=0;j<((int)vec_GMSElement(i));j++)
				{
					mt_GMSForceW(i,j) = BinaryDataReader->ReadDouble();
					mt_GMSStiffK(i,j) = BinaryDataReader->ReadDouble();
				}
			}

		}
		//			Console::WriteLine(BinaryDataReader->ReadInt32().ToString());
		BinaryDataFileStream->Close( );
	}
	catch (Exception^ e)
	{
		//		if (dynamic_cast<FileNotFoundException^>(e))
		//			Console::WriteLine("File '{0}' not found", fileName);
		//		else
		//			Console::WriteLine("Exception: ({0})", e);
		//		return -1;
	}
}
void FiveAxisCNC::InitKalmanFilterModelData(System::String^ fileKalmanFilter)
{
	int i=0,j=0,k=0;

	// Reset kalman filter
		for (i=1;i<NUMBERAXIS;i++)
		{
			vec_DOBd(i) = 0;
		}
		vec_KalmanState.clear();

	try
	{
		FileStream^ BinaryDataFileStream = gcnew FileStream(fileKalmanFilter, FileMode::Open);
		BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);

		//	while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
		{
			for (i=0;i<NUMBERAXIS;i++)
			{
// 				mt_KalmanIX.resize(NUMSTATE,NUMSTATE); //Kalman filter
// 				mt_KalmanKobsX.resize(NUMSTATE,NUMMEASURED); 
// 				mt_KalmanAX.resize(NUMSTATE,NUMSTATE); 
// 				mt_KalmanBX.resize(NUMSTATE,1); 
// 				mt_KalmanCX.resize(NUMMEASURED,NUMSTATE);    // (2,3)

				for (j=0;j<NUMSTATE;j++)
				{
					for (k=0;k<NUMSTATE;k++)
					{
						mt_KalmanIX(j,k) = BinaryDataReader->ReadDouble();
					}
				}

				for (j=0;j<NUMSTATE;j++)
				{
					for (k=0;k<NUMMEASURED;k++)
					{
						mt_KalmanKobsX(j,k) = BinaryDataReader->ReadDouble();
					}
				}

				for (j=0;j<NUMSTATE;j++)
				{
					for (k=0;k<NUMSTATE;k++)
					{
						mt_KalmanAX(j,k) = BinaryDataReader->ReadDouble();
					}
				}

				for (j=0;j<NUMSTATE;j++)
				{
					vec_KalmanBX(j) = BinaryDataReader->ReadDouble();
				}

				for (j=0;j<NUMMEASURED;j++)
				{
					for (k=0;k<NUMSTATE;k++)
					{
						mt_KalmanCX(j,k) = BinaryDataReader->ReadDouble();
					}
				}
			}
		}
		//			Console::WriteLine(BinaryDataReader->ReadInt32().ToString());
		BinaryDataFileStream->Close( );
	}
	catch (Exception^ e)
	{
		//		if (dynamic_cast<FileNotFoundException^>(e))
		//			Console::WriteLine("File '{0}' not found", fileName);
		//		else
		//			Console::WriteLine("Exception: ({0})", e);
		//		return -1;
	}
}
void FiveAxisCNC::InitGlobalVariable(System::String^ FileGlobalVariable,System::String^ FileNonlinearFriction,System::String^ FileGMSModel)
{
//	System::String^ strRead,variableName,variableValue;
	StreamReader^ GlobalVariableFileStream;
	double m_fOmega0,m_fOmega1,m_fOmega2,m_fOmega3,m_fOmega4;
// Default value if we can not find GlobalVariable.txt file
	//m_fOmega0 = 120;m_fOmega1 = 100;  // High feedback gain
	m_fOmega0 = 60;m_fOmega1 = 100; m_fOmega2 = 70; // Low feedback gain
	 m_fOmega3 = 30;  m_fOmega4 = 20; 
	// 0    1    2   3   4   
	// X    Y    Z   C   A
	mt_M.clear();mt_Kc.clear();vec_Fc.clear();
	mt_Kvw.clear();mt_Kpw.clear();
	mt_Kvl.clear();mt_Kpl.clear();

	mt_Kvw(0,0) = 2*m_fOmega0; mt_Kvw(1,1) = 2*m_fOmega1;  // X, Y1,  Z, C, A1
	mt_Kvw(2,2) = 4.545;	mt_Kvw(3,3) =  2*m_fOmega2;//1.72;
	mt_Kvw(4,4) = 2*m_fOmega3;
	
	mt_Kvw(5,5) = 2*m_fOmega4;mt_Kvw(6,6) = 0.01489;

	mt_Kpw(0,0) = m_fOmega0*m_fOmega0; mt_Kpw(1,1) = m_fOmega1*m_fOmega1;
	mt_Kpw(2,2) = 4.545;	mt_Kpw(3,3)  = m_fOmega2*m_fOmega2;//1.72;
	mt_Kpw(4,4) = m_fOmega3*m_fOmega3;mt_Kpw(5,5) = m_fOmega4*m_fOmega4; mt_Kpw(6,6) = 0.01489;


	// Pre define coefficient, but will be redefine from GlobalVariable.txt file
	mt_M(0,0) = 0.00533; mt_M(1,1) = 0.004545;//mt_M(0,0) = 5.33; mt_M(1,1) = 4.545;
	mt_M(2,2) = 4.545;	mt_M(3,3) = 0.0029;
	mt_M(4,4) = 0.0023;mt_M(5,5) = 0.001489;mt_M(6,6) = 0.01489;

	mt_Kc(0,0) = 0.025175; mt_Kc(1,1) = 0.024202;
	mt_Kc(2,2) = 0.024;	mt_Kc(3,3) = 0.024;
	mt_Kc(4,4) =0.0022;mt_Kc(5,5) = 0.001;mt_Kc(6,6) = 0.1;

// 	mt_Kc(0,0) = 25.175; mt_Kc(1,1) = 24.202;
// 	mt_Kc(2,2) = 24.202;	mt_Kc(3,3) = 71.646;
// 	mt_Kc(4,4) =0.0022;mt_Kc(5,5) = 0.1;mt_Kc(6,6) = 0.1;

	vec_Fc(0) = 0.025; vec_Fc(1) = 0.025;
	vec_Fc(2) = 0.025;	vec_Fc(3) = 0.025;
	vec_Fc(4) = 0.0015;vec_Fc(5) = 0.035;vec_Fc(6) = 0.035;

// Initial Global variable 
	System::String^ strRead;
	System::String^ variableName;
	double variableValue;
//	try 
	{
		//		Console::WriteLine("trying to open file {0}...", fileName);
// 		GcodeFileStream = File::OpenText(FileGlobalVariable);
// 		strRead = GcodeFileStream->ReadLine();

		GlobalVariableFileStream = File::OpenText(FileGlobalVariable);
		while ((strRead = GlobalVariableFileStream->ReadLine()) != nullptr)
		{
			rmsscanf(strRead,variableName,variableValue);
			InitVariableName(variableName,variableValue);
		}

// 		sscanf( tokenstring, "%80s", s ); // C4996 String
// 		sscanf( tokenstring, "%c", &c );  // C4996 Character
// 		sscanf( tokenstring, "%d", &i );  // C4996  Integer
// 		sscanf( tokenstring, "%f", &fp ); // C4996 Real
//		sscanf(strRead,"%30s%f",variableName,variableValue);

		GlobalVariableFileStream->Close();
	}
//	catch (Exception^ e)
	{
		//		if (dynamic_cast<FileNotFoundException^>(e))
		//			Console::WriteLine("file '{0}' not found", fileName);
		//		else
		//			Console::WriteLine("problem reading file '{0}'", fileName);
	}

	InitNonlinearFrictionData(FileNonlinearFriction);
	InitGMSModelData(FileGMSModel);
    InitControlVariable();

}
void FiveAxisCNC::InitControlVariable()
{
		m_fCNCStartX = 0.0;
		m_fCNCStartY = 0.0;
		m_fCNCStartZ = 0.0;
		m_fCNCEndX = 0.0;
		m_fCNCEndY = 0.0;
		m_fCNCEndZ = 0.0;
		m_CNCRefPos.X =  0.0;
		m_CNCRefPos.Y =  0.0;
		m_CNCRefPos.Z =  0.0;
		m_fexpTnow = 0.0;
		m_fexpRunT = 0.0;
		m_fexpRunTPre = 0.0;
		m_fexpTnowReal= 0.0;
		m_fexpTnowCounter= 0.0;
		m_fNextAccFirstTime = 0.0;

		m_fAbso_ec = 0.0;
		vec_refr.clear();
		//vec_refr(0) = m_CNCInitPos.X;	vec_refr(1) = m_CNCInitPos.Y;
		vec_refr_1.clear();
		vec_refr_2.clear();
		vec_Pre_refr.clear();
		vec_Pre_refr_1.clear();
//		vec_Pre_refr_2(0) =0.0   	;vec_Pre_refr_2(1) =0.0;

		vec_realx.clear();
		vec_realx_1.clear();
		vec_Pre_realx.clear();
		vec_Pre_realx_1.clear();

		vec_ew.clear();
		vec_ew_1.clear();
		vec_el.clear();
		vec_OutputControl.clear();
		vec_PredictedOutput.clear();

/*		vec_Pre_realx_2(0) = 0.0;vec_Pre_realx_2(1) = 0.0;*/
//		
//		m_bGcodeFINISH = FALSE;

		m_bGcodeFINISH = FALSE;
		m_bPathFinish = FALSE;
		m_bNextGcodeFINISH = FALSE;

		m_iNextMoveType = 1;
		//	m_fexpRunT = m_fNextexpRunT;

		 m_fNextAccFirstTime = 0.0;
		m_fNextCNCPosOneX = 0.0;;m_fNextCNCPosTwoX = 0.0;;
		//	m_fCNCPosThreeX = m_fNextCNCPosThreeX;
		m_fNextCNCPosOneY = 0.0; m_fNextCNCPosTwoY = 0.0;
		m_fNextCNCPosOneZ = 0.0; m_fNextCNCPosTwoZ = 0.0;
		//	m_fCNCPosThreeY = m_fNextCNCPosThreeY;

		m_fNextCNCStartVelX = 0.0;m_fNextCNCMidVelX = 0.0;m_fNextCNCEndVelX= 0.0;
		m_fNextCNCStartAccX = 0.0;//m_fCNCEndAccX = m_fNextCNCEndAccX;
		m_fNextCNCStartVelY = 0.0;m_fNextCNCMidVelY = 0.0;m_fNextCNCEndVelY= 0.0;
		m_fNextCNCStartAccY = 0.0;//m_fCNCEndAccY = m_fNextCNCEndAccY;
		m_fNextCNCStartVelZ = 0.0;m_fNextCNCMidVelZ = 0.0;m_fNextCNCEndVelZ= 0.0;
		m_fNextCNCStartAccZ = 0.0;//m_fCNCEndAccY = m_fNextCNCEndAccY;

		m_fNextCNCAngleOne = 0.0;m_fNextCNCAngleTwo = 0.0;
		//	m_fCNCAngleThree = m_fNextCNCAngleThree;
		m_fNextCNCStartAngleVel = 0.0;m_fNextCNCMidAngleVel= 0.0;
		m_fNextCNCStartAngleAcc = 0.0;

		
		m_fNextCNCStartX = 0.0;
		m_fNextCNCStartY = 0.0;
		m_fNextCNCStartZ = 0.0;
		m_fNextCNCEndX= 0.0;
		m_fNextCNCEndY= 0.0;
		m_fNextCNCEndZ= 0.0;
		m_fNextLengthX= 0.0;
		m_fNextLengthY= 0.0;
		m_fNextLengthZ= 0.0;
		m_fNextCNCI= 0.0;
		m_fNextCNCJ= 0.0;
		m_fNextCNCRadius= 0.0;
		m_fNextCNCStartAngle= 0.0;
		m_fNextOmg= 0.0;

		m_fNextAccFirstTime= 0.0;
		m_fNextCNCPH = 400.0;//400
		m_fNextCNCPE = 400.0;
		//	m_fStartddotOmg = m_fNextStartddotOmg;
		//	m_fStartddotAcc = m_fNextStartddotAcc;
		//	m_CNCStartRefVel = m_NextCNCStartRefVel;
		//	m_CNCEndRefVel = m_NextCNCEndRefVel;

// 		m_fNextCNCRadius = 999999.1;
// 		m_fNextCNCI = 0.0;
//		m_fCNCFeedRate = 0.0; // Give not zero value for avoid divide zero
		m_fNextCNCFeedRate = 0.0;

}
void FiveAxisCNC::OpenGcodeFile(System::String^ FileGcodeProgram)
{
	try 
	{
//		Console::WriteLine("trying to open file {0}...", fileName);
		GcodeFileStream = File::OpenText(FileGcodeProgram);
		//GetNextGCodeLine();	// Read two time to get busmemory of next curve
		//GetNextGCodeLine();	

	}
	catch (Exception^ e)
	{
//		if (dynamic_cast<FileNotFoundException^>(e))
//			Console::WriteLine("file '{0}' not found", fileName);
//		else
//			Console::WriteLine("problem reading file '{0}'", fileName);
	}

}
void FiveAxisCNC::OpenBinaryFile(System::String^ FileDataAnalysis)
{
	array<Int32>^ data = {1, 2, 3, 10000};



	try 
	{
	 DataAnalysisFileStream = gcnew FileStream(FileDataAnalysis, FileMode::Create);
	 DataAnalysisBinaryWriter = gcnew BinaryWriter(DataAnalysisFileStream);
//		Console::WriteLine("writing data to file:");
// 		for (int i=0; i<data->Length; i++)
// 		{
// //			Console::WriteLine(data[i]);
// 			DataAnalysisBinaryWriter->Write(data[i]);
// 		}
	}
	catch (Exception^) 
	{
//		Console::WriteLine("data could not be written");
//		DataAnalysisFileStream->Close();
//		return -1;
	}

}
void FiveAxisCNC::SaveDataToBinaryFile()
{
//			vec_realx = vec_refr;
			DataAnalysisBinaryWriter->Write(m_fexpTnowReal);
			DataAnalysisBinaryWriter->Write(m_fexpTnowCounter);
			DataAnalysisBinaryWriter->Write(vec_refr(0));
			DataAnalysisBinaryWriter->Write(vec_refr(1));
			DataAnalysisBinaryWriter->Write(vec_refr(3));
			DataAnalysisBinaryWriter->Write(vec_refr(4));
			DataAnalysisBinaryWriter->Write(vec_refr(5));
			DataAnalysisBinaryWriter->Write(vec_realx(0));
			DataAnalysisBinaryWriter->Write(vec_realx(1));
			DataAnalysisBinaryWriter->Write(vec_realx(3));
			DataAnalysisBinaryWriter->Write(vec_realx(4));
			DataAnalysisBinaryWriter->Write(vec_realx(5));
			DataAnalysisBinaryWriter->Write(vec_ew(0));//vec_ew(0)   vec_refr_1(0)
			DataAnalysisBinaryWriter->Write(vec_ew(1));//vec_ew(1)
			DataAnalysisBinaryWriter->Write(vec_ew(3));//vec_ew(3)
			DataAnalysisBinaryWriter->Write(vec_ew(4));//vec_ew(3)
			DataAnalysisBinaryWriter->Write(vec_ew(5));//vec_ew(3)
			DataAnalysisBinaryWriter->Write(vec_OutputControl(0));
			DataAnalysisBinaryWriter->Write(vec_OutputControl(1));
			DataAnalysisBinaryWriter->Write(vec_OutputControl(3));
			DataAnalysisBinaryWriter->Write(vec_OutputControl(4));
			DataAnalysisBinaryWriter->Write(vec_OutputControl(5));
			DataAnalysisBinaryWriter->Write(vec_PredictedOutput(0));
			DataAnalysisBinaryWriter->Write(vec_PredictedOutput(1));
			DataAnalysisBinaryWriter->Write(vec_PredictedOutput(3));
			DataAnalysisBinaryWriter->Write(vec_PredictedOutput(4));
			DataAnalysisBinaryWriter->Write(vec_PredictedOutput(5));


//			DataAnalysisBinaryWriter->Write(vec_realx_1(0));
// 			DataAnalysisBinaryWriter->Write(vec_realx(0));
// 			DataAnalysisBinaryWriter->Write(vec_realx(1));

}
void FiveAxisCNC::GetNextRealtimeRef() //*********************************Real Time********************//
{

}
void FiveAxisCNC::GetNextPointRefInRegulation() {
		//		theta = m_fOmg*m_fexpTnow;
	m_fSampTime = m_fexpTnowReal- m_fexpTnow-m_fexpRunTPre;

	m_fexpTnow = m_fexpTnowReal- m_fexpRunTPre;

	if (m_fSampTime< DOUBLE_TOLERANCE)
	{
		m_fSampTime = 0.005;
	}

		vec_Pre_refr =vec_refr;
		vec_refr(0) = m_CNCRefPos.X ;
		vec_refr(1) = m_CNCRefPos.Y;
		vec_refr(3) = m_CNCRefPos.Z;
		vec_refr(4) = m_CNCRefPos.C;
		vec_refr(5) = m_CNCRefPos.A;
}
void FiveAxisCNC::GetNextPointRefInGCodePath() {
	double referenceTime;
	double referenceAngle;
	m_fSampTime = m_fexpTnowReal- m_fexpTnow-m_fexpRunTPre;

	m_fexpTnow = m_fexpTnowReal- m_fexpRunTPre;

	if (m_fSampTime< 0.5*m_fSampTimeRef)
	{
		m_fSampTime = m_fSampTimeRef;
	}

	switch (m_iMoveType) 
	{

	case 0:

		//	m_iMoveType =(int)GetGcodeVariableValue(strRead, index);

		break;

	case 1:
		if (((m_fexpTnow+m_fexpRunTPre)<(m_fAccFirstTime))&&(m_fAccFirstTime>m_fexpRunTPre+DOUBLE_TOLERANCE))
		{
			m_CNCRefPos.X =   m_fCNCPosOneX+ m_fCNCStartVelX *m_fexpTnow + m_fCNCStartAccX*m_fexpTnow*m_fexpTnow/2.0;
			m_CNCRefPos.Y =   m_fCNCPosOneY+ m_fCNCStartVelY *m_fexpTnow + m_fCNCStartAccY*m_fexpTnow*m_fexpTnow/2.0;
			m_CNCRefPos.Z =   m_fCNCPosOneZ+ m_fCNCStartVelZ *m_fexpTnow + m_fCNCStartAccZ*m_fexpTnow*m_fexpTnow/2.0;
// 			m_CNCRefPos.X =   vec_CNCPosOne(0)+ m_CNCStartRefVel.X *m_fexpTnow + vec_StartAcc(0)*m_fexpTnow*m_fexpTnow/2.0;
// 			m_CNCRefPos.Y =   m_fCNCStartY+ m_CNCStartRefVel.Y *m_fexpTnow + vec_StartAcc(1)*m_fexpTnow*m_fexpTnow/2.0;
		}else
		if (((m_fexpTnow+m_fexpRunTPre)>(m_fAccEndTime))&&(m_fAccEndTime+DOUBLE_TOLERANCE<m_fexpRunT))
		{
			referenceTime = m_fexpTnow+m_fexpRunTPre - m_fAccEndTime;
			m_CNCRefPos.X =   m_fCNCPosThreeX+ m_fCNCMidVelX *referenceTime + m_fCNCEndAccX*referenceTime*referenceTime/2.0;
			m_CNCRefPos.Y =   m_fCNCPosThreeY+ m_fCNCMidVelY *referenceTime + m_fCNCEndAccY*referenceTime*referenceTime/2.0;
			m_CNCRefPos.Z =   m_fCNCPosThreeZ+ m_fCNCMidVelZ *referenceTime + m_fCNCEndAccZ*referenceTime*referenceTime/2.0;
// 			m_CNCRefPos.X =   m_fCNCStartX+ m_CNCStartRefVel.X *m_fexpTnow + vec_EndAcc(0)*m_fexpTnow*m_fexpTnow/2.0;
// 			m_CNCRefPos.Y =   m_fCNCStartY+ m_CNCStartRefVel.Y *m_fexpTnow + vec_EndAcc(1)*m_fexpTnow*m_fexpTnow/2.0;
		}else
		{
			referenceTime = m_fexpTnow+m_fexpRunTPre - m_fAccFirstTime;
			m_CNCRefPos.X =   m_fCNCPosTwoX+ m_fCNCMidVelX *referenceTime ; // Bug m_fCNCPosTwoX == 0 
			m_CNCRefPos.Y =   m_fCNCPosTwoY+ m_fCNCMidVelY *referenceTime ;
			m_CNCRefPos.Z =   m_fCNCPosTwoZ+ m_fCNCMidVelZ *referenceTime ;
// 			m_CNCRefPos.X =   m_fCNCStartX+ (m_fexpTnow/(m_fexpRunT- m_fexpRunTPre))* m_fLengthX;
// 			m_CNCRefPos.Y =   m_fCNCStartY+ (m_fexpTnow/(m_fexpRunT- m_fexpRunTPre))* m_fLengthY;
		}
		break;
	case 2: case 3:
		if (((m_fexpTnow+m_fexpRunTPre)<(m_fAccFirstTime))&&(m_fAccFirstTime>m_fexpRunTPre+DOUBLE_TOLERANCE))
		{
			referenceAngle = m_fCNCAngleOne + m_fCNCStartAngleVel*m_fexpTnow+ m_fCNCStartAngleAcc*m_fexpTnow*m_fexpTnow/2.0;
			
		}else
			if (((m_fexpTnow+m_fexpRunTPre)>(m_fAccEndTime))&&(m_fAccEndTime+DOUBLE_TOLERANCE<m_fexpRunT))
			{
				referenceTime = m_fexpTnow+m_fexpRunTPre - m_fAccEndTime;
				referenceAngle = m_fCNCAngleThree + m_fCNCMidAngleVel*referenceTime+ m_fCNCEndAngleAcc*referenceTime*referenceTime/2.0;
			}else
			{
				referenceTime = m_fexpTnow+m_fexpRunTPre - m_fAccFirstTime;
				referenceAngle = m_fCNCAngleTwo + m_fCNCMidAngleVel*referenceTime;
			}
			m_CNCRefPos.X =  m_fCNCI+ m_fCNCRadius*cosf(referenceAngle);
			m_CNCRefPos.Y =  m_fCNCJ+ m_fCNCRadius*sinf(referenceAngle);
// 		m_CNCRefPos.X =  m_fCNCI+ m_fCNCRadius*cos(m_fCNCStartAngle+m_fOmg*m_fexpTnow);
// 		m_CNCRefPos.Y =  m_fCNCJ+ m_fCNCRadius*sin(m_fCNCStartAngle+m_fOmg*m_fexpTnow);
		break;
	case 4:
		switch (m_iMathCurveNumber) 
		{
		case EIGHTCURVE: //r^2 = a^2cos(2*theta)sec^4(theta) 

			m_fMath_Theta = (2*m_fMath_b*PI)*m_fexpTnow/m_fMath_Time;
			m_CNCRefPos.X =  m_fMath_a*sinf(m_fMath_Theta/m_fMath_b);
			m_CNCRefPos.Y =  m_fMath_a*sinf(m_fMath_Theta/m_fMath_c);
			break;
		case TRIFOLIUM:  //r = a costheta (4sin2theta - 1) 
			m_fMath_Theta = PI*m_fexpTnow/m_fMath_Time;
			m_CNCRefPos.X =  cosf(m_fMath_Theta)*m_fMath_a*cosf(m_fMath_Theta)*(4*sinf(m_fMath_Theta)*sinf(m_fMath_Theta)-1)+ m_fMath_a;
			m_CNCRefPos.Y =  sinf(m_fMath_Theta)*m_fMath_a*cosf(m_fMath_Theta)*(4*sinf(m_fMath_Theta)*sinf(m_fMath_Theta)-1);
			break;
		case LISSAJOUS:
			m_fMath_Theta = 2*PI*m_fexpTnow/m_fMath_Time;
			m_CNCRefPos.X =  m_fMath_a*sinf(m_fMath_n*m_fMath_Theta + m_fMath_c)-m_fMath_a*sinf(m_fMath_c);
			m_CNCRefPos.Y =  m_fMath_b*sinf(m_fMath_Theta);
			break;
		case TUNINGPROCESS:
//			m_fMath_Theta = 2*PI*m_fexpTnow/m_fMath_Time;
			if (m_iTuningMotor == LINEAR_MOTOR_X){ m_CNCRefPos.X =  m_fMath_r*cosf(m_fMath_w*m_fexpTnow)-m_fMath_r;};
			if (m_iTuningMotor == LINEAR_MOTOR_Y1){ m_CNCRefPos.Y =  m_fMath_r*cosf(m_fMath_w*m_fexpTnow)-m_fMath_r;};
			if (m_iTuningMotor == LINEAR_MOTOR_Z){  m_CNCRefPos.Z =  m_fMath_r*cosf(m_fMath_w*m_fexpTnow)-m_fMath_r;};
			if (m_iTuningMotor == LINEAR_MOTOR_C){  m_CNCRefPos.C =  m_fMath_r*cosf(m_fMath_w*m_fexpTnow)-m_fMath_r;};
			if (m_iTuningMotor == LINEAR_MOTOR_A1){  m_CNCRefPos.A =  m_fMath_r*cosf(m_fMath_w*m_fexpTnow)-m_fMath_r;};
			break;
		default:
			break;
		}
		break;
	default:
		;//;
	}
		if ((m_fexpTnow+m_fexpRunTPre)>(m_fexpRunT+DOUBLE_TOLERANCE))
		{
			if (!m_bNextGcodeFINISH)
			{

			m_bPathFinish = TRUE;
			// 		m_CNCRefPos.X = m_fCNCEndX;
			// 		m_CNCRefPos.Y = m_fCNCEndY;
			vec_Pre_refr =vec_refr;
			//		theta = m_fOmg*m_fexpTnow;
			vec_refr(0) = m_CNCRefPos.X ;
			vec_refr(1) = m_CNCRefPos.Y;
			vec_refr(3) = m_CNCRefPos.Z;
			vec_refr(4) = m_CNCRefPos.C;
			vec_refr(5) = m_CNCRefPos.A;
			// 		GenerateReferenceData();// Also done in GetNextRef();
			//  		GetRealPostisionData();
			m_fexpTnow = m_fexpTnowReal-m_fexpRunT;
			m_fexpRunTPre = m_fexpRunT;//m_fexpTnowReal;//m_fexpRunTPre+m_fexpTnow; // =m_fexpTnowReal
			GetNextGCodeLine(); 	
			}else
			{
				m_bGcodeFINISH = TRUE;
				vec_refr(0) = m_fCNCEndX ;
				vec_refr(1) = m_fCNCEndY;
				vec_refr(3) = m_fCNCEndZ;
				vec_refr(4) = m_fCNCEndC;
				vec_refr(5) = m_fCNCEndA;
			}
		}
		else
		{
			//		theta = m_fOmg*m_fexpTnow;
			vec_Pre_refr =vec_refr;
			vec_refr(0) = m_CNCRefPos.X ;
			vec_refr(1) = m_CNCRefPos.Y;
			vec_refr(3) = m_CNCRefPos.Z;
			vec_refr(4) = m_CNCRefPos.C;
			vec_refr(5) = m_CNCRefPos.A;
		}	
}
double FiveAxisCNC::GetGcodeVariableValue(System::String^  gcodeString,unsigned int &indexStart)
{
	System::String^ stringNumber;
	int i= indexStart+1;
	double temp = 0.0;
	stringNumber = "";//Char.IsDigit(ch) char->IsDigit(gcodeString[i])
	//while ((i<gcodeString->Length) &&(gcodeString[i] !=' ')&&(i<256))
	while ((i<gcodeString->Length)&&(i<256) &&
		((gcodeString[i] == '+')||(gcodeString[i] == '-')||(gcodeString[i] == '.')||
		(gcodeString[i] == '0')||(gcodeString[i] == '1')||(gcodeString[i] == '2')||
		(gcodeString[i] == '3')||(gcodeString[i] == '4')||(gcodeString[i] == '5')||
		(gcodeString[i] == '6')||(gcodeString[i] == '7')||(gcodeString[i] == '8')||
		(gcodeString[i] == '9')
		))
	{
		stringNumber= stringNumber+ gcodeString[i];
		i++;
	}
	indexStart = i;
	temp= System::Convert::ToDouble(stringNumber);
//	sscanf(strRead,"%f",&temp);
	return temp;

}
bool FiveAxisCNC::AnalyseNextGcodeLine(System::String^ strGcodeLine) 
{
	bool EndofString= FALSE, IsNextGcodeMovement = FALSE;
	unsigned int index = 0;
	EndofString= FALSE;
	int tempGcmd = 0;
	//   char testChar = strGcodeLine[strGcodeLine->Length-1];
	while (!EndofString)
	{
		if (index < (unsigned int)strGcodeLine->Length)
		{
			switch (strGcodeLine[index]) 
			{

			case 'G':
				tempGcmd = (int)GetGcodeVariableValue(strGcodeLine, index);
				m_bMathematicalCurve = FALSE;
				if (tempGcmd <4)
				{
					m_iNextMoveType =tempGcmd;
				}
				if (tempGcmd == 69)  // Mean no automatic deceleration in corner moving
				{
					m_bAutoCornerSmooth = FALSE;	
				}
				if (tempGcmd == 96)  // Automatic deceleration in corner moving
				{
					m_bAutoCornerSmooth = TRUE;	
				}
				if (tempGcmd == 62)  // Mathematical Curve machining detected
				{
					m_bMathematicalCurve = TRUE;
					m_iNextMoveType =4; // Mathematical Curve machining detected
				}
				if (tempGcmd == 63)  // Mathematical Curve machining detected
				{
					m_bMathematicalCurve = TRUE;
					m_iNextMoveType =4; // Mathematical Curve machining detected
				}
				break;

			case 'X':
				m_fNextCNCEndX =GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;

				if (m_iMathCurveNumber == TUNINGPROCESS)
				{
					m_fMath_r = m_fNextCNCEndX;
					m_iTuningMotor = LINEAR_MOTOR_X;
				}

				break;
			case 'Y':

				m_fNextCNCEndY =GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;

				if (m_iMathCurveNumber == TUNINGPROCESS)
				{
					m_fMath_r = m_fNextCNCEndY;
					m_iTuningMotor = LINEAR_MOTOR_Y1;
				}
				break;
			case 'Z':

				//			m_fCNCEndZ =(int)GetGcodeVariableValue(strRead, index);
				m_fNextCNCEndZ =GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;

				if (m_iMathCurveNumber == TUNINGPROCESS)
				{
					m_fMath_r = m_fNextCNCEndZ;
					m_iTuningMotor = LINEAR_MOTOR_Z;
				}
				break;
			case 'C':

				//			m_fCNCEndZ =(int)GetGcodeVariableValue(strRead, index);
			    m_fNextCNCEndC =GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;

				if (m_iMathCurveNumber == TUNINGPROCESS)
				{
					m_fMath_r = m_fNextCNCEndC;
					m_iTuningMotor = LINEAR_MOTOR_C;
				}
				break;
			case 'A':

				//			m_fCNCEndZ =(int)GetGcodeVariableValue(strRead, index);
				m_fNextCNCEndA =GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;

				if (m_iMathCurveNumber == TUNINGPROCESS)
				{
					m_fMath_r = m_fNextCNCEndA;
					m_iTuningMotor = LINEAR_MOTOR_A1;
				}
				break;

			case 'I':

				m_fNextCNCI = GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;
				break;
			case 'J':

				m_fNextCNCJ = GetGcodeVariableValue(strGcodeLine, index);
				IsNextGcodeMovement = TRUE;
				break;
			case 'R':
				if (strGcodeLine[index+1]=='E')
				{
					index = index +1;
					m_fMath_Repeat =GetGcodeVariableValue(strGcodeLine, index);
				} else
				{
					m_fNextCNCRadius = GetGcodeVariableValue(strGcodeLine, index);
					IsNextGcodeMovement = TRUE;	
				}


				break;
			case 'M':

				m_iNextGcodeM = (int)GetGcodeVariableValue(strGcodeLine, index);
				// Change mt_ContourWorkPieceCordinate reference
				break;
			case 'F':

				if (!m_bMathematicalCurve)
				{
					m_fNextGcodeF = GetGcodeVariableValue(strGcodeLine, index);
					m_fNextCNCFeedRate = m_fNextGcodeF;
				}else
				{
					if (strGcodeLine[index+1]=='E') //Figure Eight Curve FEC
					{
						index = index +3;
						m_iMathCurveNumber = EIGHTCURVE;
					}
					else
					{
						index = index +1;
					}
				}
				break;
			case 'S':

				//			m_fGcodeS = GetGcodeVariableValue(strRead, index);

				break;
			case 'V':// Min distance between two velocity at angle point
				if (strGcodeLine[index+2]=='A')
				{
					index = index +3;
					vec_CNCVMAX(tempGcmd-60) = GetGcodeVariableValue(strGcodeLine, index);
				}
				if (strGcodeLine[index+2]=='I')
				{
					index = index +3;
					m_fCNCVMIN = GetGcodeVariableValue(strGcodeLine, index);
				}
				//			m_fGcodeS = GetGcodeVariableValue(strRead, index);

				break;
			case 'W':// Tuning process omega of the reference
				if (m_iMathCurveNumber == TUNINGPROCESS)
				{
					m_fMath_w = GetGcodeVariableValue(strGcodeLine, index);
					m_fMath_Time = m_fMath_Repeat*2.0*PI/m_fMath_w;// Repeat turning process in to m_fMath_Repeat time
				}
				//			m_fGcodeS = GetGcodeVariableValue(strRead, index);

				break;
			case 'P':
				if (strGcodeLine[index+2]=='A')
				{
					index = index +3;
					vec_CNCPMAX(tempGcmd-60) = GetGcodeVariableValue(strGcodeLine, index);
				}
				if (strGcodeLine[index+1]=='H')
				{
					index = index +1;
					m_fNextCNCPH = GetGcodeVariableValue(strGcodeLine, index);
				}
				if (strGcodeLine[index+1]=='E')
				{
					index = index +1;
					m_fNextCNCPE = GetGcodeVariableValue(strGcodeLine, index);
				}
				break;
			case 'a':
				m_fMath_a =GetGcodeVariableValue(strGcodeLine, index);	
				break;
			case 'n':
				m_fMath_n =GetGcodeVariableValue(strGcodeLine, index);	
				break;
			case 'c':
				m_fMath_c =GetGcodeVariableValue(strGcodeLine, index);	
				break;
			case 'b':
				m_fMath_b =GetGcodeVariableValue(strGcodeLine, index);	
				break;
			case 'T':
				if (strGcodeLine[index+1]=='R')
				{
					index = index +3;
					m_iMathCurveNumber = TRIFOLIUM;
				} else
					if (strGcodeLine[index+1]=='U')
					{
						index = index +3;
						m_iMathCurveNumber = TUNINGPROCESS;
					} else
					{
						m_fMath_Time =GetGcodeVariableValue(strGcodeLine, index);	
					}
				break;
			case 'L':
				index = index +3;
				m_iMathCurveNumber = LISSAJOUS;
				break;
			case ' ':

				index++;

				break;
			case '/': case ';':

				EndofString = TRUE	;
             
				break;
			default:
				EndofString = TRUE	;
			}
		}
		else
		{
			EndofString = TRUE	;
		}
	}
	return IsNextGcodeMovement;
}
bool FiveAxisCNC::AnalyseGcodeLine(System::String^ strGcodeLine) 
{
// 	System::String^ GcodeLineString;
// 	GcodeLineString  = strGcodeLine;
	bool EndofString= FALSE, IsGcodeMovement = FALSE;
	unsigned int index = 0;
	EndofString= FALSE;
	int tempGcmd = 0;
//   char testChar = strGcodeLine[strGcodeLine->Length-1];
	while (!EndofString)
	{
		if (index < (unsigned int)strGcodeLine->Length)
		{
			switch (strGcodeLine[index]) 
			{

			case 'G':
				tempGcmd = (int)GetGcodeVariableValue(strGcodeLine, index);
				if (tempGcmd <4)
				{
					m_iMoveType =tempGcmd;
				}

				break;

			case 'X':

				m_fCNCEndX =GetGcodeVariableValue(strGcodeLine, index);
				IsGcodeMovement = TRUE;

				break;
			case 'Y':

				m_fCNCEndY =GetGcodeVariableValue(strGcodeLine, index);
				IsGcodeMovement = TRUE;
				break;
			case 'Z':

				//			m_fCNCEndZ =(int)GetGcodeVariableValue(strRead, index);
				m_fCNCEndZ =GetGcodeVariableValue(strGcodeLine, index);
				IsGcodeMovement = TRUE;
				break;
			case 'I':

				m_fCNCI = GetGcodeVariableValue(strGcodeLine, index);
				IsGcodeMovement = TRUE;
				break;
			case 'J':

				m_fCNCJ = GetGcodeVariableValue(strGcodeLine, index);
				IsGcodeMovement = TRUE;
				break;
			case 'R':

				m_fCNCRadius = GetGcodeVariableValue(strGcodeLine, index);
				IsGcodeMovement = TRUE;
				break;
			case 'M':

				m_iGcodeM = (int)GetGcodeVariableValue(strGcodeLine, index);
				// Change mt_ContourWorkPieceCordinate reference
				break;
			case 'F':

				m_fGcodeF = GetGcodeVariableValue(strGcodeLine, index);
				m_fCNCFeedRate = m_fGcodeF;
				break;
			case 'S':

				//			m_fGcodeS = GetGcodeVariableValue(strRead, index);

				break;
			case 'V':// Min distance between two velocity at angle point
				if (strGcodeLine[index+1]=='M')
				{
					index = index +3;
					vec_CNCVMAX(tempGcmd-60) = GetGcodeVariableValue(strGcodeLine, index);
				}
				
				//			m_fGcodeS = GetGcodeVariableValue(strRead, index);

				break;
			case 'P':
				if (strGcodeLine[index+1]=='M')
				{
					index = index +3;
					vec_CNCPMAX(tempGcmd-60) = GetGcodeVariableValue(strGcodeLine, index);
				}
				if (strGcodeLine[index+1]=='H')
				{
					index = index +1;
					m_fCNCPH = GetGcodeVariableValue(strGcodeLine, index);
				}
				if (strGcodeLine[index+1]=='E')
				{
					index = index +1;
					m_fCNCPE = GetGcodeVariableValue(strGcodeLine, index);
				}
				break;
			case ' ':

				index++;

				break;
			case '/':

				EndofString = TRUE	;

				break;
			default:
				EndofString = TRUE	;
			}
		}
		else
		{
			EndofString = TRUE	;
		}
	}
	return IsGcodeMovement;
}
void FiveAxisCNC::GetNextGCodeLine() 
{
	bool FoundNextPath= FALSE;
	m_bPathFinish = FALSE;
	System::String^ strRead;
	double dAB = 0.0,dOH= 0.0,sABC = 0.0, refTime = 0.0,refAngle = 0.0;

	m_iMoveType = m_iNextMoveType;
//	m_fexpRunT = m_fNextexpRunT;

	m_fAccFirstTime = m_fNextAccFirstTime;
	m_fCNCPosOneX = m_fNextCNCPosOneX;m_fCNCPosTwoX = m_fNextCNCPosTwoX;
//	m_fCNCPosThreeX = m_fNextCNCPosThreeX;
	m_fCNCPosOneY = m_fNextCNCPosOneY;m_fCNCPosTwoY = m_fNextCNCPosTwoY;
	m_fCNCPosOneZ = m_fNextCNCPosOneZ;m_fCNCPosTwoZ = m_fNextCNCPosTwoZ;
//	m_fCNCPosThreeY = m_fNextCNCPosThreeY;

	m_fCNCStartVelX = m_fNextCNCStartVelX;m_fCNCMidVelX = m_fNextCNCMidVelX; m_fCNCEndVelX = m_fNextCNCEndVelX;
	m_fCNCStartAccX = m_fNextCNCStartAccX;//m_fCNCEndAccX = m_fNextCNCEndAccX;
	m_fCNCStartVelY = m_fNextCNCStartVelY;m_fCNCMidVelY = m_fNextCNCMidVelY; m_fCNCEndVelY = m_fNextCNCEndVelY;
	m_fCNCStartAccY = m_fNextCNCStartAccY;//m_fCNCEndAccY = m_fNextCNCEndAccY;
	m_fCNCStartVelZ = m_fNextCNCStartVelZ;m_fCNCMidVelZ = m_fNextCNCMidVelZ; m_fCNCEndVelZ = m_fNextCNCEndVelZ;
	m_fCNCStartAccZ = m_fNextCNCStartAccZ;//m_fCNCEndAccY = m_fNextCNCEndAccY;

	m_fCNCAngleOne = m_fNextCNCAngleOne;m_fCNCAngleTwo = m_fNextCNCAngleTwo;m_fCNCEndAngle = m_fNextCNCEndAngle;  // need
//	m_fCNCAngleThree = m_fNextCNCAngleThree;
	m_fCNCStartAngleVel = m_fNextCNCStartAngleVel;m_fCNCMidAngleVel = m_fNextCNCMidAngleVel;
	m_fCNCStartAngleAcc = m_fNextCNCStartAngleAcc;


	m_fCNCStartX = m_fNextCNCStartX;
	m_fCNCStartY = m_fNextCNCStartY;
	m_fCNCStartZ = m_fNextCNCStartZ;
	m_fCNCEndX = m_fNextCNCEndX;                // need
	m_fCNCEndY = m_fNextCNCEndY;
	m_fCNCEndZ = m_fNextCNCEndZ;
	m_fLengthX = m_fNextLengthX;
	m_fLengthY = m_fNextLengthY;
	m_fLengthZ = m_fNextLengthZ;
	m_fLengthMovement = m_fNextLengthMovement;
	m_fCNCI = m_fNextCNCI;
	m_fCNCJ = m_fNextCNCJ;
	m_fCNCRadius = m_fNextCNCRadius;
	m_fCNCStartAngle = m_fNextCNCStartAngle;
	m_fOmg = m_fNextOmg;
	m_fCNCFeedRate = m_fNextCNCFeedRate;

	m_fAccFirstTime = m_fNextAccFirstTime;
	m_fCNCPH = m_fNextCNCPH;
	m_fCNCPE = m_fNextCNCPE;
//	m_fStartddotOmg = m_fNextStartddotOmg;
//	m_fStartddotAcc = m_fNextStartddotAcc;
//	m_CNCStartRefVel = m_NextCNCStartRefVel;
//	m_CNCEndRefVel = m_NextCNCEndRefVel;



	m_fNextCNCStartX = m_fCNCEndX; 
	m_fNextCNCStartY = m_fCNCEndY;
	m_fNextCNCStartZ = m_fCNCEndZ;
	m_fNextCNCRadius = 999999.1;
	m_fNextCNCI = 0.0;

	//	while ((!FoundNextPath)&&((strRead = GcodeFileStream->ReadLine()) != nullptr))
	strRead = "NoThing";
	while ((!FoundNextPath)&&(strRead != nullptr))
	{	
		// 		inFGcode.getline(strRead, sizeof(strRead));//Read Number Line
		// 		strGcodeRead = CString(strRead);
		// 		sscanf.
		// 		strGcodeReadsscanf(strRead,"%s",strGcodeRead);
		strRead = GcodeFileStream->ReadLine();
		if ((strRead != nullptr))
			{
				FoundNextPath = AnalyseNextGcodeLine(strRead);
			}

	}
	if ((strRead == nullptr)||(!FoundNextPath))
	{// If finished setup data for move to end point
// 		if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
// 		{
// 			m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCEndX-m_fCNCPosTwoX)*(m_fCNCEndX-m_fCNCPosTwoX)
// 				+(m_fCNCEndY-m_fCNCPosTwoY)*(m_fCNCEndY-m_fCNCPosTwoY))) /m_fCNCFeedRate;
// 		}else
// 		{
// 			m_fAccEndTime = m_fAccFirstTime;
// 		}
// 		m_fexpRunT = m_fAccEndTime;
		m_bNextGcodeFINISH = TRUE;
// If finished setup data for move to end point
		if (m_bAutoCornerSmooth) // Automatic deceleration in corner moving
		{
			switch (m_iMoveType)
			{
			case 0:
				break;
			case 1:
				if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCEndX-m_fCNCPosTwoX)*(m_fCNCEndX-m_fCNCPosTwoX)
						+(m_fCNCEndY-m_fCNCPosTwoY)*(m_fCNCEndY-m_fCNCPosTwoY))) /m_fCNCFeedRate;
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}
				m_fexpRunT = m_fAccEndTime;
				break;
			case 2: case 3: case 4:
				if (fabs(m_fCNCMidAngleVel) > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCEndAngle-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}
				m_fexpRunT = m_fAccEndTime;
				break;
			default:
				;
			}// end switch
		}else //  no automatic deceleration in corner moving 
		{
			switch (m_iMoveType)
			{
			case 0:
				break;
			case 1:
				if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCEndX-m_fCNCPosTwoX)*(m_fCNCEndX-m_fCNCPosTwoX)
						+(m_fCNCEndY-m_fCNCPosTwoY)*(m_fCNCEndY-m_fCNCPosTwoY))) /m_fCNCFeedRate;
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}
				m_fexpRunT = m_fAccEndTime;
				break;
			case 2: case 3:
				if (fabs(m_fCNCMidAngleVel) > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCEndAngle-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}
				m_fexpRunT = m_fAccEndTime;
				break;
			case 4:
				m_fAccEndTime = m_fAccFirstTime + m_fMath_Time;
				m_fexpRunT = m_fAccEndTime;
				m_fCNCEndX = 0.0;
				m_fCNCEndY = 0.0;
				m_fCNCEndZ = 0.0;
				break;
			default:
				;
			}// end switch
		}

	}
	if (FoundNextPath)
	{
		//		m_fexpTnow = 0.0;
		//		m_fexpRunT = 0.0;
		//		m_fexpRunT= m_fexpRunTPre;
#pragma region  Calculate omega and start end velocity
		switch (m_iNextMoveType) 
		{
		case 0:

			//	m_iMoveType =(int)GetGcodeVariableValue(strRead, index);

			break;
		case 1:
			m_fNextLengthX = m_fNextCNCEndX- m_fNextCNCStartX;
			m_fNextLengthY =  m_fNextCNCEndY- m_fNextCNCStartY;
			m_fNextLengthZ =  m_fNextCNCEndZ- m_fNextCNCStartZ;
			m_fNextLengthMovement =  sqrtf(m_fNextLengthX*m_fNextLengthX+m_fNextLengthY*m_fNextLengthY+m_fNextLengthZ*m_fNextLengthZ);

			//2D axis calculation
			//m_fNextCNCStartAngle = atan2f(m_fNextLengthY,m_fNextLengthX);


			break;
		case 2:
			//m_fCNCStartAngle
			//	m_fOmg
			//NowGCode
			// Fin the cut point between  2 two circle with center(x1,y1) Radius R va Center (x3,y3) Radius R
			//(x-x1)^2 + (y-y1)^2 = R^2;
			//(x-x3)^2 + (y-y3)^2 = R^2;
			//             x1 = P1.X; x3 = P3.X; y1 = P1.Y; y3 = P3.Y;
			//             radius = inRadius;
			//             d13 = Math.Sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
			//             sABC = (d13 / 4) * Math.Sqrt((4 * radius * radius) - d13 * d13);// Not is S ABO
			//             hArc = radius - ((2 * sABC )/ d13);
			//             if (NowGCode == 3)
			//             {
			//                 P2.X = (x1 + x3) / 2 + hArc * (y1 - y3) / d13;
			//                 P2.Y = (y1 + y3) / 2 + hArc * (x1 - x3) / d13;
			//             }else
			// 			{
			// 				P2.X = (x1 + x3) / 2 + hArc * (y3 - y1) / d13;
			// 				P2.Y = (y1 + y3) / 2 + hArc * (x3 - x1) / d13;
			//                 }
			if (m_fNextCNCRadius< 99999.0)
			{
				dAB = sqrt((m_fNextCNCEndX - m_fNextCNCStartX) *(m_fNextCNCEndX - m_fNextCNCStartX)+ (m_fNextCNCEndY - m_fNextCNCStartY) *(m_fNextCNCEndY - m_fNextCNCStartY)) ;
				dOH = sqrt(m_fNextCNCRadius*m_fNextCNCRadius - dAB*dAB/4.0);
				if (m_fNextCNCRadius>DOUBLE_TOLERANCE)

				{
					m_fNextCNCI = ((m_fNextCNCEndX + m_fNextCNCStartX)/2.0) + (dOH * (m_fNextCNCEndY - m_fNextCNCStartY) / dAB);
					m_fNextCNCJ = ((m_fNextCNCEndY + m_fNextCNCStartY) / 2.0) - (dOH * (m_fNextCNCEndX - m_fNextCNCStartX) / dAB);

				}
				else
				{
					m_fNextCNCRadius = -m_fNextCNCRadius;
					m_fNextCNCI = ((m_fNextCNCEndX + m_fNextCNCStartX)/2.0) - (dOH * (m_fNextCNCEndY - m_fNextCNCStartY) / dAB);
					m_fNextCNCJ = ((m_fNextCNCEndY + m_fNextCNCStartY) / 2.0) + (dOH * (m_fNextCNCEndX - m_fNextCNCStartX) / dAB);
				}
				//				m_fCNCRadius = sqrt((m_fCNCEndX-m_fCNCI)*(m_fCNCEndX-m_fCNCI) +(m_fCNCEndY-m_fCNCJ)*(m_fCNCEndY-m_fCNCJ));
				m_fNextCNCStartAngle = atan2f(m_fNextCNCStartY-m_fNextCNCJ, m_fNextCNCStartX-m_fNextCNCI);
				m_fNextCNCEndAngle = atan2f(m_fNextCNCEndY-m_fNextCNCJ,m_fNextCNCEndX-m_fNextCNCI);	

				if (m_fNextCNCStartAngle <m_fNextCNCEndAngle)
				{
					m_fNextCNCStartAngle = m_fNextCNCStartAngle+ 2.0*PI;
				}
				m_fNextexpRunT =  m_fNextexpRunT+(fabs(m_fNextCNCEndAngle- m_fNextCNCStartAngle))*m_fNextCNCRadius/ m_fNextCNCFeedRate;
				m_fNextOmg = m_fNextCNCFeedRate/m_fNextCNCRadius;
			}
			else
			{
				m_fNextCNCRadius = sqrt((m_fNextCNCEndX-m_fNextCNCI)*(m_fNextCNCEndX-m_fNextCNCI) +(m_fNextCNCEndY-m_fNextCNCJ)*(m_fNextCNCEndY-m_fNextCNCJ));
				m_fNextCNCStartAngle = atan2f(m_fNextCNCStartY-m_fNextCNCJ, m_fNextCNCStartX-m_fNextCNCI);
				m_fNextCNCEndAngle = atan2f(m_fNextCNCEndY-m_fNextCNCJ,m_fNextCNCEndX-m_fNextCNCI);	

				if (m_fNextCNCStartAngle <m_fNextCNCEndAngle)
				{
					m_fNextCNCStartAngle = m_fNextCNCStartAngle+ 2.0*PI;
				}
				m_fNextexpRunT =  m_fNextexpRunT+(fabs(m_fNextCNCEndAngle- m_fNextCNCStartAngle))*m_fNextCNCRadius/ m_fNextCNCFeedRate;
				m_fNextOmg = m_fNextCNCFeedRate/m_fNextCNCRadius;

			}
			m_fNextOmg = -m_fNextOmg;// Clock Wise m_fNextOmg<0;

// 			m_NextCNCStartRefVel.X = (-m_fOmg)*(m_fNextCNCStartY- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
// 			m_NextCNCStartRefVel.Y1 = (m_fOmg)*(m_fNextCNCStartX-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
// 
// 			m_NextCNCEndRefVel.X = (-m_fOmg)*(m_fNextCNCEndY- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
// 			m_NextCNCEndRefVel.Y1 = (m_fOmg)*(m_fNextCNCEndX-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s

			break;
		case 3:

			if (m_fNextCNCRadius< 99999.0)
			{
				dAB = sqrt((m_fNextCNCEndX - m_fNextCNCStartX) *(m_fNextCNCEndX - m_fNextCNCStartX)+ (m_fNextCNCEndY - m_fNextCNCStartY) *(m_fNextCNCEndY - m_fNextCNCStartY)) ;
				dOH = sqrt(m_fNextCNCRadius*m_fNextCNCRadius - dAB*dAB/4.0);
				if (m_fNextCNCRadius>DOUBLE_TOLERANCE)

				{
					m_fNextCNCI = ((m_fNextCNCEndX + m_fNextCNCStartX)/2.0) - (dOH * (m_fNextCNCEndY - m_fNextCNCStartY) / dAB);
					m_fNextCNCJ = ((m_fNextCNCEndY + m_fNextCNCStartY) / 2.0) + (dOH * (m_fNextCNCEndX - m_fNextCNCStartX) / dAB);

				}
				else
				{
					m_fNextCNCRadius = -m_fNextCNCRadius;
					m_fNextCNCI = ((m_fNextCNCEndX + m_fNextCNCStartX)/2.0) + (dOH * (m_fNextCNCEndY - m_fNextCNCStartY) / dAB);
					m_fNextCNCJ = ((m_fNextCNCEndY + m_fNextCNCStartY) / 2.0) - (dOH * (m_fNextCNCEndX - m_fNextCNCStartX) / dAB);
				}
				//				m_fNextCNCRadius = sqrt((m_fCNCEndX-m_fCNCI)*(m_fCNCEndX-m_fCNCI) +(m_fCNCEndY-m_fCNCJ)*(m_fCNCEndY-m_fCNCJ));
				m_fNextCNCStartAngle = atan2f(m_fNextCNCStartY-m_fNextCNCJ, m_fNextCNCStartX-m_fNextCNCI);
				m_fNextCNCEndAngle = atan2f(m_fNextCNCEndY-m_fNextCNCJ,m_fNextCNCEndX-m_fNextCNCI);	

				if (m_fNextCNCStartAngle >m_fNextCNCEndAngle)
				{
					m_fNextCNCEndAngle = m_fNextCNCEndAngle+ 2.0*PI;
				}
//				m_fNextexpRunT =  m_fNextexpRunT+(fabs(m_fNextCNCEndAngle- m_fNextCNCStartAngle))*m_fNextCNCRadius/ m_fNextCNCFeedRate;
				m_fNextOmg = m_fNextCNCFeedRate/m_fNextCNCRadius;
			}
			else
			{
				m_fNextCNCRadius = sqrt((m_fNextCNCEndX-m_fNextCNCI)*(m_fNextCNCEndX-m_fNextCNCI) +(m_fNextCNCEndY-m_fNextCNCJ)*(m_fNextCNCEndY-m_fNextCNCJ));
				m_fNextCNCStartAngle = atan2f(m_fNextCNCStartY-m_fNextCNCJ, m_fNextCNCStartX-m_fNextCNCI);
				m_fNextCNCEndAngle = atan2f(m_fNextCNCEndY-m_fNextCNCJ,m_fNextCNCEndX-m_fNextCNCI);	

				if (m_fNextCNCStartAngle >m_fNextCNCEndAngle)
				{
					m_fNextCNCEndAngle = m_fNextCNCEndAngle+ 2.0*PI;
				}
//				m_fNextexpRunT =  m_fNextexpRunT+(fabs(m_fNextCNCEndAngle- m_fNextCNCStartAngle))*m_fNextCNCRadius/ m_fNextCNCFeedRate;
				m_fNextOmg = m_fNextCNCFeedRate/m_fNextCNCRadius;
			}

// 			m_NextCNCStartRefVel.X = (-m_fOmg)*(m_fNextCNCStartY- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
// 			m_NextCNCStartRefVel.Y1 = (m_fOmg)*(m_fNextCNCStartX-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
// 
// 			m_NextCNCEndRefVel.X = (-m_fOmg)*(m_fNextCNCEndY- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
// 			m_NextCNCEndRefVel.Y1 = (m_fOmg)*(m_fNextCNCEndX-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
			break;
		case 4:	
			m_fNextCNCEndX = 0.0;
			m_fNextCNCEndY = 0.0;
			m_fNextCNCEndZ = 0.0;
			break;
		default:
			;
		}// end switch
#pragma endregion  Calculate omega and start end velocity
		// Calculate NextCNC Start Velocity X and  NextCNC end VelocityX 

		switch (m_iNextMoveType)
		{
		case 0:
			break;
		case 1:
			// 2D 2 axis calculation
			//m_fNextCNCStartVelX =  m_fNextCNCFeedRate* cosf(m_fNextCNCStartAngle);
			//m_fNextCNCStartVelY = m_fNextCNCFeedRate* sinf(m_fNextCNCStartAngle);
			// 3D  3-5 axis calculation
			m_fNextCNCStartVelX =  m_fNextCNCFeedRate* m_fNextLengthX/m_fNextLengthMovement;
			m_fNextCNCStartVelY = m_fNextCNCFeedRate*  m_fNextLengthY/m_fNextLengthMovement;
			m_fNextCNCStartVelZ = m_fNextCNCFeedRate*  m_fNextLengthZ/m_fNextLengthMovement;

			m_fNextCNCMidVelX = m_fNextCNCStartVelX;
			m_fNextCNCMidVelY = m_fNextCNCStartVelY;
			m_fNextCNCMidVelZ = m_fNextCNCStartVelZ;
			m_fNextCNCEndVelX = m_fNextCNCStartVelX;
			m_fNextCNCEndVelY = m_fNextCNCStartVelY;
			m_fNextCNCEndVelZ = m_fNextCNCStartVelZ;
			//			m_fNextexpRunT = m_fNextexpRunT+ sqrt(m_fNextLengthX*m_fNextLengthX+m_fNextLengthY*m_fNextLengthY)/m_fNextCNCFeedRate;
			// 			// Auto turing Acceleration at corner 
			break;
		case 2: case 3:
			m_fNextCNCStartVelX = (-m_fNextOmg)*(m_fNextCNCStartY- m_fNextCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
			m_fNextCNCStartVelY = (m_fNextOmg)*(m_fNextCNCStartX-m_fNextCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
// 			m_fNextCNCMidVelX = m_fNextCNCStartVelX; // no need to calculte
// 			m_fNextCNCMidVelY = m_fNextCNCStartVelY;
			m_fNextCNCEndVelX = (-m_fNextOmg)*(m_fNextCNCEndY- m_fNextCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
			m_fNextCNCEndVelY = (m_fNextOmg)*(m_fNextCNCEndX-m_fNextCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s

			m_fNextCNCStartAngleVel = m_fNextOmg;
			m_fNextCNCMidAngleVel = m_fNextOmg;
			m_fNExtCNCEndAngleVel = m_fNextOmg;
			
	//		m_fNextCNCStartAngleVel
			//			m_fNextexpRunT = m_fNextexpRunT+ sqrt(m_fNextLengthX*m_fNextLengthX+m_fNextLengthY*m_fNextLengthY)/m_fNextCNCFeedRate;
			// 			// Auto turing Acceleration at corner 
			break;
		case 4:
			m_fNextCNCStartVelX = 0.0;
			m_fNextCNCStartVelY = 0.0;
			m_fNextCNCMidVelX = m_fNextCNCStartVelX;
			m_fNextCNCMidVelY = m_fNextCNCStartVelY;
			m_fNextCNCEndVelX = m_fNextCNCStartVelX;
			m_fNextCNCEndVelY = m_fNextCNCStartVelY;
			m_fNextCNCEndX = 0.0;
			m_fNextCNCEndY = 0.0;
			m_fNextCNCEndZ = 0.0;
			break;
		default:
			;
		}// end switch about Calculate NextCNC Start Velocity X and  NextCNC end VelocityX 

		vec_VelChange(0) = (fabs(m_fNextCNCStartVelX- m_fCNCEndVelX) - vec_CNCVMAX(0))/ m_fSampTimeRef;
		vec_VelChange(1) =  (fabs(m_fNextCNCStartVelY- m_fCNCEndVelY) - vec_CNCVMAX(1))/ m_fSampTimeRef;
		vec_VelChange(3) =  (fabs(m_fNextCNCStartVelZ- m_fCNCEndVelZ) - vec_CNCVMAX(3))/ m_fSampTimeRef;
		if (OutOfLimitedAcceleration())
		{
// 
// 			//				m_fCNCPEX = m_fCNCPE* cosf(m_fNextCNCStartAngle);
// 			m_fNextCNCPHX = m_fNextCNCPH* cosf(m_fNextCNCStartAngle);
// 			m_fNextCNCPHY = m_fNextCNCPH* sinf(m_fNextCNCStartAngle);
// 
// 			//				m_fCNCEndVelX = m_fCNCMidVelX +(vec_VelChange(0)-sign( vec_VelChange(0) )*vec_CNCVMIN(0))/2.0 ;
// 			m_fNextCNCStartVelX = m_fCNCVMIN* cosf(m_fNextCNCStartAngle);
// 			m_fNextCNCStartVelY = m_fCNCVMIN* sinf(m_fNextCNCStartAngle);
			
		
			switch (m_iMoveType)
			{
			case 0:
				break;
			case 1:
				// 		vec_refr_1(0) = (-m_fOmg)*(m_CNCRefPos.Y- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
				// 		vec_refr_1(1) = (m_fOmg)*(m_CNCRefPos.X-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
				//2Axis control
// 				m_fCNCEndVelX = m_fCNCVMIN* cosf(m_fCNCStartAngle);
// 				m_fCNCEndVelY = m_fCNCVMIN* sinf(m_fCNCStartAngle);
// 
// 				m_fCNCEndAccX = -m_fCNCPE* cosf(m_fCNCStartAngle);// Decrease to Zero
// 				m_fCNCEndAccY = -m_fCNCPE* sinf(m_fCNCStartAngle);

				m_fCNCEndVelX = m_fCNCVMIN* m_fLengthX/m_fLengthMovement;
				m_fCNCEndVelY = m_fCNCVMIN* m_fLengthY/m_fLengthMovement;
				m_fCNCEndVelZ = m_fCNCVMIN* m_fLengthZ/m_fLengthMovement;

				m_fCNCEndAccX = -m_fCNCPE* m_fLengthX/m_fLengthMovement;
				m_fCNCEndAccY = -m_fCNCPE* m_fLengthY/m_fLengthMovement;
				m_fCNCEndAccZ = -m_fCNCPE* m_fLengthZ/m_fLengthMovement;

				refTime = 0.0;
					if (m_fCNCPE>0.0)
					{
						refTime =  fabs((m_fCNCFeedRate - m_fCNCVMIN)/m_fCNCPE);
					}
				if (!m_bAutoCornerSmooth) {refTime = 0.0;}   //no automatic deceleration in corner moving
				
				//					m_fCNCPEY = (m_fCNCEndVelY - m_fCNCMidVelY)/refTime;
				//		m_fAccEndTime = m_fAccFirstTime+ (m_fCNCEndVelX - m_fCNCMidVelX)/m_fCNCPE;
				//2 Axis control
// 				m_fCNCPosThreeX = m_fCNCEndX - (m_fCNCFeedRate*refTime-m_fCNCPE*refTime*refTime/2.0)* cosf(m_fCNCStartAngle);
// 				m_fCNCPosThreeY = m_fCNCEndY - (m_fCNCFeedRate*refTime-m_fCNCPE*refTime*refTime/2.0)* sinf(m_fCNCStartAngle);
				
				m_fCNCPosThreeX = m_fCNCEndX - (m_fCNCMidVelX*refTime-m_fCNCEndAccX*refTime*refTime/2.0);
				m_fCNCPosThreeY = m_fCNCEndY - (m_fCNCMidVelY*refTime-m_fCNCEndAccY*refTime*refTime/2.0);
				m_fCNCPosThreeZ = m_fCNCEndZ - (m_fCNCMidVelZ*refTime-m_fCNCEndAccZ*refTime*refTime/2.0);

				if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCPosThreeX-m_fCNCPosTwoX)*(m_fCNCPosThreeX-m_fCNCPosTwoX)
						+(m_fCNCPosThreeY-m_fCNCPosTwoY)*(m_fCNCPosThreeY-m_fCNCPosTwoY)
						+(m_fCNCPosThreeZ-m_fCNCPosTwoZ)*(m_fCNCPosThreeZ-m_fCNCPosTwoZ))) /m_fCNCFeedRate;
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}

				m_fexpRunT = m_fAccEndTime+ refTime;

// 				// Cal culate for next Ref
// 				refTime = fabs((m_fNextCNCFeedRate - m_fCNCVMIN)/m_fNextCNCPH);
// 				m_fNextAccFirstTime = m_fexpRunT+refTime;
// 				m_fNextCNCPosOneX = m_fNextCNCStartX;m_fNextCNCPosOneY = m_fNextCNCStartY; 
// 				m_fNextCNCPosTwoX = m_fNextCNCPosOneX + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* cosf(m_fNextCNCStartAngle);
// 				m_fNextCNCPosTwoY =  m_fNextCNCPosOneY + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* sinf(m_fNextCNCStartAngle);
// 				m_fNextCNCStartAccX = m_fNextCNCPH* cosf(m_fNextCNCStartAngle);// Increase
// 				m_fNextCNCStartAccY = m_fNextCNCPH* sinf(m_fNextCNCStartAngle);
				break;
			case 2: case 3:
				m_fCNCEndVelX = m_fCNCVMIN* cosf(m_fCNCEndAngle);
				m_fCNCEndVelY = m_fCNCVMIN* sinf(m_fCNCEndAngle);

				// 					m_fCNCEndVelY = m_fCNCEndVelX* tanf(m_fCNCEndAngle);
				// 					m_fCNCPEY = (m_fCNCEndVelY - m_fCNCMidVelY)/refTime;

				m_fCNCEndAngleVel = m_fCNCVMIN/m_fCNCRadius;// Rad/second
				m_fCNCEndAngleAcc = Rmsign(m_fCNCEndAngleVel -m_fCNCMidAngleVel)*m_fCNCPE/m_fCNCRadius;

				refTime = 0.0;
				if (m_fCNCPE>0.0)
				{
					refTime =  (m_fCNCEndAngleVel -m_fCNCMidAngleVel)/m_fCNCEndAngleAcc;// no need fabs
				}
				
				refAngle = m_fCNCEndAngle - (m_fCNCMidAngleVel*refTime+m_fCNCEndAngleAcc*refTime*refTime/2.0);
				m_fCNCAngleThree = refAngle;

				m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCAngleThree-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
				m_fexpRunT = m_fAccEndTime+ refTime;
// 				// Cal culate for next Ref
// 				refTime = fabs((m_fNextCNCFeedRate - m_fCNCVMIN)/m_fCNCPH);
// 				m_fNextAccFirstTime = m_fexpRunT+refTime;
// 				m_fNextCNCPosOneX = m_fNextCNCStartX;m_fNextCNCPosOneY = m_fNextCNCStartY; 
// 				m_fNextCNCPosTwoX = m_fNextCNCPosOneX + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* cosf(m_fNextCNCStartAngle);
// 				m_fNextCNCPosTwoY =  m_fNextCNCPosOneY + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* sinf(m_fNextCNCStartAngle);
				break;
			case 4:
				m_fNextCNCEndX = 0.0;
				m_fNextCNCEndY = 0.0;
				m_fNextCNCEndZ = 0.0;
				break;
			default:
				;
			}// End switch

// switch about Calculate Next ref m_fNextCNCStartVelX
			switch (m_iNextMoveType)
			{
			case 0:
				break;
			case 1:
				// Cal culate for next Ref
				//2 Axis
// 				m_fNextCNCStartVelX = m_fCNCVMIN* cosf(m_fNextCNCStartAngle);
// 				m_fNextCNCStartVelY = m_fCNCVMIN* sinf(m_fNextCNCStartAngle);

				m_fNextCNCStartVelX = m_fCNCVMIN*  m_fNextLengthX/m_fNextLengthMovement;
				m_fNextCNCStartVelY = m_fCNCVMIN*  m_fNextLengthY/m_fNextLengthMovement;
				m_fNextCNCStartVelZ = m_fCNCVMIN*  m_fNextLengthZ/m_fNextLengthMovement;

				refTime = 0.0;
				if (m_fNextCNCPH>0.0)
				{
					refTime = fabs((m_fNextCNCFeedRate - m_fCNCVMIN)/m_fNextCNCPH);
				}
				if (!m_bAutoCornerSmooth) {refTime = 0.0;}   //no automatic deceleration in corner moving
				m_fNextAccFirstTime = m_fexpRunT+refTime;
				m_fNextCNCPosOneX = m_fNextCNCStartX;
				m_fNextCNCPosOneY = m_fNextCNCStartY; 
				m_fNextCNCPosOneZ = m_fNextCNCStartZ; 
				// 2 Axis
// 				m_fNextCNCPosTwoX = m_fNextCNCPosOneX + (m_fCNCVMIN*refTime+m_fNextCNCPH*refTime*refTime/2.0)* cosf(m_fNextCNCStartAngle);
// 				m_fNextCNCPosTwoY =  m_fNextCNCPosOneY + (m_fCNCVMIN*refTime+m_fNextCNCPH*refTime*refTime/2.0)* sinf(m_fNextCNCStartAngle);

				m_fNextCNCStartAccX = m_fNextCNCPH*  m_fNextLengthX/m_fNextLengthMovement;// Increase
				m_fNextCNCStartAccY = m_fNextCNCPH*  m_fNextLengthY/m_fNextLengthMovement;
				m_fNextCNCStartAccZ = m_fNextCNCPH*  m_fNextLengthZ/m_fNextLengthMovement;

				m_fNextCNCPosTwoX = m_fNextCNCPosOneX + (m_fNextCNCStartVelX*refTime+m_fNextCNCStartAccX*refTime*refTime/2.0);
				m_fNextCNCPosTwoY =  m_fNextCNCPosOneY + (m_fNextCNCStartVelY*refTime+m_fNextCNCStartAccY*refTime*refTime/2.0);
				m_fNextCNCPosTwoZ =  m_fNextCNCPosOneZ + (m_fNextCNCStartVelZ*refTime+m_fNextCNCStartAccZ*refTime*refTime/2.0);
				

				break;
			case 2: case 3:
				// Cal culate for next Ref
				m_fNextCNCStartAngleVel = m_fCNCVMIN/m_fNextCNCRadius;
				m_fNextCNCStartAngleAcc = Rmsign(m_fNextCNCMidAngleVel - m_fNextCNCStartAngleVel)* m_fNextCNCPH/m_fNextCNCRadius;
//				m_fNextCNCMidAngleVel = m_fNextOmg;
				refTime = (m_fNextCNCMidAngleVel - m_fNextCNCStartAngleVel)/m_fNextCNCStartAngleAcc;
				m_fNextAccFirstTime = m_fexpRunT+refTime;
				m_fNextCNCAngleOne = m_fNextCNCStartAngle;
				m_fNextCNCAngleTwo = m_fNextCNCAngleOne+m_fNextCNCStartAngleVel*refTime+m_fNextCNCStartAngleAcc*refTime*refTime/2.0;
	//			m_fNextCNCEndAngle = m_fNextCNCEndAngle;
				break;
			default:
				;
			}// end switch about Calculate m_fNextCNCStartVelX

		}//end if
		else // If not is corner we can avoid corner solution
		{
			switch (m_iMoveType)
			{
			case 0:
				break;
			case 1:
				if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCEndX-m_fCNCPosTwoX)*(m_fCNCEndX-m_fCNCPosTwoX)
						+(m_fCNCEndY-m_fCNCPosTwoY)*(m_fCNCEndY-m_fCNCPosTwoY)
						+(m_fCNCEndZ-m_fCNCPosTwoZ)*(m_fCNCEndZ-m_fCNCPosTwoZ))) /m_fCNCFeedRate;
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}
				m_fexpRunT = m_fAccEndTime;
				// Cal culate for next Ref if is line
				m_fNextCNCPosOneX = m_fNextCNCStartX;
				m_fNextCNCPosOneY = m_fNextCNCStartY;
				m_fNextCNCPosOneZ = m_fNextCNCStartZ;   // No acceleration at next curve
				m_fNextCNCPosTwoX = m_fNextCNCPosOneX;
				m_fNextCNCPosTwoY = m_fNextCNCPosOneY;
				m_fNextCNCPosTwoZ = m_fNextCNCPosOneZ;
//				m_fNextAccFirstTime = m_fexpRunT;

				// Cal culate for next Ref if is circle
				m_fNextCNCAngleOne = m_fNextCNCStartAngle;
				m_fNextCNCAngleTwo = m_fNextCNCAngleOne; // No acceleration at next curve
				m_fNextAccFirstTime = m_fexpRunT;
				//						m_fAccEndTime = m_fAccEndTime+30*m_fSampTimeRef; // m_fAccEndTime need greater than m_fexpRunT to avoid mistake at line 473
				break;
			case 2: case 3:
				if (fabs(m_fCNCMidAngleVel) > DOUBLE_TOLERANCE)
				{
					m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCEndAngle-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
				}else
				{
					m_fAccEndTime = m_fAccFirstTime;
				}
				m_fexpRunT = m_fAccEndTime;
				// Cal culate for next Ref if is line
				m_fNextCNCPosOneX = m_fNextCNCStartX;m_fNextCNCPosOneY = m_fNextCNCStartY;   // No acceleration at next curve
				m_fNextCNCPosTwoX = m_fNextCNCPosOneX;m_fNextCNCPosTwoY = m_fNextCNCPosOneY;
//				m_fNextAccFirstTime = m_fexpRunT;

				// Cal culate for next Ref
				m_fNextCNCAngleOne = m_fNextCNCStartAngle;
				m_fNextCNCAngleTwo = m_fNextCNCAngleOne; // No acceleration at next curve
				m_fNextAccFirstTime = m_fexpRunT;
				//					 m_fAccEndTime = m_fAccEndTime+30*m_fSampTimeRef;  // m_fAccEndTime need greater than m_fexpRunT to avoid mistake at line 473
				break;
			case 4:
				m_fAccEndTime = m_fAccFirstTime;
				m_fexpRunT = m_fAccEndTime;
				m_fNextAccFirstTime = m_fexpRunT;
				m_fNextCNCEndX = 0.0;
				m_fNextCNCEndY = 0.0;
				m_fNextCNCEndZ = 0.0;
				break;
			default:
				;
			}// end switch
		} //end if else


	}//end  if (FoundNextPath)

}
bool FiveAxisCNC::OutOfLimitedAcceleration() 

{ 
	bool isOutOfLimited;
	double AccRef;
	isOutOfLimited= false;
	for (int i=0;i<NUM_COUNTER;i++)
	{
		AccRef = (vec_VelChange(i))/m_fSampTimeRef;
		if (AccRef> vec_CNCPMAX(i))
		{
			isOutOfLimited= true;
		}
	}
	return isOutOfLimited;
}
double FiveAxisCNC::Rmsign(double refNumber) 

{
	if (refNumber>0.0)
	{
		return 1.0;
	} 
	else
	{
		return -1.0;
	}
	

}
void FiveAxisCNC::GetAccelerationTime() 

{


}
double FiveAxisCNC::CalculateAccEndTime() 

{
	double refTime;
	switch (m_iMoveType)
	{
	case 0:
		break;
	case 1:
		m_fAccFirstTime = m_fNextAccFirstTime;
		m_fCNCPosOneX = m_fNextCNCStartX;m_fCNCPosTwoX = m_fNextCNCPosTwoX;
		m_fCNCPosThreeX = m_fNextCNCPosThreeX;
		m_fCNCPosOneY = m_fNextCNCStartY;m_fCNCPosTwoY = m_fNextCNCPosTwoY;
		m_fCNCPosThreeY = m_fNextCNCPosThreeY;

		m_fCNCStartVelX = m_fNextCNCStartVelX;m_fCNCMidVelX = m_fNextCNCMidVelX;
		m_fCNCStartAccX = m_fNextCNCStartAccX;m_fCNCEndAccX = m_fNextCNCEndAccX;
		m_fCNCStartVelY = m_fNextCNCStartVelY;m_fCNCMidVelY = m_fNextCNCMidVelY;
		m_fCNCStartAccY = m_fNextCNCStartAccY;m_fCNCEndAccY = m_fNextCNCEndAccY;
		m_fNextCNCStartVelX = vec_Nextrefr_1(0);
		m_fNextCNCStartVelY = vec_Nextrefr_1(1);
		m_fNextCNCMidVelX = vec_Nextrefr_1(0);
		m_fNextCNCMidVelY = vec_Nextrefr_1(1);
		m_fNextCNCEndVelX = vec_Nextrefr_1(0);
		m_fNextCNCEndVelY = vec_Nextrefr_1(1);

		refTime =  fabs((m_fCNCEndVelX - m_fCNCMidVelX)/m_fCNCPE);
//		m_fAccEndTime = m_fAccFirstTime+ (m_fCNCEndVelX - m_fCNCMidVelX)/m_fCNCPE;
//		m_fCNCPosThreeX = m_fCNCEndX - (m_fCNCMidVelX*refTime+sign((m_fCNCEndVelX - m_fCNCMidVelX))*m_fCNCPE*refTime*refTime/2.0);
//		m_fCNCPosThreeY = m_fCNCEndY - (m_fCNCMidVelY*refTime+sign((m_fCNCEndVelY - m_fCNCMidVelY))*m_fCNCPE*refTime*refTime/2.0);

//		m_fCNCEndX-m_fCNCPosThreeX

		break;
	case 2:
		break;
	case 3:
		break;
	default:
		;
	}

	return 0.4;
}
double FiveAxisCNC::CalculateNextAccFirstTime() 

{

	return 0.4;
}


// void FiveAxisCNC::GetNextGCodeLine() 
// {
// 	bool FoundNextPath= FALSE;
// 	m_bPathFinish = FALSE;
// 	System::String^ strRead;
// 	double dAB = 0.0,dOH= 0.0,sABC = 0.0;
// 
// 	m_fCNCStartX = m_fCNCEndX; 
// 	m_fCNCStartY = m_fCNCEndY;
// 	m_fCNCRadius = 999999.1;
// 	m_fCNCI = 0.0;
// 
// //	while ((!FoundNextPath)&&((strRead = GcodeFileStream->ReadLine()) != nullptr))
// 	while ((!FoundNextPath)&&((strRead = GcodeFileStream->ReadLine()) != nullptr))
// 	{	
// // 		inFGcode.getline(strRead, sizeof(strRead));//Read Number Line
// // 		strGcodeRead = CString(strRead);
// 		// 		sscanf.
// 		// 		strGcodeReadsscanf(strRead,"%s",strGcodeRead);
// 		FoundNextPath = AnalyseGcodeLine(strRead);
// 
// 	}
// 	if ((strRead == nullptr)||(!FoundNextPath))
// 	{
// 
// 		m_bGcodeFINISH = TRUE;
// 	}
// 	if (FoundNextPath)
// 	{
// 		//		m_fexpTnow = 0.0;
// 		//		m_fexpRunT = 0.0;
// 		//		m_fexpRunT= m_fexpRunTPre;
// 
// 		switch (m_iMoveType) 
// 		{
// 
// 		case 0:
// 
// 			//	m_iMoveType =(int)GetGcodeVariableValue(strRead, index);
// 
// 			break;
// 
// 		case 1:
// 			m_fLengthX = m_fCNCEndX- m_fCNCStartX;
// 			m_fLengthY =  m_fCNCEndY- m_fCNCStartY;
// 			m_fCNCStartAngle = atan2f(m_fLengthY,m_fLengthX);
// 			vec_refr_1(0)=   m_fCNCFeedRate* cosf(m_fCNCStartAngle);
// 			vec_refr_1(1) =  m_fCNCFeedRate* sinf(m_fCNCStartAngle);
// 			// 			vec_realx_1 = vec_refr_1;
// 			// 			vec_el_1(0)= 0.0;vec_el_1(1)= 0.0;
// 
// 			vec_refr_2(0) =  0.0;
// 			vec_refr_2(1) =  0.0;
// 			m_fexpRunT = m_fexpRunT+ sqrt(m_fLengthX*m_fLengthX+m_fLengthY*m_fLengthY)/m_fCNCFeedRate;
// 
// 			// Calculate Rotation Matrix
// 			mt_rotR(0,0) = cos(m_fCNCStartAngle);
// 			mt_rotR(1,0) = sin(m_fCNCStartAngle);
// 			mt_rotR(0,1) = -mt_rotR(1,0);// -sin(theta);
// 			mt_rotR(1,1) = mt_rotR(0,0);//cos(theta);
// 
// 
// 			mt_rotR_1(0,0) = 0.0;//-OMG * sin(theta);
// 			mt_rotR_1(0,1) = 0.0;//-OMG * cos(theta);
// 			mt_rotR_1(1,0) = 0.0;//OMG * cos(theta);
// 			mt_rotR_1(1,1) = 0.0;//-OMG * sin(theta);
// 			break;
// 		case 2:
// 			//m_fCNCStartAngle
// 			//	m_fOmg
// 			//NowGCode
// 			// Fin the cut point between  2 two circle with center(x1,y1) Radius R va Center (x3,y3) Radius R
// 			//(x-x1)^2 + (y-y1)^2 = R^2;
// 			//(x-x3)^2 + (y-y3)^2 = R^2;
// 			//             x1 = P1.X; x3 = P3.X; y1 = P1.Y; y3 = P3.Y;
// 			//             radius = inRadius;
// 			//             d13 = Math.Sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
// 			//             sABC = (d13 / 4) * Math.Sqrt((4 * radius * radius) - d13 * d13);// Not is S ABO
// 			//             hArc = radius - ((2 * sABC )/ d13);
// 			//             if (NowGCode == 3)
// 			//             {
// 			//                 P2.X = (x1 + x3) / 2 + hArc * (y1 - y3) / d13;
// 			//                 P2.Y = (y1 + y3) / 2 + hArc * (x1 - x3) / d13;
// 			//             }else
// 			// 			{
// 			// 				P2.X = (x1 + x3) / 2 + hArc * (y3 - y1) / d13;
// 			// 				P2.Y = (y1 + y3) / 2 + hArc * (x3 - x1) / d13;
// 			//                 }
// 			if (m_fCNCRadius< 99999.0)
// 			{
// 				dAB = sqrt((m_fCNCEndX - m_fCNCStartX) *(m_fCNCEndX - m_fCNCStartX)+ (m_fCNCEndY - m_fCNCStartY) *(m_fCNCEndY - m_fCNCStartY)) ;
// 				dOH = sqrt(m_fCNCRadius*m_fCNCRadius - dAB*dAB/4.0);
// 				if (m_fCNCRadius>DOUBLE_TOLERANCE)
// 
// 				{
// 					m_fCNCI = ((m_fCNCEndX + m_fCNCStartX)/2.0) + (dOH * (m_fCNCEndY - m_fCNCStartY) / dAB);
// 					m_fCNCJ = ((m_fCNCEndY + m_fCNCStartY) / 2.0) - (dOH * (m_fCNCEndX - m_fCNCStartX) / dAB);
// 
// 				}
// 				else
// 				{
// 					m_fCNCRadius = -m_fCNCRadius;
// 					m_fCNCI = ((m_fCNCEndX + m_fCNCStartX)/2.0) - (dOH * (m_fCNCEndY - m_fCNCStartY) / dAB);
// 					m_fCNCJ = ((m_fCNCEndY + m_fCNCStartY) / 2.0) + (dOH * (m_fCNCEndX - m_fCNCStartX) / dAB);
// 				}
// 				//				m_fCNCRadius = sqrt((m_fCNCEndX-m_fCNCI)*(m_fCNCEndX-m_fCNCI) +(m_fCNCEndY-m_fCNCJ)*(m_fCNCEndY-m_fCNCJ));
// 				m_fCNCStartAngle = atan2f(m_fCNCStartY-m_fCNCJ, m_fCNCStartX-m_fCNCI);
// 				m_fCNCEndAngle = atan2f(m_fCNCEndY-m_fCNCJ,m_fCNCEndX-m_fCNCI);	
// 
// 				if (m_fCNCStartAngle <m_fCNCEndAngle)
// 				{
// 					m_fCNCStartAngle = m_fCNCStartAngle+ 2.0*PI;
// 				}
// 				m_fexpRunT =  m_fexpRunT+(fabs(m_fCNCEndAngle- m_fCNCStartAngle))*m_fCNCRadius/ m_fCNCFeedRate;
// 				m_fOmg = m_fCNCFeedRate/m_fCNCRadius;
// 			}
// 			else
// 			{
// 				m_fCNCRadius = sqrt((m_fCNCEndX-m_fCNCI)*(m_fCNCEndX-m_fCNCI) +(m_fCNCEndY-m_fCNCJ)*(m_fCNCEndY-m_fCNCJ));
// 				m_fCNCStartAngle = atan2f(m_fCNCStartY-m_fCNCJ, m_fCNCStartX-m_fCNCI);
// 				m_fCNCEndAngle = atan2f(m_fCNCEndY-m_fCNCJ,m_fCNCEndX-m_fCNCI);	
// 
// 				if (m_fCNCStartAngle <m_fCNCEndAngle)
// 				{
// 					m_fCNCStartAngle = m_fCNCStartAngle+ 2.0*PI;
// 				}
// 				m_fexpRunT =  m_fexpRunT+(fabs(m_fCNCEndAngle- m_fCNCStartAngle))*m_fCNCRadius/ m_fCNCFeedRate;
// 				m_fOmg = m_fCNCFeedRate/m_fCNCRadius;
// 			}
// 			m_fOmg = -m_fOmg;// Clock Wise
// 			break;
// 		case 3:
// 
// 			if (m_fCNCRadius< 99999.0)
// 			{
// 				dAB = sqrt((m_fCNCEndX - m_fCNCStartX) *(m_fCNCEndX - m_fCNCStartX)+ (m_fCNCEndY - m_fCNCStartY) *(m_fCNCEndY - m_fCNCStartY)) ;
// 				dOH = sqrt(m_fCNCRadius*m_fCNCRadius - dAB*dAB/4.0);
// 				if (m_fCNCRadius>DOUBLE_TOLERANCE)
// 
// 				{
// 					m_fCNCI = ((m_fCNCEndX + m_fCNCStartX)/2.0) - (dOH * (m_fCNCEndY - m_fCNCStartY) / dAB);
// 					m_fCNCJ = ((m_fCNCEndY + m_fCNCStartY) / 2.0) + (dOH * (m_fCNCEndX - m_fCNCStartX) / dAB);
// 
// 				}
// 				else
// 				{
// 					m_fCNCRadius = -m_fCNCRadius;
// 					m_fCNCI = ((m_fCNCEndX + m_fCNCStartX)/2.0) + (dOH * (m_fCNCEndY - m_fCNCStartY) / dAB);
// 					m_fCNCJ = ((m_fCNCEndY + m_fCNCStartY) / 2.0) - (dOH * (m_fCNCEndX - m_fCNCStartX) / dAB);
// 				}
// 				//				m_fCNCRadius = sqrt((m_fCNCEndX-m_fCNCI)*(m_fCNCEndX-m_fCNCI) +(m_fCNCEndY-m_fCNCJ)*(m_fCNCEndY-m_fCNCJ));
// 				m_fCNCStartAngle = atan2f(m_fCNCStartY-m_fCNCJ, m_fCNCStartX-m_fCNCI);
// 				m_fCNCEndAngle = atan2f(m_fCNCEndY-m_fCNCJ,m_fCNCEndX-m_fCNCI);	
// 
// 				if (m_fCNCStartAngle >m_fCNCEndAngle)
// 				{
// 					m_fCNCEndAngle = m_fCNCEndAngle+ 2.0*PI;
// 				}
// 				m_fexpRunT =  m_fexpRunT+(fabs(m_fCNCEndAngle- m_fCNCStartAngle))*m_fCNCRadius/ m_fCNCFeedRate;
// 				m_fOmg = m_fCNCFeedRate/m_fCNCRadius;
// 			}
// 			else
// 			{
// 				m_fCNCRadius = sqrt((m_fCNCEndX-m_fCNCI)*(m_fCNCEndX-m_fCNCI) +(m_fCNCEndY-m_fCNCJ)*(m_fCNCEndY-m_fCNCJ));
// 				m_fCNCStartAngle = atan2f(m_fCNCStartY-m_fCNCJ, m_fCNCStartX-m_fCNCI);
// 				m_fCNCEndAngle = atan2f(m_fCNCEndY-m_fCNCJ,m_fCNCEndX-m_fCNCI);	
// 
// 				if (m_fCNCStartAngle >m_fCNCEndAngle)
// 				{
// 					m_fCNCEndAngle = m_fCNCEndAngle+ 2.0*PI;
// 				}
// 				m_fexpRunT =  m_fexpRunT+(fabs(m_fCNCEndAngle- m_fCNCStartAngle))*m_fCNCRadius/ m_fCNCFeedRate;
// 				m_fOmg = m_fCNCFeedRate/m_fCNCRadius;
// 			}
// 			break;
// 		default:
// 			;
// 		}
// 	}
// }
void FiveAxisCNC::SendOutputControl()
{
	IOModule.OutputAllMotor(vec_OutputControl);
}
vector<double> FiveAxisCNC::GetOutputControl()
{
	return vec_OutputControl;
}
void FiveAxisCNC::CloseBinaryFile()
{
		try 
	{
//		Console::WriteLine("trying to close file {0}...", fileName);
		DataAnalysisFileStream->Close();

	}
	catch (Exception^ e)
	{
//		if (dynamic_cast<FileNotFoundException^>(e))
//			Console::WriteLine("file '{0}' not found", fileName);
//		else
//			Console::WriteLine("problem reading file '{0}'", fileName);
	}

}
void FiveAxisCNC::CloseGcodeFile()
{
			try 
	{
//		Console::WriteLine("trying to close file {0}...", fileName);
		GcodeFileStream->Close();

	}
	catch (Exception^ e)
	{
//		if (dynamic_cast<FileNotFoundException^>(e))
//			Console::WriteLine("file '{0}' not found", fileName);
//		else
//			Console::WriteLine("problem reading file '{0}'", fileName);
	}

}
void FiveAxisCNC::StopFiveAxisCNC()
{

}

void FiveAxisCNC::InitVariableName(System::String^ strName,double fValue)
{

	// Weight 
	if (strName == "MX") {mt_M(0,0) = fValue;mt_1_M(0,0) = 1.0/fValue;}
	if (strName == "MY1") {mt_M(1,1) = fValue;mt_1_M(1,1) = 1.0/fValue;}
	if (strName == "MY2") {mt_M(2,2) = fValue;mt_1_M(2,2) = 1.0/fValue;}
	if (strName == "MZ") {mt_M(3,3) = fValue;mt_1_M(3,3) = 1.0/fValue;}
	if (strName == "MC") {mt_M(4,4) = fValue;mt_1_M(4,4) = 1.0/fValue;}
	if (strName == "MA1"){ mt_M(5,5) = fValue;mt_1_M(5,5) = 1.0/fValue;}
	if (strName == "MA2"){ mt_M(6,6) = fValue;mt_1_M(6,6) = 1.0/fValue;}

	// Frequency  Coefficient update
	if (strName == "WX") {mt_Kvw(0,0) = fValue;mt_Kpw(0,0) = fValue*fValue;}
	if (strName == "WY1"){mt_Kvw(1,1) = fValue;mt_Kpw(1,1) = fValue*fValue;}
	if (strName == "WY2"){mt_Kvw(2,2) = fValue;mt_Kpw(2,2) = fValue*fValue;}
	if (strName == "WZ") {mt_Kvw(3,3) = fValue;mt_Kpw(3,3) = fValue*fValue;}
	if (strName == "WC") {mt_Kvw(4,4) = fValue;mt_Kpw(4,4) = fValue*fValue;}
	if (strName == "WA1") {mt_Kvw(5,5) = fValue;mt_Kpw(5,5) = fValue*fValue;}
	if (strName == "WA2"){mt_Kvw(6,6) = fValue;mt_Kpw(6,6) = fValue*fValue;}
	// PID Coefficient update
	if (strName == "Kpwx") {mt_Kpw(0,0) = mt_Kpw(0,0)*fValue;}
	if (strName == "Kvwx") {mt_Kvw(0,0) = mt_Kvw(0,0)*fValue;}
	if (strName == "Kiwx") {}
	if (strName == "XDOBKd") {mt_DOBKd(0,0) = fValue;}
	if (strName == "XDOBKv") {mt_DOBKv(0,0) = fValue;}

	if (strName == "Kpwy1") {mt_Kpw(1,1) = mt_Kpw(1,1)*fValue;}
	if (strName == "Kvwy1") {mt_Kvw(1,1) = mt_Kvw(1,1)*fValue;}
	if (strName == "Kiwy1") {}
	if (strName == "Y1DOBKd") {mt_DOBKd(1,1) = fValue;}
	if (strName == "Y2DOBKv") {mt_DOBKv(1,1) = fValue;}

	if (strName == "Kpwy2") {mt_Kpw(2,2) = mt_Kpw(2,2)*fValue;}
	if (strName == "Kvwy2") {mt_Kvw(2,2) = mt_Kvw(2,2)*fValue;}
	if (strName == "Kiwy2") {}
	if (strName == "Y2DOBKd") {mt_DOBKd(2,2) = fValue;}
	if (strName == "Y2DOBKv") {mt_DOBKv(2,2) = fValue;}

	if (strName == "Kpwz") {mt_Kpw(3,3) = mt_Kpw(3,3)*fValue;}
	if (strName == "Kvwz") {mt_Kvw(3,3) = mt_Kvw(3,3)*fValue;}
	if (strName == "Kiwz") {}
	if (strName == "ZDOBKd") {mt_DOBKd(3,3) = fValue;}
	if (strName == "ZDOBKv") {mt_DOBKv(3,3) = fValue;}

	if (strName == "Kpwc") {mt_Kpw(4,4) = mt_Kpw(4,4)*fValue;}
	if (strName == "Kvwc") {mt_Kvw(4,4) = mt_Kvw(4,4)*fValue;}
	if (strName == "Kiwc") {}
	if (strName == "CDOBKd") {mt_DOBKd(4,4) = fValue;}
	if (strName == "CDOBKv") {mt_DOBKv(4,4) = fValue;}

	if (strName == "Kpwa1") {mt_Kpw(5,5) = mt_Kpw(5,5)*fValue;}
	if (strName == "Kvwa1") {mt_Kvw(5,5) = mt_Kvw(5,5)*fValue;}
	if (strName == "Kiwa1") {}
	if (strName == "A1DOBKd") {mt_DOBKd(5,5) = fValue;}
	if (strName == "A1DOBKv") {mt_DOBKv(5,5) = fValue;}

	if (strName == "Kpwa2") {mt_Kpw(6,6) = mt_Kpw(6,6)*fValue;}
	if (strName == "Kvwa2") {mt_Kvw(6,6) = mt_Kvw(6,6)*fValue;}
	if (strName == "Kiwa2") {}
	if (strName == "A2DOBKd") {mt_DOBKd(6,6) = fValue;}
	if (strName == "A2DOBKv") {mt_DOBKv(6,6) = fValue;}
	// Change PID matrix
// 	mt_Kpw(0,0) = m_fOmega0*m_fOmega0; mt_Kpw(1,1) = m_fOmega1*m_fOmega1;
// 	mt_Kvw(0,0) = 2*m_fOmegaX; mt_Kvw(1,1) = 2*m_fOmegaY1;
// 	mt_Kvw(0,0) = 2*m_fOmegaX; mt_Kvw(1,1) = 2*m_fOmegaY1;


// 	// Coefficient update
// 	// X Axis 
// 	if (strName == "XFWao") vec_FcFW(0) = fValue;  if (strName == "XBWao") vec_FcBW(0) = fValue;
// 	if (strName == "XFWa1") vec_FbrFW(0) = fValue; if (strName == "XBWa1") vec_FbrBW(0) = fValue;
// 	if (strName == "XFWvs") vec_FvsFW(0) = fValue; if (strName == "XBWvs") vec_FvsBW(0) = fValue;
// 	if (strName == "XFWa2") vec_KcFW(0) = fValue;  if (strName == "XBWa2") vec_KcBW(0) = fValue;
// 	// Y1 Axis 
// 	if (strName == "Y1FWao") vec_FcFW(1) = fValue;  if (strName == "Y1BWao") vec_FcBW(1) = fValue;
// 	if (strName == "Y1FWa1") vec_FbrFW(1) = fValue; if (strName == "Y1BWa1") vec_FbrBW(1) = fValue;
// 	if (strName == "Y1FWvs") vec_FvsFW(1) = fValue; if (strName == "Y1BWvs") vec_FvsBW(1) = fValue;
// 	if (strName == "Y1FWa2") vec_KcFW(1) = fValue;  if (strName == "Y1BWa2") vec_KcBW(1) = fValue;
// 	// Y2 Axis 
// 	if (strName == "Y2FWao") vec_FcFW(2) = fValue;  if (strName == "Y2BWao") vec_FcBW(2) = fValue;
// 	if (strName == "Y2FWa1") vec_FbrFW(2) = fValue; if (strName == "Y2BWa1") vec_FbrBW(2) = fValue;
// 	if (strName == "Y2FWvs") vec_FvsFW(2) = fValue; if (strName == "Y2BWvs") vec_FvsBW(2) = fValue;
// 	if (strName == "Y2FWa2") vec_KcFW(2) = fValue;  if (strName == "Y2BWa2") vec_KcBW(2) = fValue;
// 	// Z Axis 
// 	if (strName == "ZFWao") vec_FcFW(3) = fValue;  if (strName == "ZBWao") vec_FcBW(3) = fValue;
// 	if (strName == "ZFWa1") vec_FbrFW(3) = fValue; if (strName == "ZBWa1") vec_FbrBW(3) = fValue;
// 	if (strName == "ZFWvs") vec_FvsFW(3) = fValue; if (strName == "ZBWvs") vec_FvsBW(3) = fValue;
// 	if (strName == "ZFWa2") vec_KcFW(3) = fValue;  if (strName == "ZBWa2") vec_KcBW(3) = fValue;
// 	// C Axis 
// 	if (strName == "CFWao") vec_FcFW(4) = fValue;  if (strName == "CBWao") vec_FcBW(4) = fValue;
// 	if (strName == "CFWa1") vec_FbrFW(4) = fValue; if (strName == "CBWa1") vec_FbrBW(4) = fValue;
// 	if (strName == "CFWvs") vec_FvsFW(4) = fValue; if (strName == "CBWvs") vec_FvsBW(4) = fValue;
// 	if (strName == "CFWa2") vec_KcFW(4) = fValue;  if (strName == "CBWa2") vec_KcBW(4) = fValue;
// 	// A1 Axis 
// 	if (strName == "A1FWao") vec_FcFW(5) = fValue;  if (strName == "A1BWao") vec_FcBW(5) = fValue;
// 	if (strName == "A1FWa1") vec_FbrFW(5) = fValue; if (strName == "A1BWa1") vec_FbrBW(5) = fValue;
// 	if (strName == "A1FWvs") vec_FvsFW(5) = fValue; if (strName == "A1BWvs") vec_FvsBW(5) = fValue;
// 	if (strName == "A1FWa2") vec_KcFW(5) = fValue;  if (strName == "A1BWa2") vec_KcBW(5) = fValue;
// 	// A2 Axis 
// 	if (strName == "A2FWao") vec_FcFW(6) = fValue;  if (strName == "A2BWao") vec_FcBW(6) = fValue;
// 	if (strName == "A2FWa1") vec_FbrFW(6) = fValue; if (strName == "A2BWa1") vec_FbrBW(6) = fValue;
// 	if (strName == "A2FWvs") vec_FvsFW(6) = fValue; if (strName == "A2BWvs") vec_FvsBW(6) = fValue;
// 	if (strName == "A2FWa2") vec_KcFW(6) = fValue;  if (strName == "A2BWa2") vec_KcBW(6) = fValue;

	// Static friction model parameters
	// X Axis 
	if (strName == "XFWao")  mt_StaticFricData(0,0) = fValue;  if (strName == "XBWao") mt_StaticFricData(1,0) = fValue;
	if (strName == "XFWa1")  mt_StaticFricData(0,1) = fValue; if (strName == "XBWa1") mt_StaticFricData(1,1) = fValue;
	if (strName == "XFWvs")  mt_StaticFricData(0,2) = fValue; if (strName == "XBWvs") mt_StaticFricData(1,2) = fValue;
	if (strName == "XFWa2")  mt_StaticFricData(0,3) = fValue;  if (strName == "XBWa2") mt_StaticFricData(1,3) = fValue;
	// Y1 Axis 
	if (strName == "Y1FWao")  mt_StaticFricData(2,0) = fValue;  if (strName == "Y1BWao")  mt_StaticFricData(3,0) = fValue;
	if (strName == "Y1FWa1")  mt_StaticFricData(2,1) = fValue; if (strName == "Y1BWa1")  mt_StaticFricData(3,1) = fValue;
	if (strName == "Y1FWvs")  mt_StaticFricData(2,2) = fValue; if (strName == "Y1BWvs")  mt_StaticFricData(3,2) = fValue;
	if (strName == "Y1FWa2")  mt_StaticFricData(2,3) = fValue;  if (strName == "Y1BWa2")  mt_StaticFricData(3,3) = fValue;
	// Y2 Axis 
	if (strName == "Y2FWao")  mt_StaticFricData(4,0) = fValue;  if (strName == "Y2BWao")  mt_StaticFricData(5,0) = fValue;
	if (strName == "Y2FWa1")  mt_StaticFricData(4,1) = fValue; if (strName == "Y2BWa1")  mt_StaticFricData(5,1) = fValue;
	if (strName == "Y2FWvs")  mt_StaticFricData(4,2) = fValue; if (strName == "Y2BWvs")  mt_StaticFricData(5,2) = fValue;
	if (strName == "Y2FWa2")  mt_StaticFricData(4,3) = fValue;  if (strName == "Y2BWa2")  mt_StaticFricData(5,3) = fValue;
	// Z Axis 
	if (strName == "ZFWao")  mt_StaticFricData(6,0) = fValue;  if (strName == "ZBWao")  mt_StaticFricData(7,0) = fValue;
	if (strName == "ZFWa1")  mt_StaticFricData(6,1) = fValue; if (strName == "ZBWa1")  mt_StaticFricData(7,1) = fValue;
	if (strName == "ZFWvs")  mt_StaticFricData(6,2) = fValue; if (strName == "ZBWvs")  mt_StaticFricData(7,2) = fValue;
	if (strName == "ZFWa2")  mt_StaticFricData(6,3) = fValue;  if (strName == "ZBWa2")  mt_StaticFricData(7,3) = fValue;
	// C Axis 
	if (strName == "CFWao")  mt_StaticFricData(8,0) = fValue;  if (strName == "CBWao")  mt_StaticFricData(9,0) = fValue;
	if (strName == "CFWa1")  mt_StaticFricData(8,1) = fValue; if (strName == "CBWa1")  mt_StaticFricData(9,1) = fValue;
	if (strName == "CFWvs")  mt_StaticFricData(8,2) = fValue; if (strName == "CBWvs")  mt_StaticFricData(9,2) = fValue;
	if (strName == "CFWa2")  mt_StaticFricData(8,3) = fValue;  if (strName == "CBWa2")  mt_StaticFricData(9,3) = fValue;
	// A1 Axis 
	if (strName == "A1FWao")  mt_StaticFricData(10,0) = fValue;  if (strName == "A1BWao")  mt_StaticFricData(11,0) = fValue;
	if (strName == "A1FWa1")  mt_StaticFricData(10,1) = fValue; if (strName == "A1BWa1")  mt_StaticFricData(11,1) = fValue;
	if (strName == "A1FWvs")  mt_StaticFricData(10,2) = fValue; if (strName == "A1BWvs")  mt_StaticFricData(11,2) = fValue;
	if (strName == "A1FWa2")  mt_StaticFricData(10,3) = fValue;  if (strName == "A1BWa2")  mt_StaticFricData(11,3) = fValue;
	// A2 Axis 
	if (strName == "A2FWao")  mt_StaticFricData(12,0) = fValue;  if (strName == "A2BWao")  mt_StaticFricData(13,0) = fValue;
	if (strName == "A2FWa1")  mt_StaticFricData(12,1) = fValue; if (strName == "A2BWa1")  mt_StaticFricData(13,1) = fValue;
	if (strName == "A2FWvs")  mt_StaticFricData(12,2) = fValue; if (strName == "A2BWvs")  mt_StaticFricData(13,2) = fValue;
	if (strName == "A2FWa2")  mt_StaticFricData(12,3) = fValue;  if (strName == "A2BWa2")  mt_StaticFricData(13,3) = fValue;

}
double FiveAxisCNC::GetStaticVariable(System::String^ strName)
{
	double fValue;
	fValue = 0.0;
	// Weight 
	if (strName == "MX") fValue = mt_M(0,0) ;
	if (strName == "MY1") fValue = mt_M(1,1) ;
	if (strName == "MY2") fValue = mt_M(2,2) ;
	if (strName == "MZ") fValue = mt_M(3,3) ;
	if (strName == "MC") fValue = mt_M(4,4) ;
	if (strName == "MA1") fValue = mt_M(5,5) ;
	if (strName == "MA2") fValue = mt_M(6,6) ;
	// Coefficient update
	// X Axis 
	if (strName == "XFWao") fValue = vec_FcFW(0) ;  if (strName == "XBWao") fValue = vec_FcBW(0) ;
	if (strName == "XFWa1") fValue = vec_FbrFW(0) ; if (strName == "XBWa1") fValue = vec_FbrBW(0) ;
	if (strName == "XFWvs") fValue = vec_FvsFW(0) ; if (strName == "XBWvs") fValue = vec_FvsBW(0) ;
	if (strName == "XFWa2") fValue = vec_KcFW(0) ;  if (strName == "XBWa2") fValue = vec_KcBW(0) ;
	// Y1 Axis 
	if (strName == "Y1FWao") fValue = vec_FcFW(1) ;  if (strName == "Y1BWao") fValue = vec_FcBW(1) ;
	if (strName == "Y1FWa1") fValue = vec_FbrFW(1) ; if (strName == "Y1BWa1") fValue = vec_FbrBW(1) ;
	if (strName == "Y1FWvs") fValue = vec_FvsFW(1) ; if (strName == "Y1BWvs") fValue = vec_FvsBW(1) ;
	if (strName == "Y1FWa2") fValue = vec_KcFW(1) ;  if (strName == "Y1BWa2") fValue = vec_KcBW(1) ;
	// Y2 Axis 
	if (strName == "Y2FWao") fValue = vec_FcFW(2) ;  if (strName == "Y2BWao") fValue = vec_FcBW(2) ;
	if (strName == "Y2FWa1") fValue = vec_FbrFW(2) ; if (strName == "Y2BWa1") fValue = vec_FbrBW(2) ;
	if (strName == "Y2FWvs") fValue = vec_FvsFW(2) ; if (strName == "Y2BWvs") fValue = vec_FvsBW(2) ;
	if (strName == "Y2FWa2") fValue = vec_KcFW(2) ;  if (strName == "Y2BWa2") fValue = vec_KcBW(2) ;
	// Z Axis 
	if (strName == "ZFWao") fValue = vec_FcFW(3) ;  if (strName == "ZBWao") fValue = vec_FcBW(3) ;
	if (strName == "ZFWa1") fValue = vec_FbrFW(3) ; if (strName == "ZBWa1") fValue = vec_FbrBW(3) ;
	if (strName == "ZFWvs") fValue = vec_FvsFW(3) ; if (strName == "ZBWvs") fValue = vec_FvsBW(3) ;
	if (strName == "ZFWa2") fValue = vec_KcFW(3) ;  if (strName == "ZBWa2") fValue = vec_KcBW(3) ;
	// C Axis 
	if (strName == "CFWao") fValue = vec_FcFW(4) ;  if (strName == "CBWao") fValue = vec_FcBW(4) ;
	if (strName == "CFWa1") fValue = vec_FbrFW(4) ; if (strName == "CBWa1") fValue = vec_FbrBW(4) ;
	if (strName == "CFWvs") fValue = vec_FvsFW(4) ; if (strName == "CBWvs") fValue = vec_FvsBW(4) ;
	if (strName == "CFWa2") fValue = vec_KcFW(4) ;  if (strName == "CBWa2") fValue = vec_KcBW(4) ;
	// A1 Axis 
	if (strName == "A1FWao") fValue = vec_FcFW(5) ;  if (strName == "A1BWao") fValue = vec_FcBW(5) ;
	if (strName == "A1FWa1") fValue = vec_FbrFW(5) ; if (strName == "A1BWa1") fValue = vec_FbrBW(5) ;
	if (strName == "A1FWvs") fValue = vec_FvsFW(5) ; if (strName == "A1BWvs") fValue = vec_FvsBW(5) ;
	if (strName == "A1FWa2") fValue = vec_KcFW(5) ;  if (strName == "A1BWa2") fValue = vec_KcBW(5) ;
	// A2 Axis 
	if (strName == "A2FWao") fValue = vec_FcFW(6) ;  if (strName == "A2BWao") fValue = vec_FcBW(6) ;
	if (strName == "A2FWa1") fValue = vec_FbrFW(6) ; if (strName == "A2BWa1") fValue = vec_FbrBW(6) ;
	if (strName == "A2FWvs") fValue = vec_FvsFW(6) ; if (strName == "A2BWvs") fValue = vec_FvsBW(6) ;
	if (strName == "A2FWa2") fValue = vec_KcFW(6) ;  if (strName == "A2BWa2") fValue = vec_KcBW(6) ;

	return fValue;
}
double FiveAxisCNC::TestMatrix(int mt_index)
{
	 return mt_M(mt_index,mt_index);
}
void FiveAxisCNC::SetMatrix(int mt_index, double mt_value)
{
// 	matrix<double> mt_Fbr;
// 	mt_Fbr.resize(1,5);
// 	mt_Fbr(1,3) = 3.4;
	mt_M(1,mt_index)= mt_value;
/*	mt_M->data()*/
}
	