#include "StdAfx.h"
#include "FiveAxisCNC.h"
using namespace System::IO;
using namespace RmLabCNC;

void FiveAxisCNC::rmsscanf(System::String^ scanString,System::String^ &strName,double &fValue)
{
	System::String^ stringResult;
	int i= 0;
	double temp = 0.0;
	scanString = " "+ scanString;

	stringResult = "";
	while ((i < scanString->Length) &&(scanString[i] ==' ')) i++;
	while ((i< scanString->Length ) &&(scanString[i] !=' '))
	{
		stringResult = stringResult+ scanString[i];
		i++;
	}
	strName = stringResult;
//	stringResult = "Deo hieu"; // Can not understand why cannot reset stringResult to "";
	System::String^ stringValue;
	stringValue = "";
	while ((i<scanString->Length) &&(scanString[i] ==' ')) i++;
	while ((i<scanString->Length) &&(scanString[i] !=' '))
	{
		stringValue= stringValue+ scanString[i];
		i++;
	}
	try
	{
		fValue= System::Convert::ToDouble(stringValue);
	}
	catch (Exception^ e)
	{
		fValue = 0.0;
	}

	//	sscanf(strRead,"%f",&temp);
}// end rmsscanf
// 				m_fCNCPEX = m_fCNCPE* cosf(m_fNextCNCStartAngle);
// m_fNextCNCPHX = m_fNextCNCPH* cosf(m_fNextCNCStartAngle);
// m_fNextCNCPHY = m_fNextCNCPH* sinf(m_fNextCNCStartAngle);
// 
// m_fCNCEndVelX = m_fCNCMidVelX +(vec_VelChange(0)-sign( vec_VelChange(0) )*vec_CNCVMIN(0))/2.0 ;
// m_fNextCNCStartVelX = m_fCNCEndVelX+sign( vec_VelChange(0))*vec_CNCVMIN(0);
// m_fNextCNCStartVelY = m_fNextCNCStartVelX*tanf(m_fNextCNCStartAngle);
// 
// 
// switch (m_iMoveType)
// {
// case 1:
// 	m_fCNCEndVelX = 0.3;
// 	m_fCNCEndVelY = m_fCNCEndVelX* tanf(m_fCNCStartAngle);
// 
// 	refTime =  fabs((m_fCNCEndVelX - m_fCNCMidVelX)/m_fCNCPEX);
// 	m_fCNCPEY = (m_fCNCEndVelY - m_fCNCMidVelY)/refTime;
// 	//		m_fAccEndTime = m_fAccFirstTime+ (m_fCNCEndVelX - m_fCNCMidVelX)/m_fCNCPE;
// 	m_fCNCPosThreeX = m_fCNCEndX - (m_fCNCMidVelX*refTime+sign((m_fCNCEndVelX - m_fCNCMidVelX))*m_fCNCPEX*refTime*refTime/2.0);
// 	m_fCNCPosThreeY = m_fCNCEndY - (m_fCNCMidVelY*refTime+sign((m_fCNCEndVelY - m_fCNCMidVelY))*m_fCNCPEY*refTime*refTime/2.0);
// 
// 	m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCPosThreeX-m_fCNCPosTwoX)/m_fCNCMidVelX);
// 	m_fexpRunT = m_fAccEndTime+ refTime;
// 	// Cal culate for next Ref
// 	m_fNextAccFirstTime = m_fexpRunT+ fabs((m_fNextCNCMidVelX-m_fNextCNCStartVelX)/m_fNextCNCPHX);
// 	m_fNextCNCPosOneX = m_fCNCEndX;m_fNextCNCPosOneY = m_fCNCEndY; 
// 	break;
// case 2,3:
// 	m_fCNCEndVelY = m_fCNCEndVelX* tanf(m_fCNCEndAngle);
// 	m_fCNCPEY = (m_fCNCEndVelY - m_fCNCMidVelY)/refTime;
// 
// 	m_fCNCEndAngleVel = sign(m_fCNCMidAngleVel)*sqrt(m_fCNCEndVelX*m_fCNCEndVelX+m_fCNCEndVelY*m_fCNCEndVelY)/m_fCNCRadius;
// 	m_fCNCEndAngleAcc = m_fCNCPE/m_fCNCRadius;
// 
// 	refTime =  fabs((m_fCNCMidAngleVel - m_fCNCEndAngleVel)/m_fCNCEndAngleAcc);
// 	refAngle = m_fCNCEndAngle -( m_fCNCMidAngleVel*refTime+m_fCNCEndAngleAcc*refTime*refTime/2.0);
// 	m_fCNCPosThreeX =  m_fCNCI+ m_fCNCRadius*cos(refAngle);
// 	m_fCNCPosThreeY =  m_fCNCJ+ m_fCNCRadius*sin(refAngle);
// 
// 	m_fAccEndTime = m_fAccFirstTime+ ((refAngle-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
// 	m_fexpRunT = m_fAccEndTime+ refTime;
// 	// Cal culate for next Ref
// 	m_fNextAccFirstTime = m_fexpRunT+ fabs((m_fNextCNCMidVelX-m_fNextCNCStartVelX)/m_fNextCNCPHX);
// 	m_fNextCNCPosOneX = m_fCNCEndX;m_fNextCNCPosOneY = m_fCNCEndY; 
// 
// 	break;
// default:
// 	;
// }// End switch



//if (OutOfLimitedAcceleration())
// {
	// 
	// 	//				m_fCNCPEX = m_fCNCPE* cosf(m_fNextCNCStartAngle);
	// 	m_fNextCNCPHX = m_fNextCNCPH* cosf(m_fNextCNCStartAngle);
	// 	m_fNextCNCPHY = m_fNextCNCPH* sinf(m_fNextCNCStartAngle);
	// 
	// 	//				m_fCNCEndVelX = m_fCNCMidVelX +(vec_VelChange(0)-sign( vec_VelChange(0) )*vec_CNCVMIN(0))/2.0 ;
	// 	m_fNextCNCStartVelX = m_fCNCVMIN* cosf(m_fNextCNCStartAngle);
	// 	m_fNextCNCStartVelY = m_fCNCVMIN* sinf(m_fNextCNCStartAngle);
	// 
	// 	switch (m_iMoveType)
	// 	{
	// 	case 1:
	// 		// 		vec_refr_1(0) = (-m_fOmg)*(m_CNCRefPos.Y- m_fCNCJ);//-m_fexpR*m_fOmg*sin(theta);  mm/s
	// 		// 		vec_refr_1(1) = (m_fOmg)*(m_CNCRefPos.X-m_fCNCI);//m_fexpR*m_fOmg*Cos(theta);		mm/s
	// 
	// 		m_fCNCEndVelX = m_fCNCVMIN* cosf(m_fCNCStartAngle);
	// 		m_fCNCEndVelY = m_fCNCVMIN* sinf(m_fCNCStartAngle);
	// 
	// 		m_fCNCEndAccX = -m_fCNCPE* cosf(m_fCNCStartAngle);// Decrease to Zero
	// 		m_fCNCEndAccY = -m_fCNCPE* sinf(m_fCNCStartAngle);
	// 		refTime =  fabs((m_fCNCFeedRate - m_fCNCVMIN)/m_fCNCPE);
	// 		//					m_fCNCPEY = (m_fCNCEndVelY - m_fCNCMidVelY)/refTime;
	// 		//		m_fAccEndTime = m_fAccFirstTime+ (m_fCNCEndVelX - m_fCNCMidVelX)/m_fCNCPE;
	// 		m_fCNCPosThreeX = m_fCNCEndX - (m_fCNCFeedRate*refTime-m_fCNCPE*refTime*refTime/2.0)* cosf(m_fCNCStartAngle);
	// 		m_fCNCPosThreeY = m_fCNCEndY - (m_fCNCFeedRate*refTime-m_fCNCPE*refTime*refTime/2.0)* sinf(m_fCNCStartAngle);
	// 
	// 		if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
	// 		{
	// 			m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCPosThreeX-m_fCNCPosTwoX)*(m_fCNCPosThreeX-m_fCNCPosTwoX)
	// 				+(m_fCNCPosThreeY-m_fCNCPosTwoY)*(m_fCNCPosThreeY-m_fCNCPosTwoY))) /m_fCNCFeedRate;
	// 		}else
	// 		{
	// 			m_fAccEndTime = m_fAccFirstTime;
	// 		}
	// 
	// 		m_fexpRunT = m_fAccEndTime+ refTime;
	// 
	// 		// Cal culate for next Ref
	// 		refTime = fabs((m_fNextCNCFeedRate - m_fCNCVMIN)/m_fNextCNCPH);
	// 		m_fNextAccFirstTime = m_fexpRunT+refTime;
	// 		m_fNextCNCPosOneX = m_fNextCNCStartX;m_fNextCNCPosOneY = m_fNextCNCStartY; 
	// 		m_fNextCNCPosTwoX = m_fNextCNCPosOneX + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* cosf(m_fNextCNCStartAngle);
	// 		m_fNextCNCPosTwoY =  m_fNextCNCPosOneY + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* sinf(m_fNextCNCStartAngle);
	// 		m_fNextCNCStartAccX = m_fNextCNCPH* cosf(m_fNextCNCStartAngle);// Increase
	// 		m_fNextCNCStartAccY = m_fNextCNCPH* sinf(m_fNextCNCStartAngle);
	// 		break;
	// 	case 2: case 3:
	// 		m_fCNCEndVelX = m_fCNCVMIN* cosf(m_fCNCEndAngle);
	// 		m_fCNCEndVelY = m_fCNCVMIN* sinf(m_fCNCEndAngle);
	// 
	// 		// 					m_fCNCEndVelY = m_fCNCEndVelX* tanf(m_fCNCEndAngle);
	// 		// 					m_fCNCPEY = (m_fCNCEndVelY - m_fCNCMidVelY)/refTime;
	// 
	// 		m_fCNCEndAngleVel = m_fCNCVMIN/m_fCNCRadius;// Rad/second
	// 		m_fCNCEndAngleAcc = m_fOmg/fabs(m_fOmg)*m_fCNCPE/m_fCNCRadius;
	// 
	// 		refTime =  (m_fCNCMidAngleVel - m_fCNCEndAngleVel)/m_fCNCEndAngleAcc;// no need fabs
	// 		refAngle = m_fCNCEndAngle - (m_fCNCMidAngleVel*refTime+m_fCNCEndAngleAcc*refTime*refTime/2.0);
	// 		m_fCNCAngleThree = refAngle;
	// 
	// 		m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCAngleThree-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
	// 		m_fexpRunT = m_fAccEndTime+ refTime;
	// 		// Cal culate for next Ref
	// 		refTime = fabs((m_fNextCNCFeedRate - m_fCNCVMIN)/m_fCNCPH);
	// 		m_fNextAccFirstTime = m_fexpRunT+refTime;
	// 		m_fNextCNCPosOneX = m_fNextCNCStartX;m_fNextCNCPosOneY = m_fNextCNCStartY; 
	// 		m_fNextCNCPosTwoX = m_fNextCNCPosOneX + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* cosf(m_fNextCNCStartAngle);
	// 		m_fNextCNCPosTwoY =  m_fNextCNCPosOneY + (m_fCNCVMIN*refTime+m_fCNCPH*refTime*refTime/2.0)* sinf(m_fNextCNCStartAngle);
	// 		break;
	// 	default:
	// 		;
	// 	}// End switch
	// }//end if
	// else // If not is corner we can avoid corner solution
	// {
	// 	switch (m_iMoveType)
	// 	{
	// 	case 1:
	// 
	// 
	// 		if (m_fCNCFeedRate > DOUBLE_TOLERANCE)
	// 		{
	// 			m_fAccEndTime = m_fAccFirstTime+ (sqrtf((m_fCNCEndX-m_fCNCPosTwoX)*(m_fCNCEndX-m_fCNCPosTwoX)
	// 				+(m_fCNCEndY-m_fCNCPosTwoY)*(m_fCNCEndY-m_fCNCPosTwoY))) /m_fCNCFeedRate;
	// 		}else
	// 		{
	// 			m_fAccEndTime = m_fAccFirstTime;
	// 		}
	// 		m_fexpRunT = m_fAccEndTime;
	// 		// Cal culate for next Ref
	// 		m_fNextAccFirstTime = m_fexpRunT;
	// 		//						m_fAccEndTime = m_fAccEndTime+30*m_fSampTimeRef; // m_fAccEndTime need greater than m_fexpRunT to avoid mistake at line 473
	// 		break;
	// 	case 2: case 3:
	// 		if (m_fCNCMidAngleVel > DOUBLE_TOLERANCE)
	// 		{
	// 			m_fAccEndTime = m_fAccFirstTime+ ((m_fCNCEndAngle-m_fCNCAngleTwo)/m_fCNCMidAngleVel);
	// 		}else
	// 		{
	// 			m_fAccEndTime = m_fAccFirstTime;
	// 		}
	// 		m_fexpRunT = m_fAccEndTime;
	// 		// Cal culate for next Ref
	// 		m_fNextAccFirstTime = m_fexpRunT;
	// 		//					 m_fAccEndTime = m_fAccEndTime+30*m_fSampTimeRef;  // m_fAccEndTime need greater than m_fexpRunT to avoid mistake at line 473
	// 		break;
	// 	default:
	// 		;
	// 	}// end switch
	// } //end if else
	// 
	// 
	// 	}//end if OutOfLimitedAcceleration