#pragma once
#include "StdAfx.h"
#include "FiveAxisCNC.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
using namespace boost::numeric::ublas; 
namespace RmLabCNC{

static matrix<double> mt_Simul1_M,mt_SimulPeakHeight,mt_SimulPeakPos,mt_SimulPeakWid;
static vector<double>  vec_Simulx,vec_SimulPrex,
					   vec_Simulx_1,vec_SimulPrex_1,
					   vec_SimulKc, vec_SimulFc,
					   vec_InertiaForce,vec_SimulNominalFc,vec_SimulNominalFv,vec_SimulNumberPeak
					   ;
ref class SimulationModel
{
public:
	double m_fSamplingTime;
	SimulationModel(void);
	void LoadModelParameter(System::String^ fileSimulationModel);
	void ResetSimulationModel();
	vector<double> GetSimulationModelPosition();
	void CalculateInertiaForce(vector<double> vec_SumForce);
	void SetControlInput(vector<double> vec_input);
	void rmsscanf(System::String^ scanString,System::String^ &strName,double &fValue);
	void InitVariableName(System::String^ strName,double fValue);
	double gaussian(double x, double pos, double wid);
	void InitSimulNonlinearFrictionData(System::String^ fileNLF);
};
}

