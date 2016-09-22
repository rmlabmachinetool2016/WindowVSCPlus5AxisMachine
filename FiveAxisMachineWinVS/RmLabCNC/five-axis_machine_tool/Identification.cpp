#include "stdafx.h"
#include "Identification.h"

void Identification::Identification(unsigned int step,short AioId,short CntId,unsigned int selected_axis,array<double>^ param,array<double,2>^ datalog)
{
	unsigned int	i,j;
	vector<double>	posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0);
		
	double			time,u;

	time = SAMPLING_TIME * step;

	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);

	//double gain[NUM_ACTUATOR] = {5,5,-5,7,-0.2,0.5,0.5};
	u = param[0]*sin(time*param[1]);
	forceall.clear();
	forceall(selected_axis) = u;
	if(selected_axis==1)
		forceall(2) = -u;
	else if(selected_axis==5)
		forceall(6) = -u;

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// loging data
	datalog[0,step] = time;
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+13,step] = posall(i);
		datalog[j+14,step] = velall(i);
		datalog[j+15,step] = accall(i);
		datalog[j+16,step] = forceall(i);
	}
}

void Identification::CalcModelParam(double& M,double& Fri,unsigned int selected_axis,array<double>^ param,array<double,2>^ datalog)
{
	unsigned int	i,length = datalog->GetLength(1);
	vector<double>	Ans(2),F(length),Ff(F);
	matrix<double>	Phi(length,2),tPhi,invPhi,Phif(Phi);
	unsigned int	force_index[] = {16,24,32,40,48,56,64},
					acc_index[] = {15,23,31,39,47,55,63},
					vel_index[] = {14,22,30,38,46,54,62};
	double			cut = param[2];
	LPF				lpf(cut),lpfphi(cut);

	for(i=0;i<length;i++){
		F(i) = datalog[force_index[selected_axis],i];	// force
		Phi(i,0) = datalog[acc_index[selected_axis],i];	// acc
		Phi(i,1) = datalog[vel_index[selected_axis],i];	// vel
	}
	Ff = lpf.LPFOFF( F );
	column(Phif,0) = lpfphi.LPFOFF( column(Phi,0) );
	lpfphi.clear();
	column(Phif,1) = lpfphi.LPFOFF( column(Phi,1) );

	Phi = Phif;
	F = Ff;
	tPhi = trans(Phi);
	math::invert(prod(tPhi,Phi),invPhi);
			 
	Ans = prod(prod(invPhi,tPhi),F);

	M = Ans(0);
	Fri = Ans(1);
}
