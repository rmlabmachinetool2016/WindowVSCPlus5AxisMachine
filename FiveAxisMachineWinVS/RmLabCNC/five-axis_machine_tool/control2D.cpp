#include "stdafx.h"
#include "control2D.h"
#include "math.hpp"
#include "io.h"
#include "plant.h"
#include <cmath>

void control2d::ControlSim2D(	unsigned int control_type,
						unsigned int max_step,
						bool enable_noise,
						bool enable_resolution,
						bool enable_sync,
						array<double,2>^ datalog)
{
	switch(control_type){
	case INDEPENDENT:
		IndependentCon(max_step,enable_noise,enable_resolution,enable_sync,datalog);
		break;
	case CONTOURING:
		ContouringCon(max_step,enable_noise,enable_resolution,enable_sync,datalog);
		break;
	case MOD_CONTOURING:
		modContouringCon(max_step,enable_noise,enable_resolution,enable_sync,datalog);
		break;
	default:
		IndependentCon(max_step,enable_noise,enable_resolution,enable_sync,datalog);
		break;
	}
}

void control2d::SetConParam(array<double>^ conparam)
{
	using namespace control2d::conparam;

	mass_x		= conparam[0];
	mass_y1		= conparam[1];
	mass_y2		= conparam[2];
	c_x			= conparam[7];
	c_y1		= conparam[8];
	c_y2		= conparam[9];
	pole_x		= conparam[14];
	pole_y1		= conparam[15];
	pole_y2		= conparam[16];
	pole_t		= conparam[21];
	pole_n		= conparam[22];
	pole_sync_y	= conparam[27];

	mass.resize(2,2);
	c.resize(2,2);
	kpw.resize(2,2);
	kvw.resize(2,2);
	kpl.resize(2,2);
	kvl.resize(2,2);
	mass.clear();
	c.clear();
	kpw.clear();
	kvw.clear();
	kpl.clear();
	kvl.clear();

	mass(0,0) = mass_x;
	mass(1,1) = mass_y1;
	c(0,0) = c_x;
	c(1,1) = c_y1;
	math::invert(mass,invmass);

	kpw(0,0) = pole_x * pole_x;
	kpw(1,1) = pole_y1 * pole_y1;
	kvw(0,0) = 2 * pole_x;
	kvw(1,1) = 2 * pole_y1;

	kpl(0,0) = pole_t * pole_t;
	kpl(1,1) = pole_n * pole_n;
	kvl(0,0) = 2 * pole_t;
	kvl(1,1) = 2 * pole_n;

	masss = mass_y2;
	cs = c_y2;
	kvs = 2 * pole_sync_y;
	kps = pole_sync_y * pole_sync_y;
}

bool control2d::IsReachPoint(double time,double max_angle,array<double,2>^ datalog)
{
	volatile double sdot,rlength,rangle,
					x,y,xdot,ydot;
	volatile static double len,total_angle,oldsdot;
	unsigned int step = static_cast<unsigned int>(time/SAMPLING_TIME);

	rlength = max_angle/360.*2.*RADIUS*PI;
	rangle = max_angle * PI/180.;

	if(time == 0){
		oldsdot = 0;
		len = 0;
		total_angle = 0;
	}
	
	xdot = datalog[14,step];
	ydot = datalog[22,step];
	sdot = sqrt(xdot*xdot + ydot*ydot);
	if(time==0)
		len=0;
	else
		len += (sdot+oldsdot)*SAMPLING_TIME/2;
	oldsdot = sdot;
	total_angle = len / RADIUS;

	bool is_angle;
	is_angle = (rangle*0.99<total_angle)?true:false;
	x = datalog[13,step];
	y = datalog[21,step];

	volatile bool is_time;
	is_time = time>(rangle/2/PI)?true:false;

	if(is_angle && x>0 && y<0 && is_time)
		return true;
	else
		return false;
}

void control2d::GenerateReference(double time,vector<double>& rpos,vector<double>& rvel,vector<double>& racc)
{
	double theta = OMG * time;

	rpos(0) =  RADIUS * sin(theta);					// x-axis
	rpos(1) = -RADIUS * cos(theta);					// y1-axis

	rvel(0) = RADIUS * OMG * cos(theta);			// x-axis
	rvel(1) = RADIUS * OMG * sin(theta);			// y1-axis

	racc(0) = -RADIUS * OMG * OMG * sin(theta);		// x-axis
	racc(1) =  RADIUS * OMG * OMG * cos(theta);		// y1-axis
}

void control2d::CalcRotationMat(double time,matrix<double>& Rot,matrix<double>& Rot1,matrix<double>& Rot2)
{
	double theta = OMG * time;

	Rot(0,0) =  cos(theta);
	Rot(0,1) = -sin(theta);
	Rot(1,0) =  sin(theta);
	Rot(1,1) =  cos(theta);
	
	Rot1(0,0) = -OMG * sin(theta);
	Rot1(0,1) = -OMG * cos(theta);
	Rot1(1,0) =  OMG * cos(theta);
	Rot1(1,1) = -OMG * sin(theta);

	Rot2(0,0) = -OMG * OMG * cos(theta);
	Rot2(0,1) =  OMG * OMG * sin(theta);
	Rot2(1,0) = -OMG * OMG * sin(theta);
	Rot2(1,1) = -OMG * OMG * cos(theta);
}

double control2d::CalcContouringError(vector<double> &pos)
{
	double ec,x,y;

	x = pos(0);
	y = pos(1);
	ec = sqrt( x*x+y*y ) - RADIUS;

	return ec;
}

void control2d::IndependentCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	
	unsigned int step,i,j;
	PLANT			plant;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);
					
	double			es,es1,es2,
					poss,vels,accs,
					rposs,rvels,raccs,
					forces,time,ec;

	for(step = 0;step < max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		// get state value
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
		pos(0) = posall(0);
		pos(1) = posall(1);

		vel(0) = velall(0);
		vel(1) = velall(1);

		acc(0) = accall(0);
		acc(1) = accall(1);

		// get reference
		GenerateReference(time,rpos,rvel,racc);

		// calculation tracking error
		ew = pos - rpos;
		ew1 = vel - rvel;
		ew2 = acc - racc;

		// calculation control value
		force = prod(mass,(racc-prod(kvw,ew1)-prod(kpw,ew))) + prod(c,vel);

		poss = posall(2);
		vels = velall(2);
		accs = accall(2);

		rposs = pos(1);
		rvels = vel(1);
		raccs = acc(1);

		// calculation sync error
		es = poss - rposs;
		es1 = vels - rvels;
		es2 = accs - raccs;

		// calculation control value(sync)
		forces = masss*( -kvs*es1-kps*es ) + cs*es1;

		forceall(0) = force(0);
		forceall(1) = force(1);
		forceall(2) = force(1) + forces;

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);

		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// calculation contouring error
		ec = CalcContouringError(pos);

		// loging data
		rpos_log(0) = rpos(0);
		rpos_log(1) = rpos(1);
		rpos_log(2) = rposs;
		rvel_log(0) = rvel(0);
		rvel_log(1) = rvel(1);
		rvel_log(2) = rvels;
		racc_log(0) = racc(0);
		racc_log(1) = racc(1);
		racc_log(2) = raccs;
		pos_log(0) = pos(0);
		pos_log(1) = pos(1);
		pos_log(2) = poss;
		vel_log(0) = vel(0);
		vel_log(1) = vel(1);
		vel_log(2) = vels;
		acc_log(0) = acc(0);
		acc_log(1) = acc(1);
		acc_log(2) = accs;
		ew_log(0) = ew(0);
		ew_log(1) = ew(1);
		ew_log(2) = poss - rpos(1);//es(0);
		force_log = forceall;
		datalog[0,step] = time;
		datalog[1,step] = ec;
		datalog[2,step] = 0;	//ec-en
		datalog[3,step] = 0;	//ec-enmod
		datalog[4,step] = 0;	//td
		datalog[5,step] = 0;	//el t
		datalog[6,step] = 0;	//el n
		datalog[7,step] = 0;	//el b
		datalog[8,step] = es;	//es y
		datalog[9,step] = 0;	//es a
		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
			datalog[j+10,step] = rpos_log(i);
			datalog[j+11,step] = rvel_log(i);
			datalog[j+12,step] = racc_log(i);
			datalog[j+13,step] = pos_log(i);
			datalog[j+14,step] = vel_log(i);
			datalog[j+15,step] = acc_log(i);
			datalog[j+16,step] = force_log(i);
			datalog[j+17,step] = ew_log(i);
		}
	}
}

void control2d::IndependentCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	
	unsigned int i,j;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);
					
	double			es,es1,es2,
					poss,vels,accs,
					rposs,rvels,raccs,
					forces,time,ec;

	time = step * SAMPLING_TIME;
		
	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);

	vel(0) = velall(0);
	vel(1) = velall(1);

	acc(0) = accall(0);
	acc(1) = accall(1);

	// get reference
	GenerateReference(time,rpos,rvel,racc);

	// calculation tracking error
	ew = pos - rpos;
	ew1 = vel - rvel;
	ew2 = acc - racc;

	// calculation control value
	force = prod(mass,(racc-prod(kvw,ew1)-prod(kpw,ew))) + prod(c,vel);

	poss = posall(2);
	vels = velall(2);
	accs = accall(2);

	rposs = pos(1);
	rvels = vel(1);
	raccs = acc(1);

	// calculation sync error
	es = poss - rposs;
	es1 = vels - rvels;
	es2 = accs - raccs;

	// calculation control value(sync)
	forces = masss*( -kvs*es1-kps*es ) + cs*es1;

	forceall(0) = force(0);
	forceall(1) = force(1);
	forceall(2) = -(force(1) + forces);

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// calculation contouring error
	ec = CalcContouringError(pos);

	// loging data
	rpos_log(0) = rpos(0);
	rpos_log(1) = rpos(1);
	rpos_log(2) = rposs;
	rvel_log(0) = rvel(0);
	rvel_log(1) = rvel(1);
	rvel_log(2) = rvels;
	racc_log(0) = racc(0);
	racc_log(1) = racc(1);
	racc_log(2) = raccs;
	pos_log(0) = pos(0);
	pos_log(1) = pos(1);
	pos_log(2) = poss;
	vel_log(0) = vel(0);
	vel_log(1) = vel(1);
	vel_log(2) = vels;
	acc_log(0) = acc(0);
	acc_log(1) = acc(1);
	acc_log(2) = accs;
	ew_log(0) = ew(0);
	ew_log(1) = ew(1);
	ew_log(2) = poss - rpos(1);//es(0);
	force_log = forceall;
	datalog[0,step] = time;
	datalog[1,step] = ec;
	datalog[2,step] = 0;	//ec-en
	datalog[3,step] = 0;	//ec-enmod
	datalog[4,step] = 0;	//td
	datalog[5,step] = 0;	//el t
	datalog[6,step] = 0;	//el n
	datalog[7,step] = 0;	//el b
	datalog[8,step] = es;	//es y
	datalog[9,step] = 0;	//es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = rpos_log(i);
		datalog[j+11,step] = rvel_log(i);
		datalog[j+12,step] = racc_log(i);
		datalog[j+13,step] = pos_log(i);
		datalog[j+14,step] = vel_log(i);
		datalog[j+15,step] = acc_log(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = ew_log(i);
	}
}

void control2d::ContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	
	unsigned int step,i,j;
	PLANT			plant;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					el(ew),el1(ew),el2(ew),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);

	matrix<double> Rot(2,2),Rot1(Rot),Rot2(Rot),tRot,tRot1,tRot2;
					
	double			es,es1,es2,
					poss,vels,accs,
					rposs,rvels,raccs,
					forces,time,ec;

	for(step = 0;step < max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		// get state value
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
		pos(0) = posall(0);
		pos(1) = posall(1);

		vel(0) = velall(0);
		vel(1) = velall(1);

		acc(0) = accall(0);
		acc(1) = accall(1);

		// get reference
		GenerateReference(time,rpos,rvel,racc);

		// calculation tracking error
		ew = pos - rpos;
		ew1 = vel - rvel;
		ew2 = acc - racc;
		
		// transform error
		CalcRotationMat(time,Rot,Rot1,Rot2);
		tRot = trans(Rot);
		tRot1 = trans(Rot1);
		tRot2 = trans(Rot2);
		el = prod(tRot,ew);
		el1 = prod(tRot1,ew) + prod(tRot,ew1);
		el2 = prod(tRot2,ew) + 2*prod(tRot1,ew1) + prod(tRot,ew2);
		
		// calculation control value
		force = prod(mass,racc - prod(Rot,prod(kvl,el1) + prod(kpl,el) + prod(tRot2,ew) + 2*prod(tRot1,ew1))) + prod(c,vel);

		poss = posall(2);
		vels = velall(2);
		accs = accall(2);

		rposs = pos(1);
		rvels = vel(1);
		raccs = acc(1);

		// calculation sync error
		es = poss - rposs;
		es1 = vels - rvels;
		es2 = accs - raccs;

		// calculation control value(sync)
		forces = masss*( -kvs*es1-kps*es ) + cs*es1;

		forceall(0) = force(0);
		forceall(1) = force(1);
		forceall(2) = force(1) + forces;

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);

		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// calculation contouring error
		ec = CalcContouringError(pos);

		// loging data
		rpos_log(0) = rpos(0);
		rpos_log(1) = rpos(1);
		rpos_log(2) = rposs;
		rvel_log(0) = rvel(0);
		rvel_log(1) = rvel(1);
		rvel_log(2) = rvels;
		racc_log(0) = racc(0);
		racc_log(1) = racc(1);
		racc_log(2) = raccs;
		pos_log(0) = pos(0);
		pos_log(1) = pos(1);
		pos_log(2) = poss;
		vel_log(0) = vel(0);
		vel_log(1) = vel(1);
		vel_log(2) = vels;
		acc_log(0) = acc(0);
		acc_log(1) = acc(1);
		acc_log(2) = accs;
		ew_log(0) = ew(0);
		ew_log(1) = ew(1);
		ew_log(2) = poss - rpos(1);//es(0);
		force_log = forceall;
		datalog[0,step] = time;
		datalog[1,step] = ec;
		datalog[2,step] = -el(1) - ec;	//ec-en
		datalog[3,step] = 0;			//ec-enmod
		datalog[4,step] = 0;			//td
		datalog[5,step] = el(0);		//el t
		datalog[6,step] = el(1);		//el n
		datalog[7,step] = 0;			//el b
		datalog[8,step] = es;			//es y
		datalog[9,step] = 0;			//es a
		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
			datalog[j+10,step] = rpos_log(i);
			datalog[j+11,step] = rvel_log(i);
			datalog[j+12,step] = racc_log(i);
			datalog[j+13,step] = pos_log(i);
			datalog[j+14,step] = vel_log(i);
			datalog[j+15,step] = acc_log(i);
			datalog[j+16,step] = force_log(i);
			datalog[j+17,step] = ew_log(i);
		}
	}
}

void control2d::ContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	
	unsigned int i,j;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					el(ew),el1(ew),el2(ew),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);

	matrix<double> Rot(2,2),Rot1(Rot),Rot2(Rot),tRot,tRot1,tRot2;
					
	double			es,es1,es2,
					poss,vels,accs,
					rposs,rvels,raccs,
					forces,time,ec;

	time = step * SAMPLING_TIME;
		
	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);

	vel(0) = velall(0);
	vel(1) = velall(1);

	acc(0) = accall(0);
	acc(1) = accall(1);

	// get reference
	GenerateReference(time,rpos,rvel,racc);

	// calculation tracking error
	ew = pos - rpos;
	ew1 = vel - rvel;
	ew2 = acc - racc;
		
	// transform error
	CalcRotationMat(time,Rot,Rot1,Rot2);
	tRot = trans(Rot);
	tRot1 = trans(Rot1);
	tRot2 = trans(Rot2);
	el = prod(tRot,ew);
	el1 = prod(tRot1,ew) + prod(tRot,ew1);
	el2 = prod(tRot2,ew) + 2*prod(tRot1,ew1) + prod(tRot,ew2);
		
	// calculation control value
	force = prod(mass,racc - prod(Rot,prod(kvl,el1) + prod(kpl,el) + prod(tRot2,ew) + 2*prod(tRot1,ew1))) + prod(c,vel);

	poss = posall(2);
	vels = velall(2);
	accs = accall(2);

	rposs = pos(1);
	rvels = vel(1);
	raccs = acc(1);

	// calculation sync error
	es = poss - rposs;
	es1 = vels - rvels;
	es2 = accs - raccs;

	// calculation control value(sync)
	forces = masss*( -kvs*es1-kps*es ) + cs*es1;

	forceall(0) = force(0);
	forceall(1) = force(1);
	forceall(2) = -(force(1) + forces);

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// calculation contouring error
	ec = CalcContouringError(pos);

	// loging data
	rpos_log(0) = rpos(0);
	rpos_log(1) = rpos(1);
	rpos_log(2) = rposs;
	rvel_log(0) = rvel(0);
	rvel_log(1) = rvel(1);
	rvel_log(2) = rvels;
	racc_log(0) = racc(0);
	racc_log(1) = racc(1);
	racc_log(2) = raccs;
	pos_log(0) = pos(0);
	pos_log(1) = pos(1);
	pos_log(2) = poss;
	vel_log(0) = vel(0);
	vel_log(1) = vel(1);
	vel_log(2) = vels;
	acc_log(0) = acc(0);
	acc_log(1) = acc(1);
	acc_log(2) = accs;
	ew_log(0) = ew(0);
	ew_log(1) = ew(1);
	ew_log(2) = poss - rpos(1);//es(0);
	force_log = forceall;
	datalog[0,step] = time;
	datalog[1,step] = ec;
	datalog[2,step] = -el(1) - ec;	//ec-en
	datalog[3,step] = 0;			//ec-enmod
	datalog[4,step] = 0;			//td
	datalog[5,step] = el(0);		//el t
	datalog[6,step] = el(1);		//el n
	datalog[7,step] = 0;			//el b
	datalog[8,step] = es;			//es y
	datalog[9,step] = 0;			//es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = rpos_log(i);
		datalog[j+11,step] = rvel_log(i);
		datalog[j+12,step] = racc_log(i);
		datalog[j+13,step] = pos_log(i);
		datalog[j+14,step] = vel_log(i);
		datalog[j+15,step] = acc_log(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = ew_log(i);
	}
}

void control2d::modContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	
	unsigned int step,i,j;
	PLANT			plant;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					el(ew),el1(el),el2(el),
					ewn(ew),ewn1(ewn),ewn2(ewn),
					ela(ew),ela1(ela),ela2(ela),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					rposa(2),rvela(rposa),racca(rposa),
					rposn(rposa),rveln(rposn),raccn(rposn),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);

	matrix<double>	Rot(2,2),Rot1(Rot),Rot2(Rot),
					tRot,tRot1,tRot2,
					Rota(Rot),Rota1(Rota),Rota2(Rota),
					tRota,tRota1,tRota2,
					Q(Rot),Q1(Q),Q2(Q),S(Q);
					
	double			es,es1,es2,
					poss,vels,accs,
					rposs,rvels,raccs,
					forces,time,ec,
					td,dv;

	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		// get state value
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
		pos(0) = posall(0);
		pos(1) = posall(1);

		vel(0) = velall(0);
		vel(1) = velall(1);

		acc(0) = accall(0);
		acc(1) = accall(1);

		// get reference
		GenerateReference(time,rpos,rvel,racc);

		// calculation tracking error
		ew = pos - rpos;
		ew1 = vel - rvel;
		ew2 = acc - racc;
		
		// transform error
		CalcRotationMat(time,Rot,Rot1,Rot2);
		tRot = trans(Rot);
		tRot1 = trans(Rot1);
		tRot2 = trans(Rot2);
		el = prod(tRot,ew);
		el1 = prod(tRot1,ew) + prod(tRot,ew1);
		el2 = prod(tRot2,ew) + 2*prod(tRot1,ew1) + prod(tRot,ew2);

		// calculation delay time
		dv = norm_2(rvel);	//OMG * RADIUS;
		td = -el(0) / dv;

		// calculation r^a
		GenerateReference(time-td,rposa,rvela,racca);

		// calculation rn
		CalcRotationMat(time-td,Rota,Rota1,Rota2);
		tRota = trans(Rota);
		tRota1 = trans(Rota1);
		tRota2 = trans(Rota2);

		S.clear();
		S(1,1) = 1;
		Q = prod((matrix<double>)prod(Rota,S),tRota);
		Q1 = prod((matrix<double>)prod(Rota1,S),tRota)
				+ prod((matrix<double>)prod(Rota,S),tRota1);
		Q2 = prod((matrix<double>)prod(Rota2,S),tRota)
				+ 2 * prod((matrix<double>)prod(Rota1,S),tRota1)
				+ prod((matrix<double>)prod(Rota,S),tRota2);

		rposn = rpos + prod(Q,(rposa - rpos));
		rveln = rvel + prod(Q1,(rposa - rpos)) + prod(Q,(rvela - rvel));
		raccn = racc + prod(Q2,(rposa - rpos)) + 2*prod(Q1,(rvela - rvel)) + prod(Q,(racca - racc));

		ewn = pos - rposn;
		ewn1 = vel - rveln;
		ewn2 = acc - raccn;
		
		ela = prod(tRota,ewn);
		ela1 = prod(tRota1,ewn) + prod(tRota,ewn1);
		ela2 = prod(tRota2,ewn) + 2*prod(tRota1,ewn1) + prod(tRota,ewn2);
		
		// calculation control value
		force = prod(mass,raccn - prod(Rota,prod(kvl,ela1) + prod(kpl,ela) + prod(tRota2,ewn) + 2*prod(tRota1,ewn1))) + prod(c,vel);

		poss = posall(2);
		vels = velall(2);
		accs = accall(2);

		rposs = pos(1);
		rvels = vel(1);
		raccs = acc(1);

		// calculation sync error
		es = poss - rposs;
		es1 = vels - rvels;
		es2 = accs - raccs;

		// calculation control value(sync)
		forces = masss*( -kvs*es1-kps*es ) + cs*es1;

		forceall(0) = force(0);
		forceall(1) = force(1);
		forceall(2) = force(1) + forces;

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);

		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// calculation contouring error
		ec = CalcContouringError(pos);

		// loging data
		rpos_log(0) = rpos(0);
		rpos_log(1) = rpos(1);
		rpos_log(2) = rposs;
		rvel_log(0) = rvel(0);
		rvel_log(1) = rvel(1);
		rvel_log(2) = rvels;
		racc_log(0) = racc(0);
		racc_log(1) = racc(1);
		racc_log(2) = raccs;
		pos_log(0) = pos(0);
		pos_log(1) = pos(1);
		pos_log(2) = poss;
		vel_log(0) = vel(0);
		vel_log(1) = vel(1);
		vel_log(2) = vels;
		acc_log(0) = acc(0);
		acc_log(1) = acc(1);
		acc_log(2) = accs;
		ew_log(0) = ew(0);
		ew_log(1) = ew(1);
		ew_log(2) = poss - rpos(1);//es(0);
		force_log = forceall;
		datalog[0,step] = time;
		datalog[1,step] = ec;
		datalog[2,step] = -el(1) - ec;	//ec-en
		datalog[3,step] = -ela(1) - ec;	//ec-enmod
		datalog[4,step] = td;			//td
		datalog[5,step] = ela(0);		//el t
		datalog[6,step] = ela(1);		//el n
		datalog[7,step] = 0;			//el b
		datalog[8,step] = es;			//es y
		datalog[9,step] = 0;			//es a
		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
			datalog[j+10,step] = rpos_log(i);
			datalog[j+11,step] = rvel_log(i);
			datalog[j+12,step] = racc_log(i);
			datalog[j+13,step] = pos_log(i);
			datalog[j+14,step] = vel_log(i);
			datalog[j+15,step] = acc_log(i);
			datalog[j+16,step] = force_log(i);
			datalog[j+17,step] = ew_log(i);
		}
	}
}

void control2d::modContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	
	unsigned int i,j;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					el(ew),el1(el),el2(el),
					ewn(ew),ewn1(ewn),ewn2(ewn),
					ela(ew),ela1(ela),ela2(ela),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					rposa(2),rvela(rposa),racca(rposa),
					rposn(rposa),rveln(rposn),raccn(rposn),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);

	matrix<double>	Rot(2,2),Rot1(Rot),Rot2(Rot),
					tRot,tRot1,tRot2,
					Rota(Rot),Rota1(Rota),Rota2(Rota),
					tRota,tRota1,tRota2,
					Q(Rot),Q1(Q),Q2(Q),S(Q);
					
	double			es,es1,es2,
					poss,vels,accs,
					rposs,rvels,raccs,
					forces,time,ec,
					td,dv;

	time = step * SAMPLING_TIME;
		
	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);

	vel(0) = velall(0);
	vel(1) = velall(1);

	acc(0) = accall(0);
	acc(1) = accall(1);

	// get reference
	GenerateReference(time,rpos,rvel,racc);

	// calculation tracking error
	ew = pos - rpos;
	ew1 = vel - rvel;
	ew2 = acc - racc;
		
	// transform error
	CalcRotationMat(time,Rot,Rot1,Rot2);
	tRot = trans(Rot);
	tRot1 = trans(Rot1);
	tRot2 = trans(Rot2);
	el = prod(tRot,ew);
	el1 = prod(tRot1,ew) + prod(tRot,ew1);
	el2 = prod(tRot2,ew) + 2*prod(tRot1,ew1) + prod(tRot,ew2);

	// calculation delay time
	dv = norm_2(rvel);	//OMG * RADIUS;
	td = -el(0) / dv;

	// calculation r^a
	GenerateReference(time-td,rposa,rvela,racca);

	// calculation rn
	CalcRotationMat(time-td,Rota,Rota1,Rota2);
	tRota = trans(Rota);
	tRota1 = trans(Rota1);
	tRota2 = trans(Rota2);

	S.clear();
	S(1,1) = 1;
	Q = prod((matrix<double>)prod(Rota,S),tRota);
	Q1 = prod((matrix<double>)prod(Rota1,S),tRota)
			+ prod((matrix<double>)prod(Rota,S),tRota1);
	Q2 = prod((matrix<double>)prod(Rota2,S),tRota)
			+ 2 * prod((matrix<double>)prod(Rota1,S),tRota1)
			+ prod((matrix<double>)prod(Rota,S),tRota2);

	rposn = rpos + prod(Q,(rposa - rpos));
	rveln = rvel + prod(Q1,(rposa - rpos)) + prod(Q,(rvela - rvel));
	raccn = racc + prod(Q2,(rposa - rpos)) + 2*prod(Q1,(rvela - rvel)) + prod(Q,(racca - racc));

	ewn = pos - rposn;
	ewn1 = vel - rveln;
	ewn2 = acc - raccn;
		
	ela = prod(tRota,ewn);
	ela1 = prod(tRota1,ewn) + prod(tRota,ewn1);
	ela2 = prod(tRota2,ewn) + 2*prod(tRota1,ewn1) + prod(tRota,ewn2);
		
	// calculation control value
	force = prod(mass,raccn - prod(Rota,prod(kvl,ela1) + prod(kpl,ela) + prod(tRota2,ewn) + 2*prod(tRota1,ewn1))) + prod(c,vel);

	poss = posall(2);
	vels = velall(2);
	accs = accall(2);

	rposs = pos(1);
	rvels = vel(1);
	raccs = acc(1);

	// calculation sync error
	es = poss - rposs;
	es1 = vels - rvels;
	es2 = accs - raccs;

	// calculation control value(sync)
	forces = masss*( -kvs*es1-kps*es ) + cs*es1;

	forceall(0) = force(0);
	forceall(1) = force(1);
	forceall(2) = -(force(1) + forces);

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// calculation contouring error
	ec = CalcContouringError(pos);

	// loging data
	rpos_log(0) = rpos(0);
	rpos_log(1) = rpos(1);
	rpos_log(2) = rposs;
	rvel_log(0) = rvel(0);
	rvel_log(1) = rvel(1);
	rvel_log(2) = rvels;
	racc_log(0) = racc(0);
	racc_log(1) = racc(1);
	racc_log(2) = raccs;
	pos_log(0) = pos(0);
	pos_log(1) = pos(1);
	pos_log(2) = poss;
	vel_log(0) = vel(0);
	vel_log(1) = vel(1);
	vel_log(2) = vels;
	acc_log(0) = acc(0);
	acc_log(1) = acc(1);
	acc_log(2) = accs;
	ew_log(0) = ew(0);
	ew_log(1) = ew(1);
	ew_log(2) = poss - rpos(1);//es(0);
	force_log = forceall;
	datalog[0,step] = time;
	datalog[1,step] = ec;
	datalog[2,step] = -el(1) - ec;	//ec-en
	datalog[3,step] = -ela(1) - ec;	//ec-enmod
	datalog[4,step] = td;			//td
	datalog[5,step] = ela(0);		//el t
	datalog[6,step] = ela(1);		//el n
	datalog[7,step] = 0;			//el b
	datalog[8,step] = es;			//es y
	datalog[9,step] = 0;			//es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = rpos_log(i);
		datalog[j+11,step] = rvel_log(i);
		datalog[j+12,step] = racc_log(i);
		datalog[j+13,step] = pos_log(i);
		datalog[j+14,step] = vel_log(i);
		datalog[j+15,step] = acc_log(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = ew_log(i);
	}
}

void control2d::Identification(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control2d::conparam;
	unsigned int i,j;
	vector<double>	ew(2),ew1(ew),ew2(ew),
					pos(2),vel(pos),acc(pos),
					rpos(2),rvel(rpos),racc(rpos),
					force(2),

					posall(NUM_ACTUATOR),velall(posall),accall(posall),
					forceall(NUM_ACTUATOR,0),

					ew_log(NUM_ACTUATOR,0),
					pos_log(NUM_ACTUATOR,0),vel_log(pos_log),acc_log(pos_log),
					rpos_log(NUM_ACTUATOR,0),rvel_log(rpos_log),racc_log(rpos_log),
					force_log(NUM_ACTUATOR,0);
		
	double			time;

	time = SAMPLING_TIME * step;

	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);

	vel(0) = velall(0);
	vel(1) = velall(1);

	acc(0) = accall(0);
	acc(1) = accall(1);

	rpos(0) = 0.01*sin(time*OMG);
	double e = rpos(0) - posall(3);
	forceall.clear();
	//forceall(3) = e*1200;
	forceall(3) = 7.0*sin(time*OMG);
	//forceall(3) = -10;
	//forceall(6) = -forceall(5);
	

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// loging data
	force_log = forceall;
	datalog[0,step] = time;
	datalog[1,step] = 0;
	datalog[2,step] = 0;	//ec-en
	datalog[3,step] = 0;	//ec-enmod
	datalog[4,step] = 0;	//td
	datalog[5,step] = 0;	//el t
	datalog[6,step] = 0;	//el n
	datalog[7,step] = 0;	//el b
	datalog[8,step] = 0;	//es y
	datalog[9,step] = 0;	//es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = rpos_log(i);
		datalog[j+11,step] = rvel_log(i);
		datalog[j+12,step] = racc_log(i);
		datalog[j+13,step] = posall(i);
		datalog[j+14,step] = velall(i);
		datalog[j+15,step] = accall(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = ew_log(i);
	}
}
