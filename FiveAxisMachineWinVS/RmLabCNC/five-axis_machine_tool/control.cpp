#include "stdafx.h"
#include "control.h"
#include "math.hpp"
#include "io.h"
#include "plant.h"
#include "observer.h"
#include <cmath>


void control::ControlSim(unsigned int control_type,bool select_3d,unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	if(select_3d){
		switch(control_type){
		case INDEPENDENT:
			IndependentCon5(max_step,enable_noise,enable_resolution,enable_sync,datalog);
			break;
		case CONTOURING:
			ContouringCon5(max_step,enable_noise,enable_resolution,enable_sync,datalog);
			break;
		case MOD_CONTOURING:
			modContouringCon5(max_step,enable_noise,enable_resolution,enable_sync,datalog);
			break;
		default:
			IndependentCon5(max_step,enable_noise,enable_resolution,enable_sync,datalog);
			break;
		}
	}else{
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
}

void control::SetConParam(array<double>^ conparam)
{
		using namespace control::conparam;

		mass_x		= conparam[0];
		mass_y1		= conparam[1];
		mass_y2		= conparam[2];
		mass_z		= conparam[3];
		mass_c		= conparam[4];
		mass_a1		= conparam[5];
		mass_a2		= conparam[6];
		c_x			= conparam[7];
		c_y1		= conparam[8];
		c_y2		= conparam[9];
		c_z			= conparam[10];
		c_c			= conparam[11];
		c_a1		= conparam[12];
		c_a2		= conparam[13];
		pole_x		= conparam[14];
		pole_y1		= conparam[15];
		pole_y2		= conparam[16];
		pole_z		= conparam[17];
		pole_c		= conparam[18];
		pole_a1		= conparam[19];
		pole_a2		= conparam[20];
		pole_t		= conparam[21];
		pole_n		= conparam[22];
		pole_b		= conparam[23];
		pole_i		= conparam[24];
		pole_j		= conparam[25];
		pole_k		= conparam[26];
		pole_sync_y	= conparam[27];
		pole_sync_a	= conparam[28];

		mass.resize(NUM_DOF,NUM_DOF);
		mass_inv.resize(NUM_DOF,NUM_DOF);
		c.resize(NUM_DOF,NUM_DOF);
		kpw.resize(NUM_DOF,NUM_DOF);
		kvw.resize(NUM_DOF,NUM_DOF);
		kpl.resize(NUM_DOF,NUM_DOF);
		kvl.resize(NUM_DOF,NUM_DOF);
		mass.clear();
		c.clear();
		kpw.clear();
		kvw.clear();
		kpl.clear();
		kvl.clear();
		masss.resize(2,2);
		cs.resize(2,2);
		kps.resize(2,2);
		kvs.resize(2,2);
		masss.clear();
		cs.clear();
		kps.clear();
		kvs.clear();

		mass(0,0) = mass_x;mass(1,1) = mass_y1;mass(2,2) = mass_z;
		mass(3,3) = mass_c;mass(4,4) = mass_a1;
		c(0,0) = c_x;c(1,1) = c_y1;c(2,2) = c_z;
		c(3,3) = c_c;c(4,4) = c_a1;
		math::invert(mass,mass_inv);

		kpw(0,0) = pole_x*pole_x;kpw(1,1) = pole_y1*pole_y1;kpw(2,2) = pole_z*pole_z;
		kpw(3,3) = pole_c*pole_c;kpw(4,4) = pole_a1*pole_a1;
		kvw(0,0) = 2*pole_x;kvw(1,1) = 2*pole_y1;kvw(2,2) = 2*pole_z;
		kvw(3,3) = 2*pole_c;kvw(4,4) = 2*pole_a1;

		kpl(0,0) = pole_t * pole_t;
		kpl(1,1) = pole_n * pole_n;
		kpl(2,2) = pole_b * pole_b;
		kpl(3,3) = pole_c * pole_c;
		kpl(4,4) = pole_a1 * pole_a1;
		kvl(0,0) = 2 * pole_t;
		kvl(1,1) = 2 * pole_n;
		kvl(2,2) = 2 * pole_b;
		kvl(3,3) = 2 * pole_c;
		kvl(4,4) = 2 * pole_a1;

		masss(0,0) = mass_y2;
		masss(1,1) = mass_a2;
		cs(0,0) = c_y2;
		cs(1,1) = c_a2;
		kvs(0,0) = 2 * pole_sync_y;
		kvs(1,1) = 2 * pole_sync_a;
		kps(0,0) = pole_sync_y * pole_sync_y;
		kps(1,1) = pole_sync_a * pole_sync_a;

		kvo.resize(3,3);kpo.resize(kvo.size1(),kvo.size2());
		kvo.clear();kpo.clear();
		kvo(0,0) = 2 * pole_i;
		kvo(1,1) = 2 * pole_j;
		kvo(2,2) = 2 * pole_k;
		kpo(0,0) = pole_i * pole_i;
		kpo(1,1) = pole_j * pole_j;
		kpo(2,2) = pole_k * pole_k;
		Mo.resize(2,2);Co.resize(Mo.size1(),Mo.size2());
		Mo.clear();Co.clear();
		Mo(0,0) = mass(4,4);Mo(1,1) = mass(3,3);
		Co(0,0) = c(4,4);	Co(1,1) = c(3,3);
}

bool control::IsReachPoint(double time,double max_angle,array<double,2>^ datalog)
{
	//double rangle,x,y,oldx,oldy,angle;
	//unsigned int step = static_cast<unsigned int>(time/SAMPLING_TIME);
	//static double start_angle,total_angle;

	/*x = datalog[13,step];
	y = datalog[21,step];

	if(time == 0){
		oldx = INIT_POS_X;
		oldy = INIT_POS_Y1;
		total_angle = 0;
		start_angle = 0;
	}
	else{
		oldx = datalog[13,step-1];
		oldy = datalog[21,step-1];
	}

	rangle = max_angle;

	if(x == 0){
		if(oldx > 0)
			angle = PI;
		else
			angle = 3*PI/4;
	}
	else if(y == 0){
		if(oldy > 0)
			angle = 2*PI;
		else
			angle = 0;
	}
	else if(x>0 && y>0)
		angle = atan(y/x);
	else if(x<0 && y>0)
		angle = PI + atan(y/x);
	else if(x<0 && y<0)
		angle = PI + atan(y/x);
	else
		angle = 2*PI + atan(y/x);

	angle -= start_angle;

	if(time == 0)
		start_angle = angle;

	total_angle += (angle)*180/PI;*/

	volatile double sdot,rlength,rangle,angle,
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

void control::GenerateReference(double time,vector<double>& rpos,vector<double>& rvel,vector<double>& racc)
{
	double theta = OMG * time;

	rpos(0) =  RADIUS * sin(theta);					// x-axis
	rpos(1) = -RADIUS * cos(theta);					// y1-axis

	rvel(0) = RADIUS * OMG * cos(theta);			// x-axis
	rvel(1) = RADIUS * OMG * sin(theta);			// y1-axis

	racc(0) = -RADIUS * OMG * OMG * sin(theta);		// x-axis
	racc(1) =  RADIUS * OMG * OMG * cos(theta);		// y1-axis
}

void control::GenerateWReference(double time,vector<double>& wref,vector<double>& wrefdot,vector<double>& wrefddot)
{
	const double theta = OMG * time;

	/*
	const double SQ3 = sqrt(3.);
	double f,fdot,fddot,g,gdot,gddot;

	f = SQ3*sin(theta);
	fdot = SQ3*OMG*cos(theta);
	fddot = -SQ3*OMG*OMG*sin(theta);
	g = SQ3/2*cos(theta);
	gdot = SQ3/2*OMG*-sin(theta);
	gddot = -SQ3/2*OMG*OMG*cos(theta);
	*/

	wref(0)		=  0;	// x-axis
	wref(0)		=  RADIUS*sin(theta);	// y1-axis
	wref(1)		=  RADIUS*sin(theta);	// y1-axis
	wref(2)		=  RADIUS*cos(theta);	// z-axis
	wref(3)		=  0;	// c-axis
	wref(3)		=  20*PI/180*sin(theta);	// a1-axis
	wref(4)		=  20*PI/180*sin(theta);	// a1-axis							

	wrefdot(0)	=  0;	// x-axis
	wrefdot(0)	=  RADIUS*OMG*cos(theta);	// y1-axis
	wrefdot(1)	=  RADIUS*OMG*cos(theta);	// y1-axis
	wrefdot(2)	=  RADIUS*OMG*-sin(theta);	// z-axis
	wrefdot(3)	=  0;	// c-axis
	wrefdot(3)	=  20*PI/180*OMG*cos(theta);	// a1-axis
	wrefdot(4)	=  20*PI/180*OMG*cos(theta);	// a1-axis

	wrefddot(0)	=  0;		// x-axis
	wrefddot(0)	= -RADIUS*OMG*OMG*sin(theta);		// y1-axis
	wrefddot(1)	= -RADIUS*OMG*OMG*sin(theta);		// y1-axis
	wrefddot(2)	= -RADIUS*OMG*OMG*cos(theta);	// z-axis
	wrefddot(3)	=  0;	// c-axis
	wrefddot(3)	= -20*PI/180*OMG*OMG*sin(theta);	// a1-axis
	wrefddot(4)	= -20*PI/180*OMG*OMG*sin(theta);	// a1-axis
}

void GenerateWReference3dot(double time,vector<double>& wref3)
{
	const double theta = OMG * time;

	wref3(0) =  0;		// x-axis
	wref3(0) = -RADIUS*OMG*OMG*OMG*cos(theta);		// y1-axis
	wref3(1) = -RADIUS*OMG*OMG*OMG*cos(theta);		// y1-axis
	wref3(2) =  RADIUS*OMG*OMG*OMG*sin(theta);	// z-axis
	wref3(3) =  0;	// c-axis
	wref3(3) = -20*PI/180*OMG*OMG*OMG*cos(theta);	// a1-axis
	wref3(4) = -20*PI/180*OMG*OMG*OMG*cos(theta);	// a1-axis
}

void GenerateWReference4dot(double time,vector<double>& wref4)
{
	const double theta = OMG * time;

	wref4(0) =  0;		// x-axis
	wref4(0) =  RADIUS*OMG*OMG*OMG*OMG*sin(theta);		// y1-axis
	wref4(1) =  RADIUS*OMG*OMG*OMG*OMG*sin(theta);		// y1-axis
	wref4(2) =  RADIUS*OMG*OMG*OMG*OMG*cos(theta);	// z-axis
	wref4(3) =  0;	// c-axis
	wref4(3) =  20*PI/180*OMG*OMG*OMG*OMG*sin(theta);	// a1-axis
	wref4(4) =  20*PI/180*OMG*OMG*OMG*OMG*sin(theta);	// a1-axis
}

void control::CalcConvertMatrixT(matrix<double>& T,matrix<double>& TT,vector<double>& q)
{
	double thc = q(3),tha1 = q(4);
	double sa = sin(tha1),sc = sin(thc),ca = cos(tha1),cc = cos(thc);
	/*
	T(0,0) = -cos(thc);	T(0,1) = -sin(thc)*cos(tha1);	T(0,2) = sin(tha1)*sin(thc);T(0,3) = 0;	T(0,4) = 0;
	T(1,0) =  sin(thc);	T(1,1) = -cos(thc)*cos(tha1);	T(1,2) = sin(tha1)*cos(thc);T(1,3) = 0;	T(1,4) = 0;
	T(2,0) =  0;		T(2,1) =  sin(tha1);			T(2,2) = cos(tha1);			T(2,3) = 0;	T(2,4) = 0;
	T(3,0) =  0;		T(3,1) =  0;					T(3,2) = 0;					T(3,3) = 1;	T(3,4) = 0;
	T(4,0) =  0;		T(4,1) =  0;					T(4,2) = 0;					T(4,3) = 0;	T(4,4) = 1;
	*/
	T(0,0) = -cc;	T(0,1) = -sc*ca;	T(0,2) = sa*sc;	T(0,3) = 0;	T(0,4) = 0;
	T(1,0) =  sc;	T(1,1) = -cc*ca;	T(1,2) = sa*cc;	T(1,3) = 0;	T(1,4) = 0;
	T(2,0) =  0;	T(2,1) =  sa;		T(2,2) = ca;	T(2,3) = 0;	T(2,4) = 0;
	T(3,0) =  0;	T(3,1) =  0;		T(3,2) = 0;		T(3,3) = 1;	T(3,4) = 0;
	T(4,0) =  0;	T(4,1) =  0;		T(4,2) = 0;		T(4,3) = 0;	T(4,4) = 1;

	TT(0,0) =  0;		TT(0,1) = -sc*sa;
	TT(1,0) =  0;		TT(1,1) = -cc*sa;
	TT(2,0) = -1;		TT(2,1) = -ca;
	TT(3,0) =  0;		TT(3,1) = 0;
	TT(4,0) =  0;		TT(4,1) = 0;
}

void control::CalcJ(matrix<double>& J,vector<double>& q)
{
	double x = q(0),y1 = q(1),z = q(2),thc = q(3),tha1 = q(4),
		   a2 = OFFSET_BETWEEN_COORDINATE;
	double sa = sin(tha1),sc = sin(thc),ca = cos(tha1),cc = cos(thc);
	/*
	J(0,0) = -cos(thc);
	J(0,1) = -sin(thc)*cos(tha1);
	J(0,2) =  sin(thc)*sin(tha1);
	J(0,3) =  x*sin(thc) - y1*cos(thc)*cos(tha1) + z*cos(thc)*sin(tha1) - a2*cos(thc)*sin(tha1);
	J(0,4) =			   y1*sin(thc)*sin(tha1) + z*sin(thc)*cos(tha1) - a2*sin(thc)*cos(tha1);

	J(1,0) =  sin(thc);
	J(1,1) = -cos(thc)*cos(tha1);
	J(1,2) =  cos(thc)*sin(tha1);
	J(1,3) =  x*cos(thc) + y1*sin(thc)*cos(tha1) - z*sin(thc)*sin(tha1) + a2*sin(thc)*sin(tha1);
	J(1,4) =			   y1*cos(thc)*sin(tha1) + z*cos(thc)*cos(tha1) - a2*cos(thc)*cos(tha1);
	*/

	J(0,0) = -cc;
	J(0,1) = -sc*ca;
	J(0,2) =  sc*sa;
	J(0,3) =  x*sc - y1*cc*ca + z*cc*sa - a2*cc*sa;
	J(0,4) =		 y1*sc*sa + z*sc*ca - a2*sc*ca;

	J(1,0) =  sc;
	J(1,1) = -cc*ca;
	J(1,2) =  cc*sa;
	J(1,3) =  x*cc + y1*sc*ca - z*sc*sa + a2*sc*sa;
	J(1,4) =		 y1*cc*sa + z*cc*ca - a2*cc*ca;

	J(2,0) = 0;
	J(2,1) = sa;
	J(2,2) = ca;
	J(2,3) = 0;
	J(2,4) = y1*ca - z*sa + a2*sa;

	J(3,0) = 0;
	J(3,1) = 0;
	J(3,2) = 0;
	J(3,3) = 1;
	J(3,4) = 0;

	J(4,0) = 0;
	J(4,1) = 0;
	J(4,2) = 0;
	J(4,3) = 0;
	J(4,4) = 1;
}

void control::CalcJdot(vector<double>& q,vector<double>& qdot,matrix<double>& Jdot)
{
	double x = q(0),y = q(1),z = q(2),
		   thc = q(3),tha = q(4),
		   xdot = qdot(0),ydot = qdot(1),zdot = qdot(2),
		   thcdot = qdot(3),thadot = qdot(4),
		   a2 = OFFSET_BETWEEN_COORDINATE;
	double sa = sin(tha),sc = sin(thc),ca = cos(tha),cc = cos(thc);
	/*
	Jdot(0,0) = thcdot*sin(thc);
	Jdot(0,1) = -thcdot*cos(thc)*cos(tha) + thadot*sin(thc)*sin(tha);
	Jdot(0,2) = thadot*cos(tha)*sin(thc) + thcdot*sin(tha)*cos(thc);
	Jdot(0,3) = qdot(0)*sin(thc)+q(0)*thcdot*cos(thc)
				-(qdot(1)*cos(tha)*cos(thc) - q(1)*thadot*sin(tha)*cos(thc) - q(1)*cos(tha)*thcdot*sin(thc))
				+(qdot(2)*sin(tha)*cos(thc) + q(2)*thadot*cos(tha)*cos(thc) - q(2)*sin(tha)*thcdot*sin(thc))
				-a2*(thadot*cos(tha)*cos(thc) - sin(tha)*thcdot*sin(thc));
	Jdot(0,4) = (qdot(1)*sin(thc)*sin(tha) + q(1)*thcdot*cos(thc)*sin(tha) + q(1)*sin(thc)*thadot*cos(tha))
				+(qdot(2)*sin(thc)*cos(tha) + q(2)*thcdot*cos(thc)*cos(tha) - q(2)*sin(thc)*thadot*sin(tha))
				-a2*(thcdot*cos(thc)*cos(tha) - thadot*sin(thc)*sin(tha));

	Jdot(1,0) = thcdot*cos(thc);
	Jdot(1,1) = thcdot*sin(thc)*cos(tha) + cos(thc)*thadot*sin(tha);
	Jdot(1,2) = -thcdot*sin(thc)*sin(tha) + cos(thc)*thadot*cos(tha);
	Jdot(1,3) = xdot*cos(thc) - x*thcdot*sin(thc)
				+(ydot*sin(thc)*cos(tha) + y*thcdot*cos(thc)*cos(tha) - y*sin(thc)*thadot*sin(tha))
				-(zdot*sin(thc)*sin(tha) + z*thcdot*cos(thc)*sin(tha) + z*sin(thc)*thadot*cos(tha))
				+a2*(thcdot*cos(thc)*sin(tha) + sin(thc)*thadot*cos(tha));
	Jdot(1,4) = (ydot*cos(thc)*sin(tha) - y*thcdot*sin(thc)*sin(tha) + y*cos(thc)*thadot*cos(tha))
				+(zdot*cos(thc)*cos(tha) - z*thcdot*sin(thc)*cos(tha) - z*cos(thc)*thadot*sin(tha))
				-a2*(-thcdot*sin(thc)*cos(tha) - cos(thc)*thadot*sin(tha));
	
	Jdot(2,0) = 0;
	Jdot(2,1) = thadot*cos(tha);
	Jdot(2,2) = -thadot*sin(tha);
	Jdot(2,3) = 0;
	Jdot(2,4) = (qdot(1)*cos(tha) - q(1)*thadot*sin(tha))
				-(qdot(2)*sin(tha) + q(2)*thadot*cos(tha))
				+a2*thadot*cos(tha);
	*/
	Jdot(0,0) =  thcdot*sc;
	Jdot(0,1) = -thcdot*cc*ca + thadot*sc*sa;
	Jdot(0,2) =  thadot*ca*sc + thcdot*sa*cc;
	Jdot(0,3) =  qdot(0)*sc+q(0)*thcdot*cc
				-(qdot(1)*ca*cc - q(1)*thadot*sa*cc - q(1)*ca*thcdot*sc)
				+(qdot(2)*sa*cc + q(2)*thadot*ca*cc - q(2)*sa*thcdot*sc)
				-a2*(thadot*ca*cc - sa*thcdot*sc);
	Jdot(0,4) =  (qdot(1)*sc*sa + q(1)*thcdot*cc*sa + q(1)*sc*thadot*ca)
				+(qdot(2)*sc*ca + q(2)*thcdot*cc*ca - q(2)*sc*thadot*sa)
				-a2*(thcdot*cc*ca - thadot*sc*sa);

	Jdot(1,0) =  thcdot*cc;
	Jdot(1,1) =  thcdot*sc*ca + cc*thadot*sa;
	Jdot(1,2) = -thcdot*sc*sa + cc*thadot*ca;
	Jdot(1,3) =  xdot*cc - x*thcdot*sc
				+(ydot*sc*ca + y*thcdot*cc*ca - y*sc*thadot*sa)
				-(zdot*sc*sa + z*thcdot*cc*sa + z*sc*thadot*ca)
				+a2*(thcdot*cc*sa + sc*thadot*ca);
	Jdot(1,4) =  (ydot*cc*sa - y*thcdot*sc*sa + y*cc*thadot*ca)
				+(zdot*cc*ca - z*thcdot*sc*ca - z*cc*thadot*sa)
				-a2*(-thcdot*sc*ca - cc*thadot*sa);

	Jdot(2,0) =  0;
	Jdot(2,1) =  thadot*ca;
	Jdot(2,2) = -thadot*sa;
	Jdot(2,3) =  0;
	Jdot(2,4) =  (qdot(1)*ca - q(1)*thadot*sa)
				-(qdot(2)*sa + q(2)*thadot*ca)
				+a2*thadot*ca;

	Jdot(3,0) = 0;
	Jdot(3,1) = 0;
	Jdot(3,2) = 0;
	Jdot(3,3) = 0;
	Jdot(3,4) = 0;

	Jdot(4,0) = 0;
	Jdot(4,1) = 0;
	Jdot(4,2) = 0;
	Jdot(4,3) = 0;
	Jdot(4,4) = 0;
}

void control::CalcFrenetFrame(vector<double>& wdot,vector<double>& wddot,matrix<double>& F)
{
	vector<double> Pdot(3),Pddot(3);
	double normPdot,normPddot;
	vector<double> t(3),n(3),b(3);
	double sdot,sddot;

	for(unsigned int i=0;i<Pdot.size();i++){
		Pdot(i) = wdot(i);
		Pddot(i) = wddot(i);
	}

	normPdot = norm_2(Pdot);
	normPddot = norm_2(Pddot);

	sdot = normPdot;
	sddot = inner_prod(Pdot,Pddot) / sdot;

	t = Pdot / sdot;
	n = (Pddot*sdot - Pdot*sddot) / (sdot*sdot*sdot);
	n /= norm_2(n);

	//t = Pdot / normPdot;
	//n = Pddot / normPddot;
	//n = n - inner_prod(t,n)*t;
	//n /= norm_2(n);
	//b(0) = t(1)*n(2) - t(2)*n(1);//b=t*n
	//b(1) = t(2)*n(0) - t(0)*n(2);
	//b(2) = t(0)*n(1) - t(1)*n(0);
	b = numeric::ublas::cross_prod(t,n);

	F(0,0) = t(0);	F(0,1) = n(0);	F(0,2) = b(0);	F(0,3) = 0;	F(0,4) = 0;
	F(1,0) = t(1);	F(1,1) = n(1);	F(1,2) = b(1);	F(1,3) = 0;	F(1,4) = 0;
	F(2,0) = t(2);	F(2,1) = n(2);	F(2,2) = b(2);	F(2,3) = 0;	F(2,4) = 0;
	F(3,0) = 0;		F(3,1) = 0;		F(3,2) = 0;		F(3,3) = 1;	F(3,4) = 0;
	F(4,0) = 0;		F(4,1) = 0;		F(4,2) = 0;		F(4,3) = 0;	F(4,4) = 1;
}

void CalcFrenetFramedot(vector<double>& wdot,vector<double>& wddot,vector<double>& w3,matrix<double>& Fdot)
{
	vector<double> Pdot(3),Pddot(3),P3(3);
	double normPdot,normPddot,normP3;
	vector<double> t(3),n(3),b(3),tdot(3),ndot(3),bdot(3);
	double sdot,sddot,s3;

	for(unsigned int i=0;i<Pdot.size();i++){
		Pdot(i) = wdot(i);
		Pddot(i) = wddot(i);
		P3(i) = w3(i);
	}

	normPdot = norm_2(Pdot);
	normPddot = norm_2(Pddot);
	normP3 = norm_2(P3);

	sdot = normPdot;
	sddot = inner_prod(Pdot,Pddot) / sdot;
	s3 = ( (numeric::ublas::norm_2_sq(Pddot)+inner_prod(Pdot,P3))*sdot-inner_prod(Pdot,Pddot)*sddot ) / (sdot*sdot);

	t = Pdot / sdot;
	n = (Pddot*sdot - Pdot*sddot) / (sdot*sdot*sdot);
	n /= norm_2(n);
	b = numeric::ublas::cross_prod(t,n);

	tdot = (Pddot*sdot - Pdot*sddot) / (sdot*sdot);
	vector<double>	temp = Pddot*sdot - Pdot*sddot,
					tempdot = P3*sdot - Pdot*s3;
	double	norm = norm_2(temp),
			normdot = inner_prod(temp,tempdot) / norm_2(temp);
	ndot = (tempdot*norm - temp*normdot) / (norm*norm);
	bdot = numeric::ublas::cross_prod(tdot,n) + numeric::ublas::cross_prod(t,ndot);

	Fdot.clear();
	Fdot(0,0) = tdot(0);	Fdot(0,1) = ndot(0);	Fdot(0,2) = bdot(0);
	Fdot(1,0) = tdot(1);	Fdot(1,1) = ndot(1);	Fdot(1,2) = bdot(1);
	Fdot(2,0) = tdot(2);	Fdot(2,1) = ndot(2);	Fdot(2,2) = bdot(2);
}

void CalcFrenetFrameddot(vector<double>& w1,vector<double>& w2,vector<double>& w3,vector<double>& w4,
	matrix<double>& F,matrix<double>& F1,matrix<double>& F2)
{
	vector<double>	P1d(3),P2d(3),P3d(3),P4d(3);
	double			normP1,normP2,normP3,normP4;
	vector<double>	t(3),n(3),b(3),
					t1(t),n1(n),b1(b),
					t2(t),n2(n),b2(b);
	double			s1,s2,s3,s4;
	double			tau,tau1,tau2;
	vector<double>	n_dash(3),n_dash1(n_dash),n_dash2(n_dash),
					phi(3),phi1(phi),phi2(phi);
	double			nn_dash,nn_dash1,nn_dash2;

	using namespace numeric::ublas;

	for(unsigned int i=0;i<P1d.size();i++){
		P1d(i) = w1(i);
		P2d(i) = w2(i);
		P3d(i) = w3(i);
		P4d(i) = w4(i);
	}

	tau = inner_prod(P1d,P2d);
	tau1 = inner_prod(P2d,P2d) + inner_prod(P1d,P3d);
	tau2 = 3*inner_prod(P3d,P2d) + inner_prod(P1d,P4d);

	s1 = norm_2(P1d);
	s2 =  tau / s1;
	s3 = (tau1*s1 - tau*s2) / (s1*s1);
	s4 = (tau2*s1*s1*s1-tau1*s1*s1*s2-2*tau*tau1*s1+3*tau*tau*s2) / (s1*s1*s1*s1);

	phi = P2d*s1 - P1d*s2;
	phi1 = P3d*s1 - P1d*s3;
	phi2 = P4d*s1 + P3d*s2 - P2d*s3 - P1d*s4;

	n_dash = phi / (s1*s1*s1);
	n_dash1 = (phi1*s1 - phi*3*s2) / (s1*s1*s1*s1);
	n_dash2 = (phi2*s1*s1-6*phi1*s1*s2-3*phi*s1*s3+12*phi*s2*s2) / pow(s1,5);

	nn_dash = norm_2(n_dash);
	nn_dash1 = inner_prod(n_dash,n_dash1) / nn_dash;
	nn_dash2 = (inner_prod(n_dash1,n_dash1)+inner_prod(n_dash,n_dash2)-nn_dash1*nn_dash1) / nn_dash;

	t = P1d / s1;
	n = n_dash / nn_dash;
	b = cross_prod(t,n);

	t1 = phi / (s1*s1);
	n1 = (n_dash1 - n*nn_dash1) / nn_dash;
	b1 = cross_prod(t1,n) + cross_prod(t,n1);
	
	t2 = (phi1*s1*s1 - phi*2*s1*s2) / (s1*s1*s1*s1);
	n2 = ((n_dash2-n1*nn_dash1-n*nn_dash2)*nn_dash-(n_dash1-n*nn_dash1)*nn_dash1) / (nn_dash*nn_dash);
	b2 = cross_prod(t2,n) + 2*cross_prod(t1,n1) + cross_prod(t,n2);

	F.clear();
	F(0,0) = t(0);	F(0,1) = n(0);	F(0,2) = b(0);
	F(1,0) = t(1);	F(1,1) = n(1);	F(1,2) = b(1);
	F(2,0) = t(2);	F(2,1) = n(2);	F(2,2) = b(2);
	F(3,3) = F(4,4) = 1;
	F1.clear();
	F1(0,0) = t1(0);	F1(0,1) = n1(0);	F1(0,2) = b1(0);
	F1(1,0) = t1(1);	F1(1,1) = n1(1);	F1(1,2) = b1(1);
	F1(2,0) = t1(2);	F1(2,1) = n1(2);	F1(2,2) = b1(2);
	F2.clear();
	F2(0,0) = t2(0);	F2(0,1) = n2(0);	F2(0,2) = b2(0);
	F2(1,0) = t2(1);	F2(1,1) = n2(1);	F2(1,2) = b2(1);
	F2(2,0) = t2(2);	F2(2,1) = n2(2);	F2(2,2) = b2(2);
}

void control::CalcOrientation(vector<double>& O,vector<double>& q)
{
	double tha1 = q(4),thc = q(3);
	//O(0) = sin(tha1) * sin(thc);
	//O(1) = sin(tha1) * cos(thc);
	//O(2) = cos(tha1);
	O(0) = cos(tha1) * sin(thc);
	O(1) = cos(tha1) * cos(thc);
	O(2) = -sin(tha1);
}

void control::CalcJo(matrix<double>& Jo,vector<double>& q)
{
	double tha1 = q(4),thc = q(3);
	double sa = sin(tha1),sc = sin(thc),ca = cos(tha1),cc = cos(thc);
	
	//Jo(0,0) =  ca * sc;	Jo(0,1) =  sa * cc;
	//Jo(1,0) =  ca * cc;	Jo(1,1) = -sa * sc;
	//Jo(2,0) = -sa;		Jo(2,1) = 0;
	Jo(0,0) = -sa * sc;	Jo(0,1) =  ca * cc;
	Jo(1,0) = -sa * cc;	Jo(1,1) =  ca * -sc;
	Jo(2,0) = -ca;		Jo(2,1) = 0;
}

void control::CalcJodot(matrix<double>& Jodot,vector<double>& q,vector<double>& qdot)
{
	//x y1 z c a1 
	//0 1  2 3 4 
	double tha1,thc,tha1dot,thcdot;
	tha1 = q(4),thc = q(3);
	tha1dot = qdot(4);	thcdot = qdot(3);
	double sa = sin(tha1),sc = sin(thc),ca = cos(tha1),cc = cos(thc);
	
	Jodot(0,0) = -tha1dot*sa*sc + ca*thcdot*cc;
	Jodot(0,1) =  tha1dot*ca*cc - sa*thcdot*sc;
	Jodot(1,0) = -tha1dot*sa*cc - ca*thcdot*sc;
	Jodot(1,1) = -tha1dot*ca*sc - sa*thcdot*cc;
	Jodot(2,0) = -tha1dot*ca;
	Jodot(2,1) =  0;


	Jodot(0,0) = -tha1dot*ca*sc - sa*thcdot*cc;
	Jodot(0,1) =  tha1dot*-sa*cc + ca*thcdot*-sc;
	Jodot(1,0) = -tha1dot*ca*cc - sa*thcdot*-sc;
	Jodot(1,1) =  tha1dot*-sa*-sc + ca*thcdot*-cc;
	Jodot(2,0) = -tha1dot*-sa;
	Jodot(2,1) =  0;
}

double control::CalcContouringError(vector<double> &pos)
{
	double ec,x,y;

	x = pos(0);
	y = pos(1);
	ec = sqrt( x*x+y*y ) - RADIUS;

	return ec;
}

void control::SetStateVal(dvector& pos,dvector& vel,dvector& acc,dvector& posall,dvector& velall,dvector& accall)
{
	pos(0) = posall(0);	vel(0) = velall(0);	acc(0) = accall(0);
	pos(1) = posall(1);	vel(1) = velall(1);	acc(1) = accall(1);
	pos(2) = posall(3);	vel(2) = velall(3);	acc(2) = accall(3);
	pos(3) = posall(4);	vel(3) = velall(4);	acc(3) = accall(4);
	pos(4) = posall(5);	vel(4) = velall(5);	acc(4) = accall(5);
}

void control::SetReferenceVal(bool enable_sync,dvector& rpos,dvector& rvel,dvector& racc,dvector& rposall,dvector& rvelall,dvector& raccall)
{
	rposall(0) = rpos(0);	rvelall(0) = rvel(0);	raccall(0) = racc(0);
	rposall(1) = rpos(1);	rvelall(1) = rvel(1);	raccall(1) = racc(1);
	rposall(2) = rpos(1);	rvelall(2) = rvel(1);	raccall(2) = racc(1);
	rposall(3) = rpos(2);	rvelall(3) = rvel(2);	raccall(3) = racc(2);
	rposall(4) = rpos(3);	rvelall(4) = rvel(3);	raccall(4) = racc(3);
	rposall(5) = rpos(4);	rvelall(5) = rvel(4);	raccall(5) = racc(4);
	rposall(6) = rpos(4);	rvelall(6) = rvel(4);	raccall(6) = racc(4);
}

void OutputConVal(vector<double>& u_all,vector<double>& u,vector<double>& u_sync,short AioId)
{
	u_all(0) =   u(0);
	u_all(1) =   u(1);
	u_all(2) = -(u(1) + u_sync(0));
	u_all(3) =   u(2);
	u_all(4) = - u(3);
	u_all(5) =   u(4);
	u_all(6) = -(u(4) + u_sync(1));

	// output control value
	iocda::OutputControlVariable(AioId,u_all);
}

void control::IndependentCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	using namespace control::conparam;
	
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

	matrix<double>	m(2,2,0),cc(2,2,0),kvw2(2,2,0),kpw2(2,2,0);
	m(0,0) = mass(0,0);	m(1,1) = mass(1,1);
	cc(0,0) = c(0,0);	cc(1,1) = c(1,1);
	kvw2(0,0)=kvw(0,0);kpw2(0,0)=kpw(0,0);
	kvw2(1,1)=kvw(1,1);kpw2(1,1)=kpw(1,1);

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
		force = prod(m,(racc-prod(kvw2,ew1)-prod(kpw2,ew))) + prod(cc,vel);

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
		forces = masss(0,0)*( -kvs(0,0)*es1-kps(0,0)*es ) + cs(0,0)*es1;

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

void control::IndependentCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> ew(NUM_DOF),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR),
				   pos(NUM_DOF),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(NUM_DOF),rvel(rpos),racc(rpos),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   force(NUM_DOF),forceall(NUM_ACTUATOR),force_log(forceall);
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
	double time,ec;
	double d[10];
		

	using namespace control::conparam;


	time = SAMPLING_TIME * step;

	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);
	//pos(1) = posall(2);
	pos(2) = posall(3);
	pos(3) = posall(4);
	pos(4) = posall(5);
	poss(0) = posall(2);
	//poss(0) = posall(1);
	poss(1) = posall(6);

	vel(0) = velall(0);
	vel(1) = velall(1);
	//vel(1) = velall(2);
	vel(2) = velall(3);
	vel(3) = velall(4);
	vel(4) = velall(5);
	vels(0) = velall(2);
	//vels(0) = velall(1);
	vels(1) = velall(6);

	acc(0) = accall(0);
	acc(1) = accall(1);
	acc(2) = accall(3);
	acc(3) = accall(4);
	acc(4) = accall(5);
	accs(0) = accall(2);
	accs(1) = accall(6);

	// get reference
	GenerateReference(time,rpos,rvel,racc);
	if(enable_sync){
		rposs(0) = pos(1);
		rposs(1) = pos(4);
		rvels(0) = vel(1);
		rvels(1) = vel(4);
		raccs(0) = acc(1);
		raccs(1) = acc(4);
	}else{
		rposs(0) = rpos(1);
		rposs(1) = rpos(4);
		rvels(0) = rvel(1);
		rvels(1) = rvel(4);
		raccs(0) = racc(1);
		raccs(1) = racc(4);
	}

	// calculation tracking error
	ew = pos - rpos;
	ew1 = vel - rvel;
	ew2 = acc - racc;
	// calculation sync error
	es = poss - rposs;
	es1 = vels - rvels;
	es2 = accs - raccs;	

	// calculation control value
	force = prod(mass,(racc-prod(kvw,ew1)-prod(kpw,ew))) + prod(c,vel);

	if(enable_sync)
		// calculation control value(sync)
		forces = prod(masss,(-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
	else{
		kvs(0,0) = 2 * pole_y2;
		kvs(1,1) = 2 * pole_a2;
		kps(0,0) = pole_y2 * pole_y2;
		kps(1,1) = pole_a2 * pole_a2;
		forces = prod(masss,(raccs-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
	}

	forceall(0) = force(0);
	forceall(1) = force(1);
	//forceall(2) = -force(1);
	//forceall(2) = ;
	forceall(3) = -force(2);
	forceall(4) = -force(3);
	forceall(5) = force(4);
	//forceall(6) = ;

	if(enable_sync){
		forceall(2) = -(force(1) + forces(0));
		//forceall(1) = (force(1) + forces(0));
		forceall(6) = -(force(4) + forces(1));
	}else{
		forceall(2) = -forces(0);
		forceall(6) = -forces(1);
	}

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// calculation contouring error
	ec = CalcContouringError(pos);
	
	// loging data
	rpos_log(0) = rpos(0);
	rpos_log(1) = rpos(1);
	rpos_log(2) = rposs(0);
	rpos_log(3) = rpos(2);
	rpos_log(4) = rpos(3);
	rpos_log(5) = rpos(4);
	rpos_log(6) = rposs(1);
	rvel_log(0) = rvel(0);
	rvel_log(1) = rvel(1);
	rvel_log(2) = rvels(0);
	rvel_log(3) = rvel(2);
	rvel_log(4) = rvel(3);
	rvel_log(5) = rvel(4);
	rvel_log(6) = rvels(1);
	racc_log(0) = racc(0);
	racc_log(1) = racc(1);
	racc_log(2) = raccs(0);
	racc_log(3) = racc(2);
	racc_log(4) = racc(3);
	racc_log(5) = racc(4);
	racc_log(6) = raccs(1);
	ew_log(0) = ew(0);
	ew_log(1) = ew(1);
	ew_log(2) = poss(0) - rpos(1);		//es(0);
	ew_log(3) = ew(2);
	ew_log(4) = ew(3);
	ew_log(5) = ew(4);
	ew_log(6) = poss(1) - rpos(4);		//es(1);
	force_log = forceall;
	force_log(2) *= -1;
	force_log(3) *= -1;
	force_log(4) *= -1;
	force_log(6) *= -1;
	datalog[0,step] = time;
	datalog[1,step] = ec;
	datalog[2,step] = 0;				//ec-en
	datalog[3,step] = 0;				//ec-enmod
	datalog[4,step] = 0;				//td
	datalog[5,step] = 0;				//el t
	datalog[6,step] = 0;				//el n
	datalog[7,step] = 0;				//el b
	datalog[8,step] = poss(0) - pos(1);	//es y
	datalog[9,step] = poss(1) - pos(4);	//es a
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

void control::ContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	unsigned int step;
	PLANT plant;
	vector<double> ew(NUM_DOF),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR),
				   el(NUM_DOF),el1(el),el2(el),
				   pos(NUM_DOF),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(NUM_DOF),rvel(rpos),racc(rpos),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   force(NUM_DOF),forceall(NUM_ACTUATOR),force_log(forceall);
	matrix<double> rot(NUM_DOF,NUM_DOF),rot1(rot),rot2(rot),
				   rot_t,rot1_t,rot2_t;
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);

	double time,ec,theta;

	volatile double d[10];

	using namespace control::conparam;

	posall.clear();
	velall.clear();
	accall.clear();

	rot.clear();
	rot1.clear();
	rot2.clear();

	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		// get state value
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
		pos(0) = posall(0);
		pos(1) = posall(1);
		pos(2) = posall(3);
		pos(3) = posall(4);
		pos(4) = posall(5);
		poss(0) = posall(2);
		poss(1) = posall(6);

		vel(0) = velall(0);
		vel(1) = velall(1);
		vel(2) = velall(3);
		vel(3) = velall(4);
		vel(4) = velall(5);
		vels(0) = velall(2);
		vels(1) = velall(6);

		acc(0) = accall(0);
		acc(1) = accall(1);
		acc(2) = accall(3);
		acc(3) = accall(4);
		acc(4) = accall(5);
		accs(0) = accall(2);
		accs(1) = accall(6);

		// get reference
		GenerateReference(time,rpos,rvel,racc);
		if(enable_sync){
			rposs(0) = pos(1);
			rposs(1) = pos(4);
			rvels(0) = vel(1);
			rvels(1) = vel(4);
			raccs(0) = acc(1);
			raccs(1) = acc(4);
		}else{
			rposs(0) = rpos(1);
			rposs(1) = rpos(4);
			rvels(0) = rvel(1);
			rvels(1) = rvel(4);
			raccs(0) = racc(1);
			raccs(1) = racc(4);
		}

		// calculation tracking error
		ew = pos - rpos;
		ew1 = vel - rvel;
		ew2 = acc - racc;
		// calculation sync error
		es = poss - rposs;
		es1 = vels - rvels;
		es2 = accs - raccs;

		// transform error
		theta = OMG * time;
		rot(0,0) =  cos(theta);
		rot(0,1) = -sin(theta);
		rot(1,0) =  sin(theta);
		rot(1,1) =  cos(theta);
		rot1(0,0) = -OMG * sin(theta);
		rot1(0,1) = -OMG * cos(theta);
		rot1(1,0) =  OMG * cos(theta);
		rot1(1,1) = -OMG * sin(theta);
		rot2(0,0) = -OMG * OMG * cos(theta);
		rot2(0,1) =  OMG * OMG * sin(theta);
		rot2(1,0) = -OMG * OMG * sin(theta);
		rot2(1,1) = -OMG * OMG * cos(theta);
		rot_t = trans(rot);
		rot1_t = trans(rot1);
		rot2_t = trans(rot2);
		el = prod(rot_t,ew);
		el1 = prod(rot1_t,ew) + prod(rot_t,ew1);
		el2 = prod(rot2_t,ew) + 2*prod(rot1_t,ew1) + prod(rot_t,ew2);

		// calculation control value
		force = prod(mass,racc - prod(rot,prod(kvl,el1) + prod(kpl,el) + prod(rot2_t,ew) + 2*prod(rot1_t,ew1))) + prod(c,vel);

		if(enable_sync)
		// calculation control value(sync)
			forces = prod(masss,(-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
		else{
			vector<double> els(2),els1(els);
			matrix<double> rots(2,2),
						   rots_t(2,2),rots1_t(rots_t),rots2_t(rots_t);
			rots(0,0) = rot(0,0);
			rots(0,1) = rot(0,1);
			rots_t(0,0) = rot_t(0,0);
			rots_t(0,1) = rot_t(0,1);
			rots1_t(0,0) = rot1_t(0,0);
			rots1_t(0,1) = rot1_t(0,1);
			rots2_t(0,0) = rot2_t(0,0);
			rots2_t(0,1) = rot2_t(0,1);
			els = prod(rots_t,es);
			els1 = prod(rots1_t,es) + prod(rots_t,es1);
			kvs(0,0) = 2 * pole_y2;
			kvs(1,1) = 2 * pole_a2;
			kps(0,0) = pole_y2 * pole_y2;
			kps(1,1) = pole_a2 * pole_a2;
			forces = prod(masss,raccs - prod(rots,prod(kvs,els1) + prod(kps,els) + prod(rots2_t,es) + 2*prod(rots1_t,es1))) + prod(cs,es);
		}

		forceall(0) = force(0);
		forceall(1) = force(1);
		//forceall(2) = ;
		forceall(3) = force(2);
		forceall(4) = force(3);
		forceall(5) = force(4);
		//forceall(6) = ;

		if(enable_sync){
			forceall(2) = force(1) + forces(0);
			forceall(6) = force(4) + forces(1);
		}else{
			forceall(2) = forces(0);
			forceall(6) = forces(1);
		}

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);
		
		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// calculation contouring error
		//ec = sqrt( pos(0)*pos(0) + pos(1)*pos(1) ) - RADIUS;
		ec = CalcContouringError(pos);

		// loging data
		rpos_log(0) = rpos(0);
		rpos_log(1) = rpos(1);
		rpos_log(2) = rposs(0);
		rpos_log(3) = rpos(2);
		rpos_log(4) = rpos(3);
		rpos_log(5) = rpos(4);
		rpos_log(6) = rposs(1);
		rvel_log(0) = rvel(0);
		rvel_log(1) = rvel(1);
		rvel_log(2) = rvels(0);
		rvel_log(3) = rvel(2);
		rvel_log(4) = rvel(3);
		rvel_log(5) = rvel(4);
		rvel_log(6) = rvels(1);
		racc_log(0) = racc(0);
		racc_log(1) = racc(1);
		racc_log(2) = raccs(0);
		racc_log(3) = racc(2);
		racc_log(4) = racc(3);
		racc_log(5) = racc(4);
		racc_log(6) = raccs(1);
		ew_log(0) = ew(0);
		ew_log(1) = ew(1);
		ew_log(2) = poss(0) - rpos(1);		//es(0);
		ew_log(3) = ew(2);
		ew_log(4) = ew(3);
		ew_log(5) = ew(4);
		ew_log(6) = poss(1) - rpos(4);		//es(1);
		force_log = forceall;
		datalog[0,step] = time;
		datalog[1,step] = ec;
		datalog[2,step] = -el(1) - ec;		//ec-en
		datalog[3,step] = 0;				//ec-enmod
		datalog[4,step] = 0;				//td
		datalog[5,step] = el(0);			//el t
		datalog[6,step] = el(1);			//el n
		datalog[7,step] = el(2);			//el b
		datalog[8,step] = poss(0) - pos(1);	//es y
		datalog[9,step] = poss(1) - pos(4);	//es a
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
}

void control::ContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> ew(NUM_DOF),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR),
				   el(NUM_DOF),el1(el),el2(el),
				   pos(NUM_DOF),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(NUM_DOF),rvel(rpos),racc(rpos),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   force(NUM_DOF),forceall(NUM_ACTUATOR),force_log(forceall);
	matrix<double> rot(NUM_DOF,NUM_DOF),rot1(rot),rot2(rot),
				   rot_t,rot1_t,rot2_t;
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
	double time,ec,theta;

	using namespace control::conparam;

	rot.clear();
	rot1.clear();
	rot2.clear();

	time = step * SAMPLING_TIME;
		
	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);
	pos(2) = posall(3);
	pos(3) = posall(4);
	pos(4) = posall(5);
	poss(0) = posall(2);
	poss(1) = posall(6);

	vel(0) = velall(0);
	vel(1) = velall(1);
	vel(2) = velall(3);
	vel(3) = velall(4);
	vel(4) = velall(5);
	vels(0) = velall(2);
	vels(1) = velall(6);

	acc(0) = accall(0);
	acc(1) = accall(1);
	acc(2) = accall(3);
	acc(3) = accall(4);
	acc(4) = accall(5);
	accs(0) = accall(2);
	accs(1) = accall(6);

	// get reference
	GenerateReference(time,rpos,rvel,racc);
	if(enable_sync){
		rposs(0) = pos(1);
		rposs(1) = pos(4);
		rvels(0) = vel(1);
		rvels(1) = vel(4);
		raccs(0) = acc(1);
		raccs(1) = acc(4);
	}else{
		rposs(0) = rpos(1);
		rposs(1) = rpos(4);
		rvels(0) = rvel(1);
		rvels(1) = rvel(4);
		raccs(0) = racc(1);
		raccs(1) = racc(4);
	}

	// calculation tracking error
	ew = pos - rpos;
	ew1 = vel - rvel;
	ew2 = acc - racc;
	// calculation sync error
	es = poss - rposs;
	es1 = vels - rvels;
	es2 = accs - raccs;

	/////////5-DOF

	/////////

	// ogawa/////////////////////////////////////////
	///*
	// transform error
	theta = OMG * time;
	rot(0,0) =  cos(theta);
	rot(0,1) = -sin(theta);
	rot(1,0) =  sin(theta);
	rot(1,1) =  cos(theta);
	rot1(0,0) = -OMG * sin(theta);
	rot1(0,1) = -OMG * cos(theta);
	rot1(1,0) =  OMG * cos(theta);
	rot1(1,1) = -OMG * sin(theta);
	rot2(0,0) = -OMG * OMG * cos(theta);
	rot2(0,1) =  OMG * OMG * sin(theta);
	rot2(1,0) = -OMG * OMG * sin(theta);
	rot2(1,1) = -OMG * OMG * cos(theta);
	rot_t = trans(rot);
	rot1_t = trans(rot1);
	rot2_t = trans(rot2);
	el = prod(rot_t,ew);
	el1 = prod(rot1_t,ew) + prod(rot_t,ew1);
	el2 = prod(rot2_t,ew) + 2*prod(rot1_t,ew1) + prod(rot_t,ew2);

	// calculation control value
	force = prod(mass,racc - prod(rot,prod(kvl,el1) + prod(kpl,el) + prod(rot2_t,ew) + 2*prod(rot1_t,ew1))) + prod(c,vel);
	//*/
	// /ogawa //////////////////////////////////////////////////////////

	if(enable_sync)
	// calculation control value(sync)
		forces = prod(masss,(-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
	else{
		vector<double> els(2),els1(els);
		matrix<double> rots(2,2),
					   rots_t(2,2),rots1_t(rots_t),rots2_t(rots_t);
		rots(0,0) = rot(0,0);
		rots(0,1) = rot(0,1);
		rots_t(0,0) = rot_t(0,0);
		rots_t(0,1) = rot_t(0,1);
		rots1_t(0,0) = rot1_t(0,0);
		rots1_t(0,1) = rot1_t(0,1);
		rots2_t(0,0) = rot2_t(0,0);
		rots2_t(0,1) = rot2_t(0,1);
		els = prod(rots_t,es);
		els1 = prod(rots1_t,es) + prod(rots_t,es1);
		kvs(0,0) = 2 * pole_y2;
		kvs(1,1) = 2 * pole_a2;
		kps(0,0) = pole_y2 * pole_y2;
		kps(1,1) = pole_a2 * pole_a2;
		forces = prod(masss,raccs - prod(rots,prod(kvs,els1) + prod(kps,els) + prod(rots2_t,es) + 2*prod(rots1_t,es1))) + prod(cs,es);
	}

	forceall(0) = force(0);
	forceall(1) = force(1);
	//forceall(2) = ;
	forceall(3) = -force(2);
	forceall(4) = -force(3);
	forceall(5) = force(4);
	//forceall(6) = ;

	if(enable_sync){
		forceall(2) = -(force(1) + forces(0));
		forceall(6) = -(force(4) + forces(1));
	}else{
		forceall(2) = -forces(0);
		forceall(6) = -forces(1);
	}
	
	// output control value
	iocda::OutputControlVariable(AioId,forceall);
	
	// calculation contouring error
	//ec = sqrt( pos(0)*pos(0) + pos(1)*pos(1) ) - RADIUS;
	ec = CalcContouringError(pos);

	// loging data
	rpos_log(0) = rpos(0);
	rpos_log(1) = rpos(1);
	rpos_log(2) = rposs(0);
	rpos_log(3) = rpos(2);
	rpos_log(4) = rpos(3);
	rpos_log(5) = rpos(4);
	rpos_log(6) = rposs(1);
	rvel_log(0) = rvel(0);
	rvel_log(1) = rvel(1);
	rvel_log(2) = rvels(0);
	rvel_log(3) = rvel(2);
	rvel_log(4) = rvel(3);
	rvel_log(5) = rvel(4);
	rvel_log(6) = rvels(1);
	racc_log(0) = racc(0);
	racc_log(1) = racc(1);
	racc_log(2) = raccs(0);
	racc_log(3) = racc(2);
	racc_log(4) = racc(3);
	racc_log(5) = racc(4);
	racc_log(6) = raccs(1);
	ew_log(0) = ew(0);
	ew_log(1) = ew(1);
	ew_log(2) = poss(0) - rpos(1);		//es(0);
	ew_log(3) = ew(2);
	ew_log(4) = ew(3);
	ew_log(5) = ew(4);
	ew_log(6) = poss(1) - rpos(4);		//es(1);
	force_log = forceall;
	force_log(2) *= -1;
	force_log(3) *= -1;
	force_log(4) *= -1;
	force_log(6) *= -1;
	datalog[0,step] = time;
	datalog[1,step] = ec;
	datalog[2,step] = -el(1) - ec;		//ec-en
	datalog[3,step] = 0;				//ec-enmod
	datalog[4,step] = 0;				//td
	datalog[5,step] = el(0);			//el t
	datalog[6,step] = el(1);			//el n
	datalog[7,step] = el(2);			//el b
	datalog[8,step] = poss(0) - pos(1);	//es y
	datalog[9,step] = poss(1) - pos(4);	//es a
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

void control::modContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	unsigned int step;
	PLANT plant;
	vector<double> ew(NUM_DOF),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR),
				   el(NUM_DOF),el1(el),el2(el),
				   ewn(NUM_DOF),ewn1(ewn),ewn2(ewn),
				   ela(NUM_DOF),ela1(ela),ela2(ela),
				   pos(NUM_DOF),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(NUM_DOF),rvel(rpos),racc(rpos),
				   rposa(NUM_DOF),rvela(rposa),racca(rposa),
				   rposn(rposa),rveln(rposa),raccn(rposa),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   force(NUM_DOF),forceall(NUM_ACTUATOR),force_log(forceall);
	matrix<double> rot(NUM_DOF,NUM_DOF),rot1(rot),rot2(rot),
				   rot_t,rot1_t,rot2_t,
				   rota(NUM_DOF,NUM_DOF),rota1(rota),rota2(rota),
				   rota_t,rota1_t,rota2_t,
				   q(NUM_DOF,NUM_DOF),q1(q),q2(q),s(q);
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
	matrix<double> masss(2,2),cs(masss),kvs(masss),kps(masss);
	double time,ec,theta,thetaa,thetaa1,thetaa2,td,td1,td2,oldtd,oldtd1,dv;


	using namespace control::conparam;

	posall.clear();
	velall.clear();
	accall.clear();

	rot.clear();
	rot1.clear();
	rot2.clear();
	rota.clear();
	rota1.clear();
	rota2.clear();
	q.clear();
	q1.clear();
	q2.clear();
	s.clear();
	s(1,1) = 1;

	td = 0;
	td1 = 0;
	td2 = 0;
	oldtd = 0;
	oldtd1 = 0;

	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		// get state value
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
		pos(0) = posall(0);
		pos(1) = posall(1);
		pos(2) = posall(3);
		pos(3) = posall(4);
		pos(4) = posall(5);
		poss(0) = posall(2);
		poss(1) = posall(6);

		vel(0) = velall(0);
		vel(1) = velall(1);
		vel(2) = velall(3);
		vel(3) = velall(4);
		vel(4) = velall(5);
		vels(0) = velall(2);
		vels(1) = velall(6);

		acc(0) = accall(0);
		acc(1) = accall(1);
		acc(2) = accall(3);
		acc(3) = accall(4);
		acc(4) = accall(5);
		accs(0) = accall(2);
		accs(1) = accall(6);

		// get reference
		GenerateReference(time,rpos,rvel,racc);
		if(enable_sync){
			rposs(0) = pos(1);
			rposs(1) = pos(4);
			rvels(0) = vel(1);
			rvels(1) = vel(4);
			raccs(0) = acc(1);
			raccs(1) = acc(4);
		}else{
			rposs(0) = rpos(1);
			rposs(1) = rpos(4);
			rvels(0) = rvel(1);
			rvels(1) = rvel(4);
			raccs(0) = racc(1);
			raccs(1) = racc(4);
		}

		// calculation tracking error
		ew = pos - rpos;
		ew1 = vel - rvel;
		ew2 = acc - racc;
		// calculation sync error
		es = poss - rposs;
		es1 = vels - rvels;
		es2 = accs - raccs;

		theta = OMG * time;
		rot(0,0) =  cos(theta);
		rot(0,1) = -sin(theta);
		rot(1,0) =  sin(theta);
		rot(1,1) =  cos(theta);
		rot1(0,0) = -OMG * sin(theta);
		rot1(0,1) = -OMG * cos(theta);
		rot1(1,0) =  OMG * cos(theta);
		rot1(1,1) = -OMG * sin(theta);
		rot2(0,0) = -OMG * OMG * cos(theta);
		rot2(0,1) =  OMG * OMG * sin(theta);
		rot2(1,0) = -OMG * OMG * sin(theta);
		rot2(1,1) = -OMG * OMG * cos(theta);
		rot_t = trans(rot);
		rot1_t = trans(rot1);
		rot2_t = trans(rot2);
		el = prod(rot_t,ew);
		el1 = prod(rot1_t,ew) + prod(rot_t,ew1);
		el2 = prod(rot2_t,ew) + 2*prod(rot1_t,ew1) + prod(rot_t,ew2);

		// calculation delay time
		dv = OMG * RADIUS;
		td = -el(0) / dv;
		td1 = (td - oldtd) / SAMPLING_TIME;
		td2 = (td1 - oldtd1) / SAMPLING_TIME;
		oldtd = td;
		oldtd1 = td1;

		// calculation r^a
		GenerateReference(time-td,rposa,rvela,racca);
		// calculation rn
		thetaa = OMG * (time - td);
		thetaa1 = OMG * (1 - td1);
		thetaa2 = -OMG * td2;
		rota(0,0) =  cos(thetaa);
		rota(0,1) = -sin(thetaa);
		rota(1,0) =  sin(thetaa);
		rota(1,1) =  cos(thetaa);
		rota1(0,0) = -thetaa1 * sin(thetaa);
		rota1(0,1) = -thetaa1 * cos(thetaa);
		rota1(1,0) =  thetaa1 * cos(thetaa);
		rota1(1,1) = -thetaa1 * sin(thetaa);
		rota2(0,0) = -thetaa2 * sin(thetaa) - thetaa1 * thetaa1 * cos(thetaa);
		rota2(0,1) = -thetaa2 * cos(thetaa) + thetaa1 * thetaa1 * sin(thetaa);
		rota2(1,0) =  thetaa2 * cos(thetaa) - thetaa1 * thetaa1 * sin(thetaa);
		rota2(1,1) = -thetaa2 * sin(thetaa) - thetaa1 * thetaa1 * cos(thetaa);
		rota_t = trans(rota);
		rota1_t = trans(rota1);
		rota2_t = trans(rota2);

		q = prod((matrix<double>)prod(rota,s),rota_t);

		q1 = prod(static_cast<matrix<double>>(prod(rota1,s)),rota_t)
			 + prod(static_cast<matrix<double>>(prod(rota,s)),rota1_t);

		q2 = prod(static_cast<matrix<double>>(prod(rota2,s)),rota_t)
			 + 2 * prod(static_cast<matrix<double>>(prod(rota1,s)),rota1_t)
			 + prod(static_cast<matrix<double>>(prod(rota,s)),rota2_t);

		rposn = rpos + prod(q,(rposa - rpos));
		rveln = rvel + prod(q1,(rposa - rpos)) + prod(q,(rvela - rvel));
		raccn = racc + prod(q2,(rposa - rpos)) + 2*prod(q1,(rvela - rvel)) + prod(q,(racca - racc));

		ewn = pos - rposn;
		ewn1 = vel - rveln;
		ewn2 = acc - raccn;
		
		ela = prod(rota_t,ewn);
		ela1 = prod(rota1_t,ewn) + prod(rota_t,ewn1);
		ela2 = prod(rota2_t,ewn) + 2*prod(rota1_t,ewn1) + prod(rota_t,ewn2);

		// calculation control value
		force = prod(mass,raccn - prod(rota,prod(kvl,ela1) + prod(kpl,ela) + prod(rota2_t,ewn) + 2*prod(rota1_t,ewn1))) + prod(c,vel);
		
		if(enable_sync)
		// calculation control value(sync)
			forces = prod(masss,(-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
		else{
			kvs(0,0) = 2 * pole_y2;
			kvs(1,1) = 2 * pole_a2;
			kps(0,0) = pole_y2 * pole_y2;
			kps(1,1) = pole_a2 * pole_a2;
			forces = prod(masss,(raccs-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
		}

		forceall(0) = force(0);
		forceall(1) = force(1);
		//forceall(2) = ;
		forceall(3) = force(2);
		forceall(4) = force(3);
		forceall(5) = force(4);
		//forceall(6) = ;

		if(enable_sync){
			forceall(2) = force(1) + forces(0);
			forceall(6) = force(4) + forces(1);
		}else{
			forceall(2) = forces(0);
			forceall(6) = forces(1);
		}

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);
		
		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// calculation contouring error
		//ec = sqrt( pos(0)*pos(0) + pos(1)*pos(1) ) - RADIUS;
		ec = CalcContouringError(pos);

		// loging data
		rpos_log(0) = rpos(0);
		rpos_log(1) = rpos(1);
		rpos_log(2) = rposs(0);
		rpos_log(3) = rpos(2);
		rpos_log(4) = rpos(3);
		rpos_log(5) = rpos(4);
		rpos_log(6) = rposs(1);
		rvel_log(0) = rvel(0);
		rvel_log(1) = rvel(1);
		rvel_log(2) = rvels(0);
		rvel_log(3) = rvel(2);
		rvel_log(4) = rvel(3);
		rvel_log(5) = rvel(4);
		rvel_log(6) = rvels(1);
		racc_log(0) = racc(0);
		racc_log(1) = racc(1);
		racc_log(2) = raccs(0);
		racc_log(3) = racc(2);
		racc_log(4) = racc(3);
		racc_log(5) = racc(4);
		racc_log(6) = raccs(1);
		ew_log(0) = ew(0);
		ew_log(1) = ew(1);
		ew_log(2) = poss(0) - rpos(1);		//es(0);
		ew_log(3) = ew(2);
		ew_log(4) = ew(3);
		ew_log(5) = ew(4);
		ew_log(6) = poss(1) - rpos(4);		//es(1);
		force_log = forceall;
		datalog[0,step] = time;
		datalog[1,step] = ec;
		datalog[2,step] = -el(1) - ec;		//ec-en
		datalog[3,step] = -ela(1) - ec;		//ec-enmod
		datalog[4,step] = td;				//td
		datalog[5,step] = el(0);			//el t
		datalog[6,step] = el(1);			//el n
		datalog[7,step] = el(2);			//el b
		datalog[8,step] = poss(0) - pos(1);	//es y
		datalog[9,step] = poss(1) - pos(4);	//es a
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
}

void control::modContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> ew(NUM_DOF),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR),
				   el(NUM_DOF),el1(el),el2(el),
				   ewn(NUM_DOF),ewn1(ewn),ewn2(ewn),
				   ela(NUM_DOF),ela1(ela),ela2(ela),
				   pos(NUM_DOF),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(NUM_DOF),rvel(rpos),racc(rpos),
				   rposa(NUM_DOF),rvela(rposa),racca(rposa),
				   rposn(rposa),rveln(rposa),raccn(rposa),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   force(NUM_DOF),forceall(NUM_ACTUATOR),force_log(forceall);
	matrix<double> rot(NUM_DOF,NUM_DOF),rot1(rot),rot2(rot),
				   rot_t,rot1_t,rot2_t,
				   rota(NUM_DOF,NUM_DOF),rota1(rota),rota2(rota),
				   rota_t,rota1_t,rota2_t,
				   q(NUM_DOF,NUM_DOF),q1(q),q2(q),s(q);
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
	matrix<double> masss(2,2),cs(masss),kvs(masss),kps(masss);
	double time,ec,theta,thetaa,thetaa1,thetaa2;
	static double td,td1,td2,oldtd,oldtd1,dv;

	double d[10];

	using namespace control::conparam;


	rot.clear();
	rot1.clear();
	rot2.clear();
	rota.clear();
	rota1.clear();
	rota2.clear();
	q.clear();
	q1.clear();
	q2.clear();
	s.clear();
	s(1,1) = 1;

	time = step * SAMPLING_TIME;
		
	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);
	pos(2) = posall(3);
	pos(3) = posall(4);
	pos(4) = posall(5);
	poss(0) = posall(2);
	poss(1) = posall(6);

	vel(0) = velall(0);
	vel(1) = velall(1);
	vel(2) = velall(3);
	vel(3) = velall(4);
	vel(4) = velall(5);
	vels(0) = velall(2);
	vels(1) = velall(6);

	acc(0) = accall(0);
	acc(1) = accall(1);
	acc(2) = accall(3);
	acc(3) = accall(4);
	acc(4) = accall(5);
	accs(0) = accall(2);
	accs(1) = accall(6);

	// get reference
	GenerateReference(time,rpos,rvel,racc);
	if(enable_sync){
		rposs(0) = pos(1);
		rposs(1) = pos(4);
		rvels(0) = vel(1);
		rvels(1) = vel(4);
		raccs(0) = acc(1);
		raccs(1) = acc(4);
	}else{
		rposs(0) = rpos(1);
		rposs(1) = rpos(4);
		rvels(0) = rvel(1);
		rvels(1) = rvel(4);
		raccs(0) = racc(1);
		raccs(1) = racc(4);
	}

	// calculation tracking error
	ew = pos - rpos;
	ew1 = vel - rvel;
	ew2 = acc - racc;
	// calculation sync error
	es = poss - rposs;
	es1 = vels - rvels;
	es2 = accs - raccs;

	// transform error
	theta = OMG * time;
	rot(0,0) =  cos(theta);
	rot(0,1) = -sin(theta);
	rot(1,0) =  sin(theta);
	rot(1,1) =  cos(theta);
	rot1(0,0) = -OMG * sin(theta);
	rot1(0,1) = -OMG * cos(theta);
	rot1(1,0) =  OMG * cos(theta);
	rot1(1,1) = -OMG * sin(theta);
	rot2(0,0) = -OMG * OMG * cos(theta);
	rot2(0,1) =  OMG * OMG * sin(theta);
	rot2(1,0) = -OMG * OMG * sin(theta);
	rot2(1,1) = -OMG * OMG * cos(theta);
	rot_t = trans(rot);
	rot1_t = trans(rot1);
	rot2_t = trans(rot2);
	el = prod(rot_t,ew);
	el1 = prod(rot1_t,ew) + prod(rot_t,ew1);
	el2 = prod(rot2_t,ew) + 2*prod(rot1_t,ew1) + prod(rot_t,ew2);

	// calculation delay time
	if(time==0)
		oldtd = oldtd1 = 0;
	dv = OMG * RADIUS;
	td = -el(0) / dv;
	td1 = (td - oldtd) / SAMPLING_TIME;
	td2 = (td1 - oldtd1) / SAMPLING_TIME;
	if(time==0){
		td1=td2=0;
	}else if(time==SAMPLING_TIME)
		td2=0;
	
	double tdf,tdf1,tdf2,cut=200;
	static double oldtdf,oldtdf1,oldtdf2;
	double a = SAMPLING_TIME*cut / (cut*SAMPLING_TIME+2),
		   b = 2*cut / (cut*SAMPLING_TIME+2),
		   ccc = (cut*SAMPLING_TIME-2) / (cut*SAMPLING_TIME+2);

	tdf = a*(td+oldtd) - ccc*oldtdf;
	tdf1 = b*(td-oldtd) - ccc*oldtdf1;
	tdf2 = 2/SAMPLING_TIME*(tdf1-oldtdf1) - oldtdf2;
	oldtdf = tdf;
	oldtdf1 = tdf1;

	oldtd = td;
	oldtd1 = td1;
	

	// calculation r^a
	GenerateReference(time-td,rposa,rvela,racca);
	// calculation rn
	thetaa = OMG * (time - td);
	thetaa1 = OMG * (1 - td1);
	thetaa2 = -OMG * td2;
	//thetaa = OMG * (time - tdf);
	//thetaa1 = OMG * (1 - tdf1);
	//thetaa2 = -OMG * tdf2;
	rota(0,0) =  cos(thetaa);
	rota(0,1) = -sin(thetaa);
	rota(1,0) =  sin(thetaa);
	rota(1,1) =  cos(thetaa);
	rota1(0,0) = -thetaa1 * sin(thetaa);
	rota1(0,1) = -thetaa1 * cos(thetaa);
	rota1(1,0) =  thetaa1 * cos(thetaa);
	rota1(1,1) = -thetaa1 * sin(thetaa);
	rota2(0,0) = -thetaa2 * sin(thetaa) - thetaa1 * thetaa1 * cos(thetaa);
	rota2(0,1) = -thetaa2 * cos(thetaa) + thetaa1 * thetaa1 * sin(thetaa);
	rota2(1,0) =  thetaa2 * cos(thetaa) - thetaa1 * thetaa1 * sin(thetaa);
	rota2(1,1) = -thetaa2 * sin(thetaa) - thetaa1 * thetaa1 * cos(thetaa);
	rota_t = trans(rota);
	rota1_t = trans(rota1);
	rota2_t = trans(rota2);

	q = prod((matrix<double>)prod(rota,s),rota_t);

	q1 = prod((matrix<double>)prod(rota1,s),rota_t) + prod((matrix<double>)prod(rota,s),rota1_t);

	q2 = prod((matrix<double>)prod(rota2,s),rota_t)
		+ 2 * prod((matrix<double>)prod(rota1,s),rota1_t)
		+ prod((matrix<double>)prod(rota,s),rota2_t);

	rposn = rpos + prod(q,(rposa - rpos));
	rveln = rvel + prod(q1,(rposa - rpos)) + prod(q,(rvela - rvel));
	raccn = racc + prod(q2,(rposa - rpos)) + 2*prod(q1,(rvela - rvel)) + prod(q,(racca - racc));

	ewn = pos - rposn;
	ewn1 = vel - rveln;
	ewn2 = acc - raccn;
		
	ela = prod(rota_t,ewn);
	ela1 = prod(rota1_t,ewn) + prod(rota_t,ewn1);
	ela2 = prod(rota2_t,ewn) + 2*prod(rota1_t,ewn1) + prod(rota_t,ewn2);

	// calculation control value
	force = prod(mass,raccn - prod(rota,prod(kvl,ela1) + prod(kpl,ela) + prod(rota2_t,ewn) + 2*prod(rota1_t,ewn1))) + prod(c,vel);
	
	if(enable_sync)
	// calculation control value(sync)
		forces = prod(masss,(-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
	else{
		/*
		vector<double> els(2),els1(els);
		matrix<double> rots(2,2),
					   rots_t(2,2),rots1_t(rots_t),rots2_t(rots_t);
		rots(0,0) = rot(0,0);
		rots(0,1) = rot(0,1);
		rots_t(0,0) = rot_t(0,0);
		rots_t(0,1) = rot_t(0,1);
		rots1_t(0,0) = rot1_t(0,0);
		rots1_t(0,1) = rot1_t(0,1);
		rots2_t(0,0) = rot2_t(0,0);
		rots2_t(0,1) = rot2_t(0,1);
		els = prod(rots_t,es);
		els1 = prod(rots1_t,es) + prod(rots_t,es1);
		kvs(0,0) = 2 * pole_y2;
		kvs(1,1) = 2 * pole_a2;
		kps(0,0) = pole_y2 * pole_y2;
		kps(1,1) = pole_a2 * pole_a2;
		forces = prod(masss,raccs - prod(rots,prod(kvs,els1) + prod(kps,els) + prod(rots2_t,es) + 2*prod(rots1_t,es1))) + prod(cs,es);
		*/

		kvs(0,0) = 2 * pole_y2;
		kvs(1,1) = 2 * pole_a2;
		kps(0,0) = pole_y2 * pole_y2;
		kps(1,1) = pole_a2 * pole_a2;
		forces = prod(masss,(raccs-prod(kvs,es1)-prod(kps,es))) + prod(cs,es1);
	}

	forceall(0) = force(0);
	forceall(1) = force(1);
	//forceall(2) = ;
	forceall(3) = -force(2);
	forceall(4) = -force(3);
	forceall(5) = force(4);
	//forceall(6) = ;

	if(enable_sync){
		forceall(2) = -(force(1) + forces(0));
		forceall(6) = -(force(4) + forces(1));
	}else{
		forceall(2) = -forces(0);
		forceall(6) = -forces(1);
	}

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	// calculation contouring error
	//ec = sqrt( pos(0)*pos(0) + pos(1)*pos(1) ) - RADIUS;
	ec = CalcContouringError(pos);

	// loging data
	rpos_log(0) = rpos(0);
	rpos_log(1) = rpos(1);
	rpos_log(2) = rposs(0);
	rpos_log(3) = rpos(2);
	rpos_log(4) = rpos(3);
	rpos_log(5) = rpos(4);
	rpos_log(6) = rposs(1);
	rvel_log(0) = rvel(0);
	rvel_log(1) = rvel(1);
	rvel_log(2) = rvels(0);
	rvel_log(3) = rvel(2);
	rvel_log(4) = rvel(3);
	rvel_log(5) = rvel(4);
	rvel_log(6) = rvels(1);
	racc_log(0) = racc(0);
	racc_log(1) = racc(1);
	racc_log(2) = raccs(0);
	racc_log(3) = racc(2);
	racc_log(4) = racc(3);
	racc_log(5) = racc(4);
	racc_log(6) = raccs(1);
	ew_log(0) = ew(0);
	ew_log(1) = ew(1);
	ew_log(2) = poss(0) - rpos(1);//es(0);
	ew_log(3) = ew(2);
	ew_log(4) = ew(3);
	ew_log(5) = ew(4);
	ew_log(6) = poss(1) - rpos(4);//es(1);
	force_log = forceall;
	force_log(2) *= -1;
	force_log(3) *= -1;
	force_log(4) *= -1;
	force_log(6) *= -1;
	datalog[0,step] = time;
	datalog[1,step] = ec;
	datalog[2,step] = -el(1) - ec;	//ec-en
	datalog[3,step] = -ela(1) - ec;	//ec-enmod
	datalog[4,step] = td;	//td
	datalog[5,step] = ela(0);		//el t
	datalog[6,step] = ela(1);		//el n
	datalog[7,step] = ela(2);		//el b
	datalog[8,step] = poss(0) - pos(1);	//es y
	datalog[9,step] = poss(1) - pos(4);	//es a
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

double ori(double time,vector<double>& q,vector<double>& qdot,vector<double>& qddot,vector<double>& qref,vector<double>& qrefdot,vector<double>& qrefddot,vector<double>& ur,double& err)//vector<double>& O)
{
	using namespace control;
	using namespace control::conparam;
	volatile double d[3];


	volatile double	tha1,thc,tha1ref,thcref,
			tha1dot,thcdot,tha1refdot,thcrefdot,
			tha1ddot,thcddot,tha1refddot,thcrefddot;
	tha1		 = q(4),		thc			 = q(3);
	tha1dot		 = qdot(4),		thcdot		 = qdot(3);
	tha1ref		 = qref(4),		thcref		 = qref(3);
	tha1refdot	 = qrefdot(4),	thcrefdot	 = qrefdot(3);
	tha1refddot	 = qrefddot(4),	thcrefddot	 = qrefddot(3);

	// orientation
	matrix<double> Jo(3,2),Jodot(Jo),Joref(Jo),Jorefdot(Jo),tJo,invtJoJo;
	CalcJo(Jo,q);
	CalcJodot(Jodot,q,qdot);
	tJo = trans(Jo);
	math::invert(prod(tJo,Jo),invtJoJo);
	CalcJo(Joref,qref);
	CalcJodot(Jorefdot,qref,qrefdot);

	vector<double> O(3),Odot(O),thodot(2);
	CalcOrientation(O,q);
	thodot(0) = tha1dot;
	thodot(1) = thcdot;
	Odot = prod(Jo,thodot);

	vector<double>	Oref(3),Orefdot(Oref),Orefddot(Oref),
					thorefdot(2),thorefddot(2);
	CalcOrientation(Oref,qref);
	thorefdot(0) = tha1refdot;
	thorefdot(1) = thcrefdot;
	thorefddot(0) = tha1refddot;
	thorefddot(1) = thcrefddot;
	Orefdot = prod(Joref,thorefdot);
	Orefddot = prod(Jorefdot,thorefdot) + prod(Joref,thorefddot);

	vector<double> eo(3),eodot(eo);
	eo = Oref - O;
	eodot = Orefdot - Odot;
		
	vector<double> torque(2);

	torque = Orefddot + prod(kvo,eodot) + prod(kpo,eo) - prod(Jodot,thodot);
	torque = prod(prod((matrix<double>)prod(Mo,invtJoJo),tJo),torque) + prod(Co,thodot);
	//if(tha1 == 0)
	//	torque(0) = torque(1) = 5;
	ur = torque;

	double phi = acos( inner_prod(Oref,O) );
	return phi;
}

double oritd(double time,vector<double>& q,vector<double>& qdot,vector<double>& qrefh,vector<double>& qrefhdot,vector<double>& qrefhddot,vector<double>& ur,double& phi_t)
{
	using namespace control;
	using namespace control::conparam;
	volatile double d[3];


	volatile double	tha1,thc,tha1ref,thcref,tha1refh,thcrefh,
			tha1dot,thcdot,tha1refdot,thcrefdot,tha1refhdot,thcrefhdot,
			tha1ddot,thcddot,tha1refddot,thcrefddot,tha1refhddot,thcrefhddot;
	vector<double> qref(5),qrefdot(5),qrefddot(5);
	GenerateWReference(time,qref,qrefdot,qrefddot);
	tha1		 = q(4),		thc			 = q(3);
	tha1dot		 = qdot(4),		thcdot		 = qdot(3);
	tha1refh	 = qrefh(4),	thcrefh		 = qrefh(3);
	tha1refhdot	 = qrefhdot(4),	thcrefhdot	 = qrefhdot(3);
	tha1refhddot = qrefhddot(4),thcrefhddot	 = qrefhddot(3);
	tha1ref		 = qref(4),		thcref		 = qref(3);
	tha1refdot	 = qrefdot(4),	thcrefdot	 = qrefdot(3);
	tha1refddot	 = qrefddot(4),	thcrefddot	 = qrefddot(3);

	// orientation
	matrix<double>	Jo(3,2),Jodot(Jo),
					Joref(Jo),Jorefdot(Jo),
					Jorefh(Jo),Jorefhdot(Jo),
					tJo,invtJoJo;
	static matrix<double> sJo(3,2),sJoref(3,2);
	CalcJo(Jo,q);
	CalcJodot(Jodot,q,qdot);
	tJo = trans(Jo);
	math::invert(prod(tJo,Jo),invtJoJo);
	CalcJo(Joref,qref);
	CalcJodot(Jorefdot,qref,qrefdot);
	CalcJo(Jorefh,qrefh);
	CalcJodot(Jorefhdot,qrefh,qrefhdot);

	vector<double> O(3),Odot(O),thodot(2);
	CalcOrientation(O,q);
	thodot(0) = tha1dot;
	thodot(1) = thcdot;
	Odot = prod(Jo,thodot);

	vector<double>	Oref(3),Orefdot(Oref),Orefddot(Oref),
					thorefdot(2),thorefddot(2);
	CalcOrientation(Oref,qref);
	thorefdot(0) = tha1refdot;
	thorefdot(1) = thcrefdot;
	thorefddot(0) = tha1refddot;
	thorefddot(1) = thcrefddot;
	Orefdot = prod(Joref,thorefdot);
	Orefddot = prod(Jorefdot,thorefdot) + prod(Joref,thorefddot);

	vector<double>	Orefh(3),Orefhdot(Orefh),Orefhddot(Orefh),
					thorefhdot(2),thorefhddot(2);
	CalcOrientation(Orefh,qrefh);
	thorefhdot(0) = tha1refhdot;
	thorefhdot(1) = thcrefhdot;
	thorefhddot(0) = tha1refhddot;
	thorefhddot(1) = thcrefhddot;
	Orefhdot = prod(Jorefh,thorefhdot);
	Orefhddot = prod(Jorefhdot,thorefhdot) + prod(Jorefh,thorefhddot);

	vector<double> eo(3),eodot(eo);
	eo = Oref - O;
	eodot = Orefdot - Odot;
	for(int i=0;i<3;i++){
		d[i]=eo(i);
		d[i]=O(i);
		d[i]=Oref(i);
	}
	for(int i=0;i<3;i++)
		d[i]=eodot(i);
	d[0] = tha1refdot*-sin(tha1ref)*sin(thcref)+cos(tha1ref)*thcrefdot*cos(thcref);
	//cos(tha1ref)*sin(thcref);
	d[1] = qrefdot(4)*-sin(qref(4))*sin(qref(3))+cos(qref(4))*qrefdot(3)*cos(qref(3));
	//cos(wref(4))*sin(wref(3));
		
	vector<double> torque(2);

	//torque = Orefddot + prod(kvo,eodot) + prod(kpo,eo) - prod(Jodot,thodot);
	//torque = prod(prod((matrix<double>)prod(Mo,invtJoJo),tJo),torque) + prod(Co,thodot);

	vector<double> eoh(3),eohdot(eo);
	eoh = Orefh - O;
	eohdot = Orefhdot - Odot;

	torque = Orefhddot + prod(kvo,eohdot) + prod(kpo,eoh) - prod(Jodot,thodot);
	torque = prod(prod((matrix<double>)prod(Mo,invtJoJo),tJo),torque) + prod(Co,thodot);

	ur = torque;

	volatile double d1,d2;
	d1=torque(0);
	d2=torque(1);
	d[0] = eo(0);d[1] = eo(1);d[2] = eo(2);

	double phi_td = acos( inner_prod(Orefh,O) );
	phi_t = acos( inner_prod(Oref,O) );
	return phi_td;
}

void control::IndependentCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	unsigned int step;
	PLANT plant;
	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
	double time;

	using namespace control::conparam;

	matrix<double> At(2,2,0);
	vector<double> Bt(2,0),Ct(2,0),Lt(2,0),Xh(2);
	double Y(0);
	At(0,1)=1;At(1,1)=-c(0,0)/mass(0,0);
	Bt(0) = 0;Bt(1)=1/mass(0,0);
	Ct(0)=1;
	Lt(0)=0.01853*100000,Lt(1)=8.1255*100000;
	observer::SetObserver(At,Bt,Ct,Lt);

	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		//x y1 y2 z c a1 a2	::	x y1 z c a1
		//0 1  2  3 4 5  6	::	0 1  2 3 4

		// get state value(M-system)
		vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);

		// set state value(M-system)
		vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
		SetStateVal(q,qdot,qddot,posall,velall,accall);

		// M-systemの状態量をM-systemに変換(q -> w)
		// 位置情報の変換行列の計算
		matrix<double> T(5,5),TT(5,2);
		CalcConvertMatrixT(T,TT,q);
		// オフセットの用意
		vector<double> offset(2);
		double a2 = OFFSET_BETWEEN_COORDINATE,
			   d4 = DISTANCE_FROM_A_TO_C_AXIS;
		offset(0) = d4;
		offset(1) = a2;
		// M-systemの位置情報をP-systemに変換
		w = prod(T,q) + prod(TT,offset);
		// 速度情報の変換行列の計算
		matrix<double> J(5,5),Jdot(J),invJ(J);
		CalcJ(J,q);
		math::invert(J,invJ);
		// M-systemの速度情報をP-systemに変換
		wdot = prod(J,qdot);
		// 加速度情報の変換行列の計算
		CalcJdot(q,qdot,Jdot);
		// M-systemの加速度情報をP-systemに変換
		wddot = prod(J,qddot) + prod(Jdot,qdot);
		

		// get reference(P-system)
		vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
		GenerateWReference(time,wref,wrefdot,wrefddot);
		
		// P-systemの目標値をM-systemに変換(wref -> qref)
		// 位置情報の変換行列の計算
		matrix<double> Tref(5,5),TTref(5,2),invTref;
		CalcConvertMatrixT(Tref,TTref,wref);
		math::invert(Tref,invTref);
		// P-systemの位置情報をM-systemに変換
		qref = prod( invTref,wref - prod(TTref,offset) );
		// 速度情報の変換行列の計算
		matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
		CalcJ(Jref,qref);
		math::invert(Jref,invJref);
		// P-systemの速度情報をM-systemに変換
		qrefdot = prod(invJref,wrefdot);
		// 加速度情報の変換行列の計算
		CalcJdot(qref,qrefdot,Jrefdot);
		// P-systemの加速度情報をM-systemに変換
		qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));
		

		// calculation tracking error
		// M-systemでの誤差（追従誤差）
		vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
		eq = q - qref;
		eqdot = qdot- qrefdot;
		eqddot = qddot - qrefddot;

		force = prod(mass,qrefddot - prod(kvw,eqdot) - prod(kpw,eq)) + prod(c,qdot);

		// calculation control value(sync)
		vector<double> es(2),esdot(es),forces(es),
					   poss(es),vels(es),accs(es),
					   rposs(es),rvels(es),raccs(es);
		poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
		poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
		rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
		rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
		es = poss - rposs;
		esdot = vels - rvels;
		forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);

		
		forceall(0) = force(0);
		forceall(1) = force(1);
		forceall(2) = force(1) + forces(0);
		forceall(3) = force(2);
		forceall(4) = force(3);// torque(1);
		forceall(5) = force(4);// torque(0);
		forceall(6) = force(4) + forces(1);// torque(0) + forces(1);		

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);
		
		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		Y=q(0);
		observer::CalcObserver(Y,forceall(0),Xh);
		volatile double d;
		d=Xh(0);
		//d[1]=Xh(1);

		// loging data
		qref_log(0) = qdot(0)-Xh(1);//qref(0);
		qref_log(1) = qref(1);
		qref_log(2) = rposs(0);
		qref_log(3) = qref(2);
		qref_log(4) = qref(3);
		qref_log(5) = qref(4);
		qref_log(6) = rposs(1);
		qrefdot_log(0) = qrefdot(0);
		qrefdot_log(1) = qrefdot(1);
		qrefdot_log(2) = rvels(0);
		qrefdot_log(3) = qrefdot(2);
		qrefdot_log(4) = qrefdot(3);
		qrefdot_log(5) = qrefdot(4);
		qrefdot_log(6) = rvels(1);
		qrefddot_log(0) = qrefddot(0);
		qrefddot_log(1) = qrefddot(1);
		qrefddot_log(2) = raccs(0);
		qrefddot_log(3) = qrefddot(2);
		qrefddot_log(4) = qrefddot(3);
		qrefddot_log(5) = qrefddot(4);
		qrefddot_log(6) = raccs(1);
		eq_log(0) = eq(0);
		eq_log(1) = eq(1);
		eq_log(2) = posall(2) - qref(1);		//
		eq_log(3) = eq(2);
		eq_log(4) = eq(3);
		eq_log(5) = eq(4);
		eq_log(6) = posall(6) - qref(4);		//
		force_log = forceall;
		vector<double> w_log(7);
		w_log = posall;
		w_log(0) = w(0);
		w_log(1) = w(1);
		w_log(3) = w(2);
		datalog[0,step] = time;
		datalog[1,step] = 0;
		datalog[2,step] = 0;
		datalog[3,step] = 0;			//ec-enmod
		datalog[4,step] = 0;			//td
		datalog[5,step] = 0;			//el t
		datalog[6,step] = 0;			//el n
		datalog[7,step] = 0;			//el b
		datalog[8,step] = posall(2) - posall(1);	//es y
		datalog[9,step] = posall(6) - posall(5);	//es a
		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
			datalog[j+10,step] = qref_log(i);
			datalog[j+11,step] = qrefdot_log(i);
			datalog[j+12,step] = qrefddot_log(i);
			//datalog[j+13,step] = posall(i);
			datalog[j+13,step] = w_log(i);
			datalog[j+14,step] = velall(i);
			datalog[j+15,step] = accall(i);
			datalog[j+16,step] = force_log(i);
			datalog[j+17,step] = eq_log(i);
		}
	}
}

void control::ContouringCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	unsigned int step;
	PLANT plant;
	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
	double time;

	volatile double d[10];

	using namespace control::conparam;


	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		//x y1 y2 z c a1 a2	::	x y1 z c a1
		//0 1  2  3 4 5  6	::	0 1  2 3 4

		// get state value(M-system)
		vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);

		// set state value(M-system)
		vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
		SetStateVal(q,qdot,qddot,posall,velall,accall);

		// M-systemの状態量をM-systemに変換(q -> w)
		// 位置情報の変換行列の計算
		matrix<double> T(5,5),TT(5,2);
		CalcConvertMatrixT(T,TT,q);
		// オフセットの用意
		vector<double> offset(2);
		double a2 = OFFSET_BETWEEN_COORDINATE,
			   d4 = DISTANCE_FROM_A_TO_C_AXIS;
		offset(0) = d4;
		offset(1) = a2;
		// M-systemの位置情報をP-systemに変換
		w = prod(T,q) + prod(TT,offset);
		// 速度情報の変換行列の計算
		matrix<double> J(5,5),Jdot(J),invJ(J);
		CalcJ(J,q);
		math::invert(J,invJ);
		// M-systemの速度情報をP-systemに変換
		wdot = prod(J,qdot);
		// 加速度情報の変換行列の計算
		CalcJdot(q,qdot,Jdot);
		// M-systemの加速度情報をP-systemに変換
		wddot = prod(J,qddot) + prod(Jdot,qdot);
		

		// get reference(P-system)
		vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
		GenerateWReference(time,wref,wrefdot,wrefddot);
		
		// P-systemの目標値をM-systemに変換(wref -> qref)
		// 位置情報の変換行列の計算
		matrix<double> Tref(5,5),TTref(5,2),invTref;
		CalcConvertMatrixT(Tref,TTref,wref);
		math::invert(Tref,invTref);
		// P-systemの位置情報をM-systemに変換
		qref = prod( invTref,wref - prod(TTref,offset) );
		// 速度情報の変換行列の計算
		matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
		CalcJ(Jref,qref);
		math::invert(Jref,invJref);
		// P-systemの速度情報をM-systemに変換
		qrefdot = prod(invJref,wrefdot);
		// 加速度情報の変換行列の計算
		CalcJdot(qref,qrefdot,Jrefdot);
		// P-systemの加速度情報をM-systemに変換
		qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));
		

		// calculation tracking error
		// M-systemでの誤差（追従誤差）
		vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
		eq = q - qref;
		eqdot = qdot- qrefdot;
		eqddot = qddot - qrefddot;

		// P-systemでの誤差
		vector<double>  e(NUM_DOF),edot(e),eddot(e);
		e = wref - w;
		edot = wrefdot - wdot;
		eddot = wrefddot - wddot;

		matrix<double> F(5,5),Fdot(F),Fddot(F),tF,tFdot,tFddot;
		static matrix<double> oldF(5,5,0),oldFdot(5,5,0);
		if(time == 0)
			oldF.clear(),oldFdot.clear();
		CalcFrenetFrame(wrefdot,wrefddot,F);	// get Frenet frame(current)
		Fdot = (F - oldF) / SAMPLING_TIME;
		oldF = F;
		Fddot = (Fdot - oldFdot) / SAMPLING_TIME;
		oldFdot = Fdot;

		vector<double> w1(5),w2(w1),w3(w1),w4(w1);
		GenerateWReference(time,wref,w1,w2);
		GenerateWReference3dot(time,w3);
		GenerateWReference4dot(time,w4);
		CalcFrenetFrameddot(w1,w2,w3,w4,F,Fdot,Fddot);
		tF = trans(F);
		tFdot = trans(Fdot);
		tFddot = trans(Fddot);
		
		vector<double> eF(NUM_DOF),eFdot(eF);
		eF = prod(tF,e);
		eFdot = prod(tFdot,e) + prod(tF,edot);


		force = wrefddot + prod(F,prod(kvl,eFdot) + prod(kpl,eF) + 2*prod(tFdot,edot) + prod(tFddot,e)) - prod(Jdot,qdot);
		force = prod(prod(mass,invJ),force) + prod(c,qdot);


		// orientation
		vector<double> torque(2),ur(2);
		double epo,phi_t,phi_td;
		phi_t = ori(time,q,qdot,qddot,qref,qrefdot,qrefddot,torque,epo);
		
		vector<double> Prefdot(3);
		for(i=0;i<Prefdot.size();i++)
			Prefdot(i) = wrefdot(i);
		double td = eF(0) / norm_2(Prefdot);
		// get reference
		vector<double> wrefh(5),wrefhdot(5),wrefhddot(5);
		GenerateWReference(time-td,wrefh,wrefhdot,wrefhddot);

		phi_td = oritd(time,w,wdot,wrefh,wrefhdot,wrefhddot,ur,phi_t);
		//torque(0) = force(4);
		//torque(1) = force(3);

		// calculation control value(sync)
		vector<double> es(2),esdot(es),
					   poss(es),vels(es),accs(es),rposs(es),
					   rvels(es),raccs(es),forces(es);
		poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
		poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
		rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
		rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
		es = poss - rposs;
		esdot = vels - rvels;
		forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);

		
		forceall(0) = force(0);
		forceall(1) = force(1);
		forceall(2) = force(1) + forces(0);
		forceall(3) = force(2);
		forceall(4) = torque(1);
		forceall(5) = torque(0);
		forceall(6) = torque(0) + forces(1);

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);
		
		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// loging data
		qref_log(0) = qref(0);
		qref_log(1) = qref(1);
		qref_log(2) = rposs(0);
		qref_log(3) = qref(2);
		qref_log(4) = qref(3);
		qref_log(5) = qref(4);
		qref_log(6) = rposs(1);
		qrefdot_log(0) = qrefdot(0);
		qrefdot_log(1) = qrefdot(1);
		qrefdot_log(2) = rvels(0);
		qrefdot_log(3) = qrefdot(2);
		qrefdot_log(4) = qrefdot(3);
		qrefdot_log(5) = qrefdot(4);
		qrefdot_log(6) = rvels(1);
		qrefddot_log(0) = qrefddot(0);
		qrefddot_log(1) = qrefddot(1);
		qrefddot_log(2) = raccs(0);
		qrefddot_log(3) = qrefddot(2);
		qrefddot_log(4) = qrefddot(3);
		qrefddot_log(5) = qrefddot(4);
		qrefddot_log(6) = raccs(1);
		eq_log(0) = eq(0);
		eq_log(1) = eq(1);
		eq_log(2) = posall(2) - qref(1);		//
		eq_log(3) = eq(2);
		eq_log(4) = eq(3);
		eq_log(5) = eq(4);
		eq_log(6) = posall(6) - qref(4);		//
		force_log = forceall;
		vector<double> w_log(7);
		w_log = posall;
		w_log(0) = w(0);
		w_log(1) = w(1);
		w_log(3) = w(2);
		datalog[0,step] = time;
		datalog[1,step] = 0;
		datalog[2,step] = sqrt(eF(1)*eF(1)+eF(2)*eF(2));;
		datalog[3,step] = 0;			//ec-enmod
		datalog[4,step] = 0;			//td
		datalog[5,step] = eF(0);			//el t
		datalog[6,step] = eF(1);			//el n
		datalog[7,step] = eF(2);			//el b
		datalog[8,step] = posall(2) - posall(1);	//es y
		datalog[9,step] = posall(6) - posall(5);	//es a
		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
			datalog[j+10,step] = qref_log(i);
			datalog[j+11,step] = qrefdot_log(i);
			datalog[j+12,step] = qrefddot_log(i);
			//datalog[j+13,step] = posall(i);
			datalog[j+13,step] = w_log(i);
			datalog[j+14,step] = velall(i);
			datalog[j+15,step] = accall(i);
			datalog[j+16,step] = force_log(i);
			datalog[j+17,step] = eq_log(i);
		}
		datalog[66,step] = phi_t;
		datalog[67,step] = phi_td;
	}
}

//void control::modContouringCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
//{
//	int i,j;
//	unsigned int step;
//	PLANT plant;
//	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
//				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
//	double time;
//
//	volatile double d[10];
//
//	using namespace control::conparam;
//
//
//	for(step=0;step<max_step;step++)
//	{
//		time = step * SAMPLING_TIME;
//		
//		//x y1 y2 z c a1 a2
//		//0 1  2  3 4 5  6
//
//		// get state value(M-system)
//		vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
//		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
//
//		// set state value(M-system)
//		vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
//		SetStateVal(q,qdot,qddot,posall,velall,accall);
//
//		// M-systemの状態量をM-systemに変換(q -> w)
//		// 位置情報の変換行列の計算
//		matrix<double> T(5,5),TT(5,2);
//		CalcConvertMatrixT(T,TT,q);
//		// オフセットの用意
//		vector<double> offset(2);
//		double a2 = OFFSET_BETWEEN_COORDINATE,
//			   d4 = DISTANCE_FROM_A_TO_C_AXIS;
//		offset(0) = d4;
//		offset(1) = a2;
//		// M-systemの位置情報をP-systemに変換
//		w = prod(T,q) + prod(TT,offset);
//		// 速度情報の変換行列の計算
//		matrix<double> J(5,5),Jdot(J),invJ(J);
//		CalcJ(J,q);
//		math::invert(J,invJ);
//		// M-systemの速度情報をP-systemに変換
//		wdot = prod(J,qdot);
//		// 加速度情報の変換行列の計算
//		CalcJdot(q,qdot,Jdot);
//		// M-systemの加速度情報をP-systemに変換
//		wddot = prod(J,qddot) + prod(Jdot,qdot);
//		
//
//		// get reference(P-system)
//		vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
//		GenerateWReference(time,wref,wrefdot,wrefddot);
//		
//		// P-systemの目標値をM-systemに変換(wref -> qref)
//		// 位置情報の変換行列の計算
//		matrix<double> Tref(5,5),TTref(5,2),invTref;
//		CalcConvertMatrixT(Tref,TTref,wref);
//		math::invert(Tref,invTref);
//		// P-systemの位置情報をM-systemに変換
//		qref = prod( invTref,wref - prod(TTref,offset) );
//		// 速度情報の変換行列の計算
//		matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
//		CalcJ(Jref,qref);
//		math::invert(Jref,invJref);
//		// P-systemの速度情報をM-systemに変換
//		qrefdot = prod(invJref,wrefdot);
//		// 加速度情報の変換行列の計算
//		CalcJdot(qref,qrefdot,Jrefdot);
//		// P-systemの加速度情報をM-systemに変換
//		qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));
//		
//
//		// calculation tracking error
//		// M-systemでの誤差（追従誤差）
//		vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
//		eq = q - qref;
//		eqdot = qdot- qrefdot;
//		eqddot = qddot - qrefddot;
//
//		// P-systemでの誤差
//		vector<double>  e(NUM_DOF),edot(e),eddot(e);
//		e = wref - w;
//		edot = wrefdot - wdot;
//		eddot = wrefddot - wddot;
//
//		matrix<double> F(5,5),Fdot(F),Fddot(F),tF,tFdot,tFddot;
//		static matrix<double> oldF(5,5,0),oldFdot(5,5,0);
//		if(time == 0)
//			oldF.clear(),oldFdot.clear();
//		CalcFrenetFrame(wrefdot,wrefddot,F);	// get Frenet frame(current)
//		Fdot = (F - oldF) / SAMPLING_TIME;
//		oldF = F;
//		Fddot = (Fdot - oldFdot) / SAMPLING_TIME;
//		oldFdot = Fdot;
//		tF = trans(F);
//		tFdot = trans(Fdot);
//		tFddot = trans(Fddot);
//		
//		vector<double> eF(NUM_DOF),eFdot(eF),eFddot(eF);
//		eF = prod(tF,e);
//		eFdot = prod(tFdot,e) + prod(tF,edot);
//		eFddot = prod(tFddot,e) + 2*prod(tFdot,edot) + prod(tF,eddot);
//
//
//		force = wrefddot + prod(F,prod(kvl,eFdot) + prod(kpl,eF) + 2*prod(tFdot,edot) + prod(tFddot,e)) - prod(Jdot,qdot);
//		force = prod(prod(mass,invJ),force) + prod(c,qdot);
//
//		
//		vector<double> Prefdot(3),Prefddot(3),Pref3dot(3),ww3(5);
//		Prefdot(0) = wrefdot(0);
//		Prefdot(1) = wrefdot(1);
//		Prefdot(2) = wrefdot(2);
//		Prefddot(0) = wrefddot(0);
//		Prefddot(1) = wrefddot(1);
//		Prefddot(2) = wrefddot(2);
//		GenerateWReference3dot(time,ww3);
//		Pref3dot(0) = ww3(0);
//		Pref3dot(1) = ww3(1);
//		Pref3dot(2) = ww3(2);
//		double td,tddot,tdddot;
//		static double oldtd,oldtddot;
//		td = eF(0) / norm_2(Prefdot);
//		tddot = (td-oldtd) / SAMPLING_TIME;
//		tdddot = (tddot-oldtddot) / SAMPLING_TIME;
//		oldtd = td;
//		oldtddot = tddot;
//		double nPrefddot = norm_2(Prefddot),
//			nPrefdot = norm_2(Prefdot),
//			nPrefdotdot = inner_prod(Prefdot,Prefddot)/nPrefdot,
//			nPrefdotddot = (nPrefddot*nPrefddot-nPrefdot*nPrefdot+inner_prod(Prefdot,Pref3dot))/nPrefdot;
//		tddot = (eFdot(0) - td*nPrefdotdot)/(nPrefdot);
//		tdddot = (eFddot(0)-2*tddot*nPrefdotdot-td*nPrefdotddot)/(nPrefdot);
//
//		// get reference
//		vector<double> wrefh(5),wrefhdot(5),wrefhddot(5);
//		double thcrefh,tha1refh,thcrefhdot,tha1refhdot,thcrefhddot,tha1refhddot;
//		GenerateWReference(time-td,wrefh,wrefhdot,wrefhddot);
//		//wrefhddot = -tdddot*wrefhdot + (1-tddot)*wrefhddot;
//		//wrefhdot = (1-tddot)*wrefhdot;
//		
//
//		matrix<double> Fh(5,5),Fhdot(Fh),Fhddot(Fh),tFh,tFhdot,tFhddot;
//		vector<double> w3(5),w4(w3);
//		GenerateWReference3dot(time-td,w3);
//		GenerateWReference4dot(time-td,w4);
//		CalcFrenetFrameddot(wrefhdot,wrefhddot,w3,w4,Fh,Fhdot,Fhddot);
//		/*
//		static matrix<double> oldFh(5,5,0),oldFhdot(5,5,0);
//		Fhdot = (Fh - oldFh) / SAMPLING_TIME;
//		oldFh = Fh;
//		Fhddot = (Fhdot - oldFhdot) / SAMPLING_TIME;
//		oldFhdot = Fhdot;
//		//*/
//		tFh = trans(Fh);
//		tFhdot = trans(Fhdot);
//		tFhddot = trans(Fhddot);
//
//		vector<double> h(5),hdot(h),hddot(h),
//					   hF(5),hFdot(hF),hFddot(hF),
//					   hFh(5),hFhdot(hFh),hFhddot(hFh);
//		h(0) = wref(0) - wrefh(0);
//		h(1) = wref(1) - wrefh(1);
//		h(2) = wref(2) - wrefh(2);
//		h(3) = 0;
//		h(4) = 0;
//		hdot(0) = wrefdot(0) - wrefhdot(0);
//		hdot(1) = wrefdot(1) - wrefhdot(1);
//		hdot(2) = wrefdot(2) - wrefhdot(2);
//		hdot(3) = 0;
//		hdot(4) = 0;
//		hddot(0) = wrefddot(0) - wrefhddot(0);
//		hddot(1) = wrefddot(1) - wrefhddot(1);
//		hddot(2) = wrefddot(2) - wrefhddot(2);
//		hddot(3) = 0;
//		hddot(4) = 0;
//
//		hF = prod(tFh,h);
//		hFdot = prod(tFhdot,h) + prod(tFh,hdot);
//		hFddot = prod(tFhddot,h) + 2*prod(tFhdot,hdot) + prod(tFh,hddot);
//		matrix<double> W(5,5,0);
//		vector<double> eFh(5),eFhdot(eFh);
//		W(1,1) = -1;
//		W(2,2) = -1;
//		hFh = prod(W,hF);
//		hFhdot = prod(W,hFdot);
//		hFhddot = prod(W,hFddot);
//		eFh = prod(tFh,e) + hFh;
//		eFhdot = prod(tFhdot,e) + prod(tFh,edot) + hFhdot;
//
//		vector<double> uf(5);
//		uf = wrefddot + prod(Fh,prod(kvl,eFhdot) + prod(kpl,eFh) + hFhddot + 2*prod(tFhdot,edot) + prod(tFhddot,e)) - prod(Jdot,qdot);
//		uf = prod(prod(mass,invJ),uf) + prod(c,qdot);
//		
//		//wrefddot(3) = wrefhddot(3);
//		force = wrefddot + prod(F,prod(kvl,eFdot) + prod(kpl,eF) + 2*prod(tFdot,edot) + prod(tFddot,e)) - prod(Jdot,qdot);
//		force = prod(prod(mass,invJ),force) + prod(c,qdot);
//		force = uf;
//		double bv = kvl(4,4),bp = kpl(4,4);
//		e(4) = wrefh(4) - w(4);
//		edot(4) = (1-tddot)*wrefhdot(4) - wdot(4);
//		force(4) = -tdddot*wrefhdot(4)+(1-tddot)*(1-tddot)*wrefhddot(4) + bv*edot(4) + bp*e(4);
//		edot(4) = wrefdot(4) - wdot(4);
//		force(4) = wrefddot(4) + bv*edot(4) + bp*e(4);
//		force(4) = mass(4,4)*force(4) + c(4,4)*qdot(4);
//
//
//		// orientation
//		vector<double> torque(2),ur(2);
//		double epo;
//		ori(time,q,qdot,qddot,qref,qrefdot,qrefddot,torque,epo);
//		oritd(time-td,q,qdot,wrefh,wrefhdot,wrefhddot,ur);
//		torque = ur;
//		torque(0) = force(4);
//		torque(1) = force(3);
//
//		// calculation control value(sync)
//		vector<double> es(2),esdot(es),
//					   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
//		poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
//		poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
//		rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
//		rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
//		es = poss - rposs;
//		esdot = vels - rvels;
//		forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);
//
//		
//		forceall(0) = force(0);
//		forceall(1) = force(1);
//		forceall(2) = force(1) + forces(0);
//		forceall(3) = force(2);
//		forceall(4) = torque(1);
//		forceall(5) = torque(0);
//		forceall(6) = torque(0) + forces(1);
//
//		// output control value
//		iocda::OutputControlVariable(enable_resolution,forceall);
//		
//		// input control value to plantmodel
//		plant.plantmodel(time,forceall,enable_noise);
//
//		// loging data
//		qref_log(0) = qref(0);
//		qref_log(1) = qref(1);
//		qref_log(2) = rposs(0);
//		qref_log(3) = qref(2);
//		qref_log(4) = qref(3);
//		qref_log(5) = qref(4);
//		qref_log(6) = rposs(1);
//		qrefdot_log(0) = qrefdot(0);
//		qrefdot_log(1) = qrefdot(1);
//		qrefdot_log(2) = rvels(0);
//		qrefdot_log(3) = qrefdot(2);
//		qrefdot_log(4) = qrefdot(3);
//		qrefdot_log(5) = qrefdot(4);
//		qrefdot_log(6) = rvels(1);
//		qrefddot_log(0) = qrefddot(0);
//		qrefddot_log(1) = qrefddot(1);
//		qrefddot_log(2) = raccs(0);
//		qrefddot_log(3) = qrefddot(2);
//		qrefddot_log(4) = qrefddot(3);
//		qrefddot_log(5) = qrefddot(4);
//		qrefddot_log(6) = raccs(1);
//		eq_log(0) = eq(0);
//		eq_log(1) = eq(1);
//		eq_log(2) = posall(2) - qref(1);		//
//		eq_log(3) = eq(2);
//		eq_log(4) = eq(3);
//		eq_log(5) = eq(4);
//		eq_log(6) = posall(6) - qref(4);		//
//		force_log = forceall;
//		vector<double> w_log(7);
//		w_log = posall;
//		w_log(0) = w(0);
//		w_log(1) = w(1);
//		w_log(3) = w(2);
//		datalog[0,step] = time;
//		datalog[1,step] = 0;
//		datalog[2,step] = sqrt(eF(1)*eF(1)+eF(2)*eF(2));
//		datalog[3,step] = sqrt(eFh(1)*eFh(1)+eFh(2)*eFh(2));			//ec-enmod
//		datalog[4,step] = td;			//td
//		datalog[5,step] = eF(0);			//el t
//		datalog[6,step] = eF(1);			//el n
//		datalog[7,step] = eF(2);			//el b
//		datalog[8,step] = posall(2) - posall(1);	//es y
//		datalog[9,step] = posall(6) - posall(5);	//es a
//		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
//			datalog[j+10,step] = qref_log(i);
//			datalog[j+11,step] = qrefdot_log(i);
//			datalog[j+12,step] = qrefddot_log(i);
//			//datalog[j+13,step] = posall(i);
//			datalog[j+13,step] = w_log(i);
//			datalog[j+14,step] = velall(i);
//			datalog[j+15,step] = accall(i);
//			datalog[j+16,step] = force_log(i);
//			datalog[j+17,step] = eq_log(i);
//		}
//	}
//}

void control::modContouringCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	unsigned int i,j,step;
	PLANT plant;
	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
	double time;

	volatile double d[10];

	using namespace control::conparam;


	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		//x y1 y2 z c a1 a2
		//0 1  2  3 4 5  6

		// get state value(M-system)
		vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);

		// set state value(M-system)
		vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
		SetStateVal(q,qdot,qddot,posall,velall,accall);

		// M-systemの状態量をM-systemに変換(q -> w)
		// 位置情報の変換行列の計算
		matrix<double> T(5,5),TT(5,2);
		CalcConvertMatrixT(T,TT,q);
		// オフセットの用意
		vector<double> offset(2);
		double a2 = OFFSET_BETWEEN_COORDINATE,
			   d4 = DISTANCE_FROM_A_TO_C_AXIS;
		offset(0) = d4;
		offset(1) = a2;
		// M-systemの位置情報をP-systemに変換
		w = prod(T,q) + prod(TT,offset);
		// 速度情報の変換行列の計算
		matrix<double> J(5,5),Jdot(J),invJ(J);
		CalcJ(J,q);
		math::invert(J,invJ);
		// M-systemの速度情報をP-systemに変換
		wdot = prod(J,qdot);
		// 加速度情報の変換行列の計算
		CalcJdot(q,qdot,Jdot);
		// M-systemの加速度情報をP-systemに変換
		wddot = prod(J,qddot) + prod(Jdot,qdot);
		

		// get reference(P-system)
		vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
		GenerateWReference(time,wref,wrefdot,wrefddot);
		
		// P-systemの目標値をM-systemに変換(wref -> qref)
		// 位置情報の変換行列の計算
		matrix<double> Tref(5,5),TTref(5,2),invTref;
		CalcConvertMatrixT(Tref,TTref,wref);
		math::invert(Tref,invTref);
		// P-systemの位置情報をM-systemに変換
		qref = prod( invTref,wref - prod(TTref,offset) );
		// 速度情報の変換行列の計算
		matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
		CalcJ(Jref,qref);
		math::invert(Jref,invJref);
		// P-systemの速度情報をM-systemに変換
		qrefdot = prod(invJref,wrefdot);
		// 加速度情報の変換行列の計算
		CalcJdot(qref,qrefdot,Jrefdot);
		// P-systemの加速度情報をM-systemに変換
		qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));
		

		// calculation tracking error
		// M-systemでの誤差（追従誤差）
		vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
		eq = q - qref;
		eqdot = qdot- qrefdot;
		eqddot = qddot - qrefddot;

		// P-systemでの誤差
		vector<double>  e(NUM_DOF),edot(e),eddot(e);
		e = wref - w;
		edot = wrefdot - wdot;
		eddot = wrefddot - wddot;

		matrix<double> F(5,5),Fdot(F),Fddot(F),tF,tFdot,tFddot;
		vector<double> w3(5),w4(w3);
		GenerateWReference3dot(time,w3);
		GenerateWReference4dot(time,w4);
		CalcFrenetFrameddot(wrefdot,wrefddot,w3,w4,F,Fdot,Fddot);	// get Frenet frame(current)
		tF = trans(F);
		tFdot = trans(Fdot);
		tFddot = trans(Fddot);
		
		vector<double> eF(NUM_DOF),eFdot(eF),eFddot(eF);
		eF = prod(tF,e);
		eFdot = prod(tFdot,e) + prod(tF,edot);
		eFddot = prod(tFddot,e) + 2*prod(tFdot,edot) + prod(tF,eddot);

		force = wrefddot + prod(F,prod(kvl,eFdot) + prod(kpl,eF) + 2*prod(tFdot,edot) + prod(tFddot,e)) - prod(Jdot,qdot);
		force = prod(prod(mass,invJ),force) + prod(c,qdot);
		
		vector<double> Prefdot(3),Prefddot(3),Pref3dot(3);
		for(i=0;i<Prefdot.size();i++){
			Prefdot(i) = wrefdot(i);
			Prefddot(i) = wrefddot(i);
			Pref3dot(i) = w3(i);
		}
		double td,tddot,tdddot;
		static double oldtd,oldtddot;
		td = eF(0) / norm_2(Prefdot);
		tddot = (td-oldtd) / SAMPLING_TIME;
		tdddot = (tddot-oldtddot) / SAMPLING_TIME;
		oldtd = td;
		oldtddot = tddot;
		double nPrefddot = norm_2(Prefddot),
			nPrefdot = norm_2(Prefdot),
			nPrefdotdot = inner_prod(Prefdot,Prefddot)/nPrefdot,
			nPrefdotddot = (nPrefddot*nPrefddot-nPrefdot*nPrefdot+inner_prod(Prefdot,Pref3dot))/nPrefdot;
		tddot = (eFdot(0) - td*nPrefdotdot)/(nPrefdot);
		tdddot = (eFddot(0)-2*tddot*nPrefdotdot-td*nPrefdotddot)/(nPrefdot);

		// get reference
		vector<double> wrefh(5),wrefhdot(5),wrefhddot(5);
		double thcrefh,tha1refh,thcrefhdot,tha1refhdot,thcrefhddot,tha1refhddot;
		GenerateWReference(time-td,wrefh,wrefhdot,wrefhddot);		

		matrix<double> Fh(5,5),Fhdot(Fh),Fhddot(Fh),tFh,tFhdot,tFhddot;
		GenerateWReference3dot(time-td,w3);
		GenerateWReference4dot(time-td,w4);
		CalcFrenetFrameddot(wrefhdot,wrefhddot,w3,w4,Fh,Fhdot,Fhddot);
		tFh = trans(Fh);
		tFhdot = trans(Fhdot);
		tFhddot = trans(Fhddot);

		matrix<double> Q(5,5),Qdot(Q),Qddot(Q),S(5,5,0);
		S(1,1)=S(2,2)=S(3,3)=S(4,4)=1;
		Q = prod(matrix<double>(prod(Fh,S)),tFh);
		Qdot = prod(matrix<double>(prod(Fhdot,S)),tFh)+prod(matrix<double>(prod(Fh,S)),tFhdot);
		Qddot = prod(matrix<double>(prod(Fhddot,S)),tFh)+2*prod(matrix<double>(prod(Fhdot,S)),tFhdot)+prod(matrix<double>(prod(Fh,S)),tFhddot);
		vector<double> wrefn(5),wrefndot(5),wrefnddot(5);
		wrefn = wref + prod(Q,wrefh-wref);
		wrefndot = wrefdot + prod(Qdot,wrefh-wref) + prod(Q,wrefhdot-wrefdot);
		wrefnddot = wrefddot + prod(Qddot,wrefh-wref) + 2*prod(Qdot,wrefhdot-wrefdot) + prod(Q,wrefhddot-wrefddot);

		vector<double> en(5),endot(5);
		en = wrefn - w;
		endot = wrefndot - wdot;
		vector<double> eFn(5),eFndot(5);
		eFn = prod(tFh,en);
		eFndot = prod(tFhdot,en) + prod(tFh,endot);

		vector<double> uf(5);
		uf = wrefnddot + prod(Fh,prod(kvl,eFndot) + prod(kpl,eFn) + 2*prod(tFhdot,endot) + prod(tFhddot,en)) - prod(Jdot,qdot);
		uf = prod(prod(mass,invJ),uf) + prod(c,qdot);
		force = uf;

		// orientation
		vector<double> torque(2),ur(2);
		double epo,phi_t,phi_td;
		//ori(time,q,qdot,qddot,qref,qrefdot,qrefddot,torque,epo);
		phi_td = oritd(time,w,wdot,wrefh,wrefhdot,wrefhddot,ur,phi_t);
		phi_td = oritd(time,w,wdot,wrefn,wrefndot,wrefnddot,ur,phi_t);
		torque = ur;
		//torque(0) = force(4);
		//torque(1) = force(3);

		// calculation control value(sync)
		vector<double> es(2),esdot(es),
					   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
		poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
		poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
		rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
		rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
		es = poss - rposs;
		esdot = vels - rvels;
		forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);

		
		forceall(0) = force(0);
		forceall(1) = force(1);
		forceall(2) = force(1) + forces(0);
		forceall(3) = force(2);
		forceall(4) = torque(1);
		forceall(5) = torque(0);
		forceall(6) = torque(0) + forces(1);

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);
		
		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// loging data
		qref_log(0) = qref(0);
		qref_log(1) = qref(1);
		qref_log(2) = rposs(0);
		qref_log(3) = qref(2);
		qref_log(4) = qref(3);
		qref_log(5) = qref(4);
		qref_log(6) = rposs(1);
		qrefdot_log(0) = qrefdot(0);
		qrefdot_log(1) = qrefdot(1);
		qrefdot_log(2) = rvels(0);
		qrefdot_log(3) = qrefdot(2);
		qrefdot_log(4) = qrefdot(3);
		qrefdot_log(5) = qrefdot(4);
		qrefdot_log(6) = rvels(1);
		qrefddot_log(0) = qrefddot(0);
		qrefddot_log(1) = qrefddot(1);
		qrefddot_log(2) = raccs(0);
		qrefddot_log(3) = qrefddot(2);
		qrefddot_log(4) = qrefddot(3);
		qrefddot_log(5) = qrefddot(4);
		qrefddot_log(6) = raccs(1);
		eq_log(0) = eq(0);
		eq_log(1) = eq(1);
		eq_log(2) = posall(2) - qref(1);		//
		eq_log(3) = eq(2);
		eq_log(4) = eq(3);
		eq_log(5) = eq(4);
		eq_log(6) = posall(6) - qref(4);		//
		force_log = forceall;
		vector<double> w_log(7);
		w_log = posall;
		w_log(0) = w(0);
		w_log(1) = w(1);
		w_log(3) = w(2);
		datalog[0,step] = time;
		datalog[1,step] = 0;
		datalog[2,step] = sqrt(eF(1)*eF(1)+eF(2)*eF(2));
		datalog[3,step] = sqrt(eFn(1)*eFn(1)+eFn(2)*eFn(2));			//ec-enmod
		datalog[4,step] = td;			//td
		datalog[5,step] = eF(0);			//el t
		datalog[6,step] = eF(1);			//el n
		datalog[7,step] = eF(2);			//el b
		datalog[8,step] = posall(2) - posall(1);	//es y
		datalog[9,step] = posall(6) - posall(5);	//es a
		for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
			datalog[j+10,step] = qref_log(i);
			datalog[j+11,step] = qrefdot_log(i);
			datalog[j+12,step] = qrefddot_log(i);
			//datalog[j+13,step] = posall(i);
			datalog[j+13,step] = w_log(i);
			datalog[j+14,step] = velall(i);
			datalog[j+15,step] = accall(i);
			datalog[j+16,step] = force_log(i);
			datalog[j+17,step] = eq_log(i);
		}
		datalog[66,step] = phi_t;
		datalog[67,step] = phi_td;
	}
}

void control::IndependentCon5(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
	double time;

	using namespace control::conparam;


	time = step * SAMPLING_TIME;
		
	//x y1 y2 z c a1 a2	::	x y1 z c a1
	//0 1  2  3 4 5  6	::	0 1  2 3 4

	// get state value(M-system)
	vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
	iocda::GetStateVariable(time,CntId,posall,velall,accall);

	// set state value(M-system)
	vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
	SetStateVal(q,qdot,qddot,posall,velall,accall);

	// M-systemの状態量をP-systemに変換(q -> w)
	// 位置情報の変換行列の計算
	matrix<double> T(5,5),TT(5,2);
	CalcConvertMatrixT(T,TT,q);
	// オフセットの用意
	vector<double> offset(2);
	double a2 = OFFSET_BETWEEN_COORDINATE,
			d4 = DISTANCE_FROM_A_TO_C_AXIS;
	offset(0) = d4;
	offset(1) = a2;
	// M-systemの位置情報をP-systemに変換
	w = prod(T,q) + prod(TT,offset);
	// 速度情報の変換行列の計算
	matrix<double> J(5,5),Jdot(J),invJ(J);
	CalcJ(J,q);
	math::invert(J,invJ);
	// M-systemの速度情報をP-systemに変換
	wdot = prod(J,qdot);
	// 加速度情報の変換行列の計算
	CalcJdot(q,qdot,Jdot);
	// M-systemの加速度情報をP-systemに変換
	wddot = prod(J,qddot) + prod(Jdot,qdot);
		

	// get reference(P-system)
	vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
	GenerateWReference(time,wref,wrefdot,wrefddot);
		
	// P-systemの目標値をM-systemに変換(wref -> qref)
	// 位置情報の変換行列の計算
	matrix<double> Tref(5,5),TTref(5,2),invTref;
	CalcConvertMatrixT(Tref,TTref,wref);
	math::invert(Tref,invTref);
	// P-systemの位置情報をM-systemに変換
	qref = prod( invTref,wref - prod(TTref,offset) );
	// 速度情報の変換行列の計算
	matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
	CalcJ(Jref,qref);
	math::invert(Jref,invJref);
	// P-systemの速度情報をM-systemに変換
	qrefdot = prod(invJref,wrefdot);
	// 加速度情報の変換行列の計算
	CalcJdot(qref,qrefdot,Jrefdot);
	// P-systemの加速度情報をM-systemに変換
	qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));		

	// calculation tracking error
	// M-systemでの誤差（追従誤差）
	vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
	eq = q - qref;
	eqdot = qdot- qrefdot;
	eqddot = qddot - qrefddot;

	force = prod(mass,qrefddot - prod(kvw,eqdot) - prod(kpw,eq)) + prod(c,qdot);

	// calculation control value(sync)
	vector<double> es(2),esdot(es),forces(es),
					poss(es),vels(es),accs(es),
					rposs(es),rvels(es),raccs(es);
	poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
	poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
	rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
	rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
	es = poss - rposs;
	esdot = vels - rvels;
	forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);

	OutputConVal(forceall,force,forces,AioId);

	// loging data
	qref_log(0) = qref(0);
	qref_log(1) = qref(1);
	qref_log(2) = rposs(0);
	qref_log(3) = qref(2);
	qref_log(4) = qref(3);
	qref_log(5) = qref(4);
	qref_log(6) = rposs(1);
	qrefdot_log(0) = qrefdot(0);
	qrefdot_log(1) = qrefdot(1);
	qrefdot_log(2) = rvels(0);
	qrefdot_log(3) = qrefdot(2);
	qrefdot_log(4) = qrefdot(3);
	qrefdot_log(5) = qrefdot(4);
	qrefdot_log(6) = rvels(1);
	qrefddot_log(0) = qrefddot(0);
	qrefddot_log(1) = qrefddot(1);
	qrefddot_log(2) = raccs(0);
	qrefddot_log(3) = qrefddot(2);
	qrefddot_log(4) = qrefddot(3);
	qrefddot_log(5) = qrefddot(4);
	qrefddot_log(6) = raccs(1);
	eq_log(0) = eq(0);
	eq_log(1) = eq(1);
	eq_log(2) = posall(2) - qref(1);		//
	eq_log(3) = eq(2);
	eq_log(4) = eq(3);
	eq_log(5) = eq(4);
	eq_log(6) = posall(6) - qref(4);		//
	force_log = forceall;
	vector<double> w_log(7);
	w_log = posall;
	w_log(0) = w(0);
	w_log(1) = w(1);
	w_log(3) = w(2);
	datalog[0,step] = time;
	datalog[1,step] = 0;			//
	datalog[2,step] = 0;			//
	datalog[3,step] = 0;			//
	datalog[4,step] = 0;			// td
	datalog[5,step] = 0;			// error t
	datalog[6,step] = 0;			// error n
	datalog[7,step] = 0;			// error b
	datalog[8,step] = posall(2) - posall(1);	// es y
	datalog[9,step] = posall(6) - posall(5);	// es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = qref_log(i);
		datalog[j+11,step] = qrefdot_log(i);
		datalog[j+12,step] = qrefddot_log(i);
		//datalog[j+13,step] = posall(i);
		datalog[j+13,step] = w_log(i);
		datalog[j+14,step] = velall(i);
		datalog[j+15,step] = accall(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = eq_log(i);
	}
}

void control::ContouringCon5(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
	double time;

	volatile double d[10];

	using namespace control::conparam;



	time = step * SAMPLING_TIME;
		
	//x y1 y2 z c a1 a2	::	x y1 z c a1
	//0 1  2  3 4 5  6	::	0 1  2 3 4

	// get state value(M-system)
	vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
	iocda::GetStateVariable(time,CntId,posall,velall,accall);

	// set state value(M-system)
	vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
	SetStateVal(q,qdot,qddot,posall,velall,accall);
		

	// M-systemの状態量をP-systemに変換(q -> w)
	// 位置情報の変換行列の計算
	matrix<double> T(5,5),TT(5,2);
	CalcConvertMatrixT(T,TT,q);
	// オフセットの用意
	vector<double> offset(2);
	double a2 = OFFSET_BETWEEN_COORDINATE,
			d4 = DISTANCE_FROM_A_TO_C_AXIS;
	offset(0) = d4;
	offset(1) = a2;
	// M-systemの位置情報をP-systemに変換
	w = prod(T,q) + prod(TT,offset);
	// 速度情報の変換行列の計算
	matrix<double> J(5,5),Jdot(J),invJ(J);
	CalcJ(J,q);
	math::invert(J,invJ);
	// M-systemの速度情報をP-systemに変換
	wdot = prod(J,qdot);
	// 加速度情報の変換行列の計算
	CalcJdot(q,qdot,Jdot);
	// M-systemの加速度情報をP-systemに変換
	wddot = prod(J,qddot) + prod(Jdot,qdot);
		

	// get reference(P-system)
	vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
	GenerateWReference(time,wref,wrefdot,wrefddot);
		
	// P-systemの目標値をM-systemに変換(wref -> qref)
	// 位置情報の変換行列の計算
	matrix<double> Tref(5,5),TTref(5,2),invTref;
	CalcConvertMatrixT(Tref,TTref,wref);
	math::invert(Tref,invTref);
	// P-systemの位置情報をM-systemに変換
	qref = prod( invTref,wref - prod(TTref,offset) );
	// 速度情報の変換行列の計算
	matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
	CalcJ(Jref,qref);
	math::invert(Jref,invJref);
	// P-systemの速度情報をM-systemに変換
	qrefdot = prod(invJref,wrefdot);
	// 加速度情報の変換行列の計算
	CalcJdot(qref,qrefdot,Jrefdot);
	// P-systemの加速度情報をM-systemに変換
	qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));		

	// calculation tracking error
	// M-systemでの誤差（追従誤差）
	vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
	eq = q - qref;
	eqdot = qdot- qrefdot;
	eqddot = qddot - qrefddot;

	// P-systemでの誤差
	vector<double>  e(NUM_DOF),edot(e),eddot(e);
	e = wref - w;
	edot = wrefdot - wdot;
	eddot = wrefddot - wddot;

	matrix<double> F(5,5),Fdot(F),Fddot(F),tF,tFdot,tFddot;
	static matrix<double> oldF(5,5,0),oldFdot(5,5,0);
	if(time == 0)
		oldF.clear(),oldFdot.clear();
	CalcFrenetFrame(wrefdot,wrefddot,F);	// get Frenet frame(current)
	Fdot = (F - oldF) / SAMPLING_TIME;
	//vector<double> w3(5);
	//GenerateWReference3dot(time,w3);
	//CalcFrenetFramedot(wrefdot,wrefddot,w3,Fdot);
	oldF = F;
	Fddot = (Fdot - oldFdot) / SAMPLING_TIME;
	//vector<double> w4(5);
	//GenerateWReference4dot(time,w4);
	//CalcFrenetFrameddot(wrefdot,wrefddot,w3,w4,Fddot);
	oldFdot = Fdot;
	tF = trans(F);
	tFdot = trans(Fdot);
	tFddot = trans(Fddot);
		
	vector<double> eF(NUM_DOF),eFdot(eF);
	eF = prod(tF,e);
	eFdot = prod(tFdot,e) + prod(tF,edot);


	force = wrefddot + prod(F,prod(kvl,eFdot) + prod(kpl,eF) + 2*prod(tFdot,edot) + prod(tFddot,e)) - prod(Jdot,qdot);
	force = prod(prod(mass,invJ),force) + prod(c,qdot);


	// orientation
	vector<double> torque(2);
	double epo;
	ori(time,q,qdot,qddot,qref,qrefdot,qrefddot,torque,epo);


	// calculation control value(sync)
	vector<double> es(2),esdot(es),
					poss(es),vels(es),accs(es),rposs(es),
					rvels(es),raccs(es),forces(es);
	poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
	poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
	rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
	rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
	es = poss - rposs;
	esdot = vels - rvels;
	forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);
	
	force(3) = torque(1);
	force(4) = torque(0);
	OutputConVal(forceall,force,forces,AioId);		

	// loging data
	qref_log(0) = qref(0);
	qref_log(1) = qref(1);
	qref_log(2) = rposs(0);
	qref_log(3) = qref(2);
	qref_log(4) = qref(3);
	qref_log(5) = qref(4);
	qref_log(6) = rposs(1);
	qrefdot_log(0) = qrefdot(0);
	qrefdot_log(1) = qrefdot(1);
	qrefdot_log(2) = rvels(0);
	qrefdot_log(3) = qrefdot(2);
	qrefdot_log(4) = qrefdot(3);
	qrefdot_log(5) = qrefdot(4);
	qrefdot_log(6) = rvels(1);
	qrefddot_log(0) = qrefddot(0);
	qrefddot_log(1) = qrefddot(1);
	qrefddot_log(2) = raccs(0);
	qrefddot_log(3) = qrefddot(2);
	qrefddot_log(4) = qrefddot(3);
	qrefddot_log(5) = qrefddot(4);
	qrefddot_log(6) = raccs(1);
	eq_log(0) = eq(0);
	eq_log(1) = eq(1);
	eq_log(2) = posall(2) - qref(1);		//
	eq_log(3) = eq(2);
	eq_log(4) = eq(3);
	eq_log(5) = eq(4);
	eq_log(6) = posall(6) - qref(4);		//
	force_log = forceall;
	vector<double> w_log(7);
	w_log = posall;
	w_log(0) = w(0);
	w_log(1) = w(1);
	w_log(3) = w(2);
	datalog[0,step] = time;
	datalog[1,step] = 0;			//
	datalog[2,step] = sqrt(eF(1)*eF(1)+eF(2)*eF(2));			//
	datalog[3,step] = 0;			//
	datalog[4,step] = 0;			// td
	datalog[5,step] = eF(0);			// error t
	datalog[6,step] = eF(1);			// error n
	datalog[7,step] = eF(2);			// error b
	datalog[8,step] = posall(2) - posall(1);	// es y
	datalog[9,step] = posall(6) - posall(5);	// es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = qref_log(i);
		datalog[j+11,step] = qrefdot_log(i);
		datalog[j+12,step] = qrefddot_log(i);
		//datalog[j+13,step] = posall(i);
		datalog[j+13,step] = w_log(i);
		datalog[j+14,step] = velall(i);
		datalog[j+15,step] = accall(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = eq_log(i);
	}
}

void control::modContouringCon5(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> eq_log(NUM_ACTUATOR),qref_log(NUM_ACTUATOR),qrefdot_log(qref_log),qrefddot_log(qref_log),
				   force(NUM_ACTUATOR),forceall(NUM_ACTUATOR),force_log(forceall);
	double time;

	volatile double d[10];

	using namespace control::conparam;


	time = step * SAMPLING_TIME;
		
	//x y1 y2 z c a1 a2
	//0 1  2  3 4 5  6

	// get state value(M-system)
	vector<double> posall(NUM_ACTUATOR),velall(posall),accall(posall);
	iocda::GetStateVariable(time,CntId,posall,velall,accall);

	// set state value(M-system)
	vector<double> w(NUM_DOF),wdot(w),wddot(w),q(NUM_DOF),qdot(q),qddot(q);
	SetStateVal(q,qdot,qddot,posall,velall,accall);

	// M-systemの状態量をP-systemに変換(q -> w)
	// 位置情報の変換行列の計算
	matrix<double> T(5,5),TT(5,2);
	CalcConvertMatrixT(T,TT,q);
	// オフセットの用意
	vector<double> offset(2);
	double a2 = OFFSET_BETWEEN_COORDINATE,
			d4 = DISTANCE_FROM_A_TO_C_AXIS;
	offset(0) = d4;
	offset(1) = a2;
	// M-systemの位置情報をP-systemに変換
	w = prod(T,q) + prod(TT,offset);
	// 速度情報の変換行列の計算
	matrix<double> J(5,5),Jdot(J),invJ(J);
	CalcJ(J,q);
	math::invert(J,invJ);
	// M-systemの速度情報をP-systemに変換
	wdot = prod(J,qdot);
	// 加速度情報の変換行列の計算
	CalcJdot(q,qdot,Jdot);
	// M-systemの加速度情報をP-systemに変換
	wddot = prod(J,qddot) + prod(Jdot,qdot);
		

	// get reference(P-system)
	vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref),qref(NUM_DOF),qrefdot(qref),qrefddot(qref);
	GenerateWReference(time,wref,wrefdot,wrefddot);
		
	// P-systemの目標値をM-systemに変換(wref -> qref)
	// 位置情報の変換行列の計算
	matrix<double> Tref(5,5),TTref(5,2),invTref;
	CalcConvertMatrixT(Tref,TTref,wref);
	math::invert(Tref,invTref);
	// P-systemの位置情報をM-systemに変換
	qref = prod( invTref,wref - prod(TTref,offset) );
	// 速度情報の変換行列の計算
	matrix<double> Jref(5,5),Jrefdot(Jref),invJref(Jref);
	CalcJ(Jref,qref);
	math::invert(Jref,invJref);
	// P-systemの速度情報をM-systemに変換
	qrefdot = prod(invJref,wrefdot);
	// 加速度情報の変換行列の計算
	CalcJdot(qref,qrefdot,Jrefdot);
	// P-systemの加速度情報をM-systemに変換
	qrefddot = prod(invJref,wrefddot - prod(Jrefdot,qrefdot));		

	// calculation tracking error
	// M-systemでの誤差（追従誤差）
	vector<double> eq(NUM_DOF),eqdot(eq),eqddot(eq);
	eq = q - qref;
	eqdot = qdot- qrefdot;
	eqddot = qddot - qrefddot;

	// P-systemでの誤差
	vector<double>  e(NUM_DOF),edot(e),eddot(e);
	e = wref - w;
	edot = wrefdot - wdot;
	eddot = wrefddot - wddot;

	matrix<double> F(5,5),Fdot(F),Fddot(F),tF,tFdot,tFddot;
	static matrix<double> oldF(5,5,0),oldFdot(5,5,0);
	if(time == 0)
		oldF.clear(),oldFdot.clear();
	CalcFrenetFrame(wrefdot,wrefddot,F);	// get Frenet frame(current)
	Fdot = (F - oldF) / SAMPLING_TIME;
	//vector<double> w3(5);
	//GenerateWReference3dot(time,w3);
	//CalcFrenetFramedot(wrefdot,wrefddot,w3,Fdot);
	oldF = F;
	Fddot = (Fdot - oldFdot) / SAMPLING_TIME;
	//vector<double> w4(5);
	//GenerateWReference4dot(time,w4);
	//CalcFrenetFrameddot(wrefdot,wrefddot,w3,w4,Fddot);
	oldFdot = Fdot;
	tF = trans(F);
	tFdot = trans(Fdot);
	tFddot = trans(Fddot);
		
	vector<double> eF(NUM_DOF),eFdot(eF);
	eF = prod(tF,e);
	eFdot = prod(tFdot,e) + prod(tF,edot);


		
	vector<double> Prefdot(3);
	Prefdot(0) = wrefdot(0);
	Prefdot(1) = wrefdot(1);
	Prefdot(2) = wrefdot(2);
	double td,tddot,tdddot;
	static double oldtd,oldtddot;
	td = eF(0) / norm_2(Prefdot);
	tddot = (td-oldtd) / SAMPLING_TIME;
	tdddot = (tddot-oldtddot) / SAMPLING_TIME;
	oldtd = td;
	oldtddot = tddot;

	// get reference
	vector<double> wrefh(5),wrefhdot(5),wrefhddot(5);
	double thcrefh,tha1refh,thcrefhdot,tha1refhdot,thcrefhddot,tha1refhddot;
	GenerateWReference(time-td,wrefh,wrefhdot,wrefhddot);
	//wrefhddot = -tdddot*wrefhdot + (1-tddot)*wrefhddot;
	//wrefhdot = (1-tddot)*wrefhdot;
		

	matrix<double> Fh(5,5),Fhdot(Fh),Fhddot(Fh),tFh,tFhdot,tFhddot;
	vector<double> w3(5),w4(w3);
	CalcFrenetFrame(wrefhdot,wrefhddot,Fh);	// get Frenet frame(delay)
	GenerateWReference3dot(time-td,w3);
	CalcFrenetFramedot(wrefhdot,wrefhddot,w3,Fhdot);
	GenerateWReference3dot(time-td,w4);
	CalcFrenetFrameddot(wrefhdot,wrefhddot,w3,w4,Fh,Fhdot,Fhddot);
	/*
	static matrix<double> oldFh(5,5,0),oldFhdot(5,5,0);
	Fhdot = (Fh - oldFh) / SAMPLING_TIME;
	oldFh = Fh;
	Fhddot = (Fhdot - oldFhdot) / SAMPLING_TIME;
	oldFhdot = Fhdot;
	//*/
	tFh = trans(Fh);
	tFhdot = trans(Fhdot);
	tFhddot = trans(Fhddot);

	vector<double> h(5),hdot(h),
					hF(5),hFdot(hF),
					hFh(5),hFhdot(hFh),hFhddot(hFh);
	h(0) = wref(0) - wrefh(0);
	h(1) = wref(1) - wrefh(1);
	h(2) = wref(2) - wrefh(2);
	h(3) = 0;
	h(4) = 0;
	hdot(0) = wrefdot(0) - wrefhdot(0);
	hdot(1) = wrefdot(1) - wrefhdot(1);
	hdot(2) = wrefdot(2) - wrefhdot(2);
	hdot(3) = 0;
	hdot(4) = 0;

	hF = prod(tFh,h);
	hFdot = prod(tFhdot,h) + prod(tFh,hdot);
	matrix<double> W(5,5,0);
	vector<double> eFh(5),eFhdot(eFh);
	W(1,1) = -1;
	W(2,2) = -1;
	hFh = prod(W,hF);
	hFhdot = prod(W,hFdot);
	eFh = prod(tFh,e) + hFh;
	eFhdot = prod(tFhdot,e) + prod(tFh,edot) + hFhdot;

	vector<double> uf(5);
	uf = wrefddot + prod(Fh,prod(kvl,eFhdot) + prod(kpl,eFh) + hFhddot + 2*prod(tFhdot,edot) + prod(tFhddot,e)) - prod(Jdot,qdot);
	uf = prod(prod(mass,invJ),uf) + prod(c,qdot);
		
	force = wrefddot + prod(F,prod(kvl,eFdot) + prod(kpl,eF) + 2*prod(tFdot,edot) + prod(tFddot,e)) - prod(Jdot,qdot);
	force = prod(prod(mass,invJ),force) + prod(c,qdot);
	force = uf;


	// orientation
	vector<double> torque(2),ur(2);
	double epo,phi_t,phi_td;
	//ori(time-td,q,qdot,qddot,qref,qrefdot,qrefddot,ur,epo);
	ori(time,q,qdot,qddot,qref,qrefdot,qrefddot,torque,epo);
	phi_td = oritd(time,q,qdot,wrefh,wrefhdot,wrefhddot,ur,phi_t);
	torque = ur;

	// calculation control value(sync)
	vector<double> es(2),esdot(es),
					poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);
	poss(0) = posall(2);	vels(0) = velall(2);	accs(0) = accall(2);// Y2
	poss(1) = posall(6);	vels(1) = velall(6);	accs(1) = accall(6);// A2
	rposs(0) = q(1);		rvels(0) = qdot(1);		raccs(0) = qddot(1);// Y2
	rposs(1) = q(4);		rvels(1) = qdot(4);		raccs(1) = qddot(4);// A2
	es = poss - rposs;
	esdot = vels - rvels;
	forces = prod(masss,(-prod(kvs,esdot)-prod(kps,es))) + prod(cs,esdot);


	force(3) = torque(1);
	force(4) = torque(0);
	OutputConVal(forceall,force,forces,AioId);

	// loging data
	qref_log(0) = qref(0);
	qref_log(1) = qref(1);
	qref_log(2) = rposs(0);
	qref_log(3) = qref(2);
	qref_log(4) = qref(3);
	qref_log(5) = qref(4);
	qref_log(6) = rposs(1);
	qrefdot_log(0) = qrefdot(0);
	qrefdot_log(1) = qrefdot(1);
	qrefdot_log(2) = rvels(0);
	qrefdot_log(3) = qrefdot(2);
	qrefdot_log(4) = qrefdot(3);
	qrefdot_log(5) = qrefdot(4);
	qrefdot_log(6) = rvels(1);
	qrefddot_log(0) = qrefddot(0);
	qrefddot_log(1) = qrefddot(1);
	qrefddot_log(2) = raccs(0);
	qrefddot_log(3) = qrefddot(2);
	qrefddot_log(4) = qrefddot(3);
	qrefddot_log(5) = qrefddot(4);
	qrefddot_log(6) = raccs(1);
	eq_log(0) = eq(0);
	eq_log(1) = eq(1);
	eq_log(2) = posall(2) - qref(1);		//
	eq_log(3) = eq(2);
	eq_log(4) = eq(3);
	eq_log(5) = eq(4);
	eq_log(6) = posall(6) - qref(4);		//
	force_log = forceall;
	vector<double> w_log(7);
	w_log = posall;
	w_log(0) = w(0);
	w_log(1) = w(1);
	w_log(3) = w(2);
	datalog[0,step] = time;
	datalog[1,step] = 0;			//
	datalog[2,step] = sqrt(eF(1)*eF(1)+eF(2)*eF(2));			//
	datalog[3,step] = sqrt(eFh(1)*eFh(1)+eFh(2)*eFh(2));		//
	datalog[4,step] = td;				// td
	datalog[5,step] = eF(0);			// error t
	datalog[6,step] = eF(1);			// error n
	datalog[7,step] = eF(2);			// error b
	datalog[8,step] = posall(2) - posall(1);	// es y
	datalog[9,step] = posall(6) - posall(5);	// es a
	for(i=0,j=0;i<NUM_ACTUATOR;i++,j+=8){
		datalog[j+10,step] = qref_log(i);
		datalog[j+11,step] = qrefdot_log(i);
		datalog[j+12,step] = qrefddot_log(i);
		//datalog[j+13,step] = posall(i);
		datalog[j+13,step] = w_log(i);
		datalog[j+14,step] = velall(i);
		datalog[j+15,step] = accall(i);
		datalog[j+16,step] = force_log(i);
		datalog[j+17,step] = eq_log(i);
	}
}

void control::Identification(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	vector<double> ew(NUM_DOF),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR),
				   pos(NUM_DOF),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(NUM_DOF),rvel(rpos),racc(rpos),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   force(NUM_DOF),forceall(NUM_ACTUATOR),force_log(forceall);
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),rposs(es),rvels(es),raccs(es),forces(es);

	double time;

	using namespace control::conparam;


	time = SAMPLING_TIME * step;

	// get state value
	iocda::GetStateVariable(time,CntId,posall,velall,accall);
	pos(0) = posall(0);
	pos(1) = posall(1);
	pos(2) = posall(3);
	pos(3) = posall(4);
	pos(4) = posall(5);
	poss(0) = posall(2);
	poss(1) = posall(6);

	vel(0) = velall(0);
	vel(1) = velall(1);
	vel(2) = velall(3);
	vel(3) = velall(4);
	vel(4) = velall(5);
	vels(0) = velall(2);
	vels(1) = velall(6);

	acc(0) = accall(0);
	acc(1) = accall(1);
	acc(2) = accall(3);
	acc(3) = accall(4);
	acc(4) = accall(5);
	accs(0) = accall(2);
	accs(1) = accall(6);

	rpos(0) = 0.01*sin(time*OMG);
	double e = rpos(0) - posall(3);
	forceall.clear();
	forceall(3) = e*12000;
	//forceall(3) = 7.0*sin(time*OMG);
	//forceall(5) = 0.3*sin(time*OMG);
	//forceall(6) = -forceall(5);
	

	// output control value
	iocda::OutputControlVariable(AioId,forceall);

	
	// loging data
	force_log = forceall;
	force_log(2) *= -1;
	force_log(3) *= -1;
	force_log(4) *= -1;
	force_log(6) *= -1;
	datalog[0,step] = time;
	datalog[1,step] = 0;
	datalog[2,step] = 0;	//ec-en
	datalog[3,step] = 0;	//ec-enmod
	datalog[4,step] = 0;	//td
	datalog[5,step] = 0;	//el t
	datalog[6,step] = 0;	//el n
	datalog[7,step] = 0;	//el b
	datalog[8,step] = posall(2) - posall(1);	//es y
	datalog[9,step] = posall(5) - posall(6);	//es a
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

/*void IndCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog)
{
	int i,j;
	unsigned int step;
	PLANT plant;
	vector<double> ew(2),ew1(ew),ew2(ew),ew_log(NUM_ACTUATOR,0),
				   pos(2),vel(pos),acc(pos),
				   posall(NUM_ACTUATOR),velall(posall),accall(posall),
				   rpos(2),rvel(rpos),racc(rpos),
				   rpos_log(NUM_ACTUATOR),rvel_log(rpos_log),racc_log(rpos_log),
				   uc(2),forceall(NUM_ACTUATOR,0),force_log(forceall);
	vector<double> es(2),es1(es),es2(es),
				   poss(es),vels(es),accs(es),
				   rposs(es),rvels(es),raccs(es),
				   forces(es);
	double time,ec;

	using namespace control::conparam;

	posall.clear();
	velall.clear();
	accall.clear();
	
	for(step=0;step<max_step;step++)
	{
		time = step * SAMPLING_TIME;
		
		// get state value
		vector<double>	pos(2),vel(pos),acc(pos),
						posall(NUM_ACTUATOR),velall(posall),accall(posall);
		iocda::GetStateVariable(time,plant,enable_resolution,posall,velall,accall);
		pos(0) = posall(0);
		pos(1) = posall(1);
		vel(0) = velall(0);
		vel(1) = velall(1);
		acc(0) = accall(0);
		acc(1) = accall(1);

		// get reference
		vector<double>	rpos(2),rvel(rpos),racc(rpos),
						rposall(NUM_DOF),rvelall(rposall),raccall(rposall);
		GenerateReference(time,rposall,rvelall,raccall);
		rpos(0) = rposall(0);
		rpos(1) = rposall(1);
		rvel(0) = rvelall(0);
		rvel(1) = rvelall(1);
		racc(0) = raccall(0);
		racc(1) = raccall(1);

		// calculation tracking error
		vector<double>	ew(2),ew1(ew);
		ew = pos - rpos;
		ew1 = vel - rvel;

		// calculation control value
		matrix<double>	M(2,2,0),C(2,2,0),
						kvw(2,2,0),kpw(2,2,0);
		vector<double>	uc(2);
		uc = prod(M,(racc-prod(kvw,ew1)-prod(kpw,ew))) + prod(C,vel);

		// calculation control value(sync)
		double	posy1 = pos(1),
				posy2 = posall(2),
				vely1 = vel(1),
				vely2 = velall(2);
		double Ms,Cs,kvs,kps,es,es1,us;
		es = posy2 - posy1;
		es1 = vely2 - vely1;
		us = (Ms * (-kvs*es1 - kps*es)) + Cs * es1;

		// output control value
		iocda::OutputControlVariable(enable_resolution,forceall);

		// input control value to plantmodel
		plant.plantmodel(time,forceall,enable_noise);

		// calculation contouring error
		ec = CalcContouringError(pos);

		// loging data
		rpos_log(0) = rpos(0);
		rpos_log(1) = rpos(1);
		rpos_log(2) = rposs(0);
		rpos_log(3) = rpos(2);
		rpos_log(4) = rpos(3);
		rpos_log(5) = rpos(4);
		rpos_log(6) = rposs(1);
		rvel_log(0) = rvel(0);
		rvel_log(1) = rvel(1);
		rvel_log(2) = rvels(0);
		rvel_log(3) = rvel(2);
		rvel_log(4) = rvel(3);
		rvel_log(5) = rvel(4);
		rvel_log(6) = rvels(1);
		racc_log(0) = racc(0);
		racc_log(1) = racc(1);
		racc_log(2) = raccs(0);
		racc_log(3) = racc(2);
		racc_log(4) = racc(3);
		racc_log(5) = racc(4);
		racc_log(6) = raccs(1);
		ew_log(0) = ew(0);
		ew_log(1) = ew(1);
		ew_log(2) = poss(0) - rpos(1);//es(0);
		ew_log(3) = ew(2);
		ew_log(4) = ew(3);
		ew_log(5) = ew(4);
		ew_log(6) = poss(1) - rpos(4);//es(1);
		force_log = forceall;
		datalog[0,step] = time;
		datalog[1,step] = ec;
		datalog[2,step] = 0;	//ec-en
		datalog[3,step] = 0;	//ec-enmod
		datalog[4,step] = 0;	//td
		datalog[5,step] = 0;	//el t
		datalog[6,step] = 0;	//el n
		datalog[7,step] = 0;	//el b
		datalog[8,step] = poss(0) - pos(1);	//es y
		datalog[9,step] = poss(1) - pos(4);	//es a
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
	}*/