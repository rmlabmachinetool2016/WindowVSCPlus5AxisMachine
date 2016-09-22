#include "stdafx.h"
#include "plant.h"
#include "math.hpp"
#include <cmath>
#include <boost/random.hpp>
#include <ctime>


PLANT::cplant()
{
	pos.resize(NUM_ACTUATOR);
	vel.resize(NUM_ACTUATOR);
	acc.resize(NUM_ACTUATOR);
	pos(0) = INIT_POS_X;
	pos(1) = INIT_POS_Y1;
	pos(2) = INIT_POS_Y2;
	pos(3) = INIT_POS_Z;
	pos(4) = INIT_ANGLE_C;
	pos(5) = INIT_ANGLE_A1;
	pos(6) = INIT_ANGLE_A2;
	vel.clear();
	acc.clear();

	mass.resize(NUM_ACTUATOR,NUM_ACTUATOR);
	mass(0,0)=MMASS_X;mass(1,1)=MMASS_Y1;mass(2,2)=MMASS_Y2;mass(3,3)=MMASS_Z;
	mass(4,4)=MINERTIA_C;mass(5,5)=MINERTIA_A1;mass(6,6)=MINERTIA_A2;
	math::invert(mass,invmass);

	c.resize(NUM_ACTUATOR,NUM_ACTUATOR);
	c(0,0)=MFRICTION_X;c(1,1)=MFRICTION_Y1;c(2,2)=MFRICTION_Y2;c(3,3)=MFRICTION_Z;
	c(4,4)=MFRICTION_C;c(5,5)=MFRICTION_A1;c(6,6)=MFRICTION_A2;
	math::invert(c,invc);

	// init noise(Mersenne twister)
	noise.resize(NUM_ACTUATOR,MAX_COUNT);
	boost::mt19937 seed(static_cast<unsigned long>(time(NULL)));
	boost::uniform_real<> range(-1,1);
	boost::variate_generator< boost::mt19937, boost::uniform_real<> > rand( seed,range );
	for(int i=0;i<NUM_ACTUATOR;i++)
		for(int j=0;j<MAX_COUNT;j++)
			noise(i,j) = rand() * NOISE_GAIN;
}

template <class T>
struct exponential : scalar_unary_functor<T> {
    static T apply(T t) { return exp(t); }
};

void PLANT::plantmodel(double time,vector<double>& force,bool enable_noise)
{
	vector<double> A(NUM_ACTUATOR),B(A);
	diagonal_matrix<double> CiM,E;

	A = prod(prod(mass,invc),prod(invc,force)-vel);
	B = pos - A;
	E = numeric::ublas::apply_to_all<exponential<double>>(-prod(c,invmass)*SAMPLING_TIME);
	CiM = prod(c,invmass);
		
	pos = prod(E,A) + B + prod(invc,force)*SAMPLING_TIME;
	vel = -prod((prod(CiM,E)),A) + prod(invc,force);
	acc = prod(prod(matrix<double>(prod(CiM,CiM)),E),A);
}

void PLANT::getstate(vector<double>& refpos,vector<double>& refvel,vector<double>& refacc)
{
	refpos = pos;
	refvel = vel;
	refacc = acc;
}
