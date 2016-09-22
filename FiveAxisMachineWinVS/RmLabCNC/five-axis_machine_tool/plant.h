#ifndef _PLANT_H_
#define _PLANT_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/banded.hpp>
#include "define.h"

using namespace boost::numeric::ublas;  // boost::numeric::ublas
using namespace define;


namespace plant{
	// model parameter of machine tool
	static const double MMASS_X			= 5.33;		// kg
	static const double MMASS_Y1		= 4.545;	// kg
	static const double MMASS_Y2		= 4.545;	// kg
	static const double MMASS_Z			= 1.72;		// kg
	static const double MINERTIA_C		= 0.00342;	// kg*m^2
	static const double MINERTIA_A1		= 0.00956;	// kg*m^2
	static const double MINERTIA_A2		= 0.00956;	// kg*m^2
	static const double MFRICTION_X		= 25.175;	// N*sec/m
	static const double MFRICTION_Y1	= 24.202;	// N*sec/m
	static const double MFRICTION_Y2	= 24.202;	// N*sec/m
	static const double MFRICTION_Z		= 71.647;	// N*sec/m
	static const double MFRICTION_C		= 0.022;	// N*sec*m
	static const double MFRICTION_A1	= 0.100;	// N*sec*m
	static const double MFRICTION_A2	= 0.100;	// N*sec*m

	static const double NOISE_GAIN		= OMG*RADIUS*10;

	class cplant{
	private:
		diagonal_matrix<double> mass,c,invmass,invc;
		vector<double> pos,vel,acc;
		matrix<double> noise;
	public:
		cplant();
		void plantmodel(double time,vector<double>& force,bool enable_noise);
		void getstate(vector<double>& pos,vector<double>& vel,vector<double>& acc);
	};
}
typedef plant::cplant PLANT;

#endif