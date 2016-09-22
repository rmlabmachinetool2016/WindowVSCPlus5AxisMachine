#ifndef _IO_H_
#define _IO_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include "define.h"
#include "plant.h"

using namespace boost::numeric::ublas;  // boost::numeric::ublas
using namespace define;

namespace iocda{
	static const unsigned int RESOLUTION_DA	= 4095;
	static const int DA_MAX_VOLT			= 10;
	static const int DA_MIN_VOLT			= -DA_MAX_VOLT;

	void GetStateVariable(double time,PLANT& plant,bool enable_resolution,vector<double>& pos,vector<double>& vel,vector<double>& acc);
	void GetStateVariable(double time,short CntId,vector<double>& pos,vector<double>& vel,vector<double>& acc);
	void OutputControlVariable(short AioId,vector<double>& f);
	void OutputControlVariable(bool enable_resolution,vector<double>& f);
}

#endif