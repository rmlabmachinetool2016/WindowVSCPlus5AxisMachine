#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include "io.h"

namespace Identification{
	void Identification(unsigned int step,short AioId,short CntId,unsigned int selected_axis,array<double>^ param,array<double,2>^ datalog);
	void CalcModelParam(double& M,double& Fri,unsigned int selected_axis,array<double>^ param,array<double,2>^ datalog);
}

#endif
