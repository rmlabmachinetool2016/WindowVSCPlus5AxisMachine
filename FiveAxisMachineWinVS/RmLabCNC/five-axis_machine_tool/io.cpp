#include "stdafx.h"
#include "io.h"

using namespace define;


void iocda::GetStateVariable(double time,short CntId,vector<double>& pos,vector<double>& vel,vector<double>& acc)
{
	int i;
	vector<long> cnt(NUM_COUNTER);
	static vector<double> oldpos,oldvel;
	long CntRet,dir[NUM_COUNTER]={-1,1,-1,-1,1,1,-1};
	unsigned long CntDat[NUM_COUNTER];
	short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};
	double cut[NUM_COUNTER] = {CUTOFF_FREC_X,CUTOFF_FREC_Y1,CUTOFF_FREC_Y2,CUTOFF_FREC_Z,CUTOFF_FREC_C,CUTOFF_FREC_A1,CUTOFF_FREC_A2};
	
	if(time == 0){
		oldpos.resize(NUM_COUNTER);
		oldvel.resize(NUM_COUNTER);
		oldpos.clear();
		oldvel.clear();
	}

	// reading values from encorder
	CntRet = CCntCLI::CntReadCount ( CntId , ChNo , NUM_COUNTER , CntDat );

	// convert right-hand system
	for(i=0;i<NUM_COUNTER;i++)
		cnt(i) = static_cast<long>(CntDat[i] * dir[i]);

	// convert position(meter)
	pos(0) = cnt(0) * RESONATE_LINER_ENC_X;
	pos(1) = cnt(1) * RESONATE_LINER_ENC_Y1;
	pos(2) = cnt(2) * RESONATE_LINER_ENC_Y2;
	pos(3) = cnt(3) * RESONATE_LINER_ENC_Z;
	pos(4) = cnt(4) * RESONATE_ROTATION_ENC_C;
	pos(5) = cnt(5) * RESONATE_ROTATION_ENC_A1;
	pos(6) = cnt(6) * RESONATE_ROTATION_ENC_A2;
	
	for(i=0;i<NUM_COUNTER;i++)
	{
		if(cut[i] != 0){
			//*
			// lpf(z convert)
			double a = 2*cut[i] / (cut[i]*SAMPLING_TIME+2),
				   b = (cut[i]*SAMPLING_TIME-2) / (cut[i]*SAMPLING_TIME+2);
			vel(i) = a * (pos(i)-oldpos(i)) - b * oldvel(i);
			//acc(i) = a * (vel(i)-oldvel(i)) - b * oldacc(i);
			acc(i) = (vel(i) - oldvel(i)) / SAMPLING_TIME;
			//*/

			/*
			// lpf(z convert,second)
			matrix<double> A(2,2),I=identity_matrix<double>(A.size1()),A_inv;
			vector<double> X(A.size1()),oldX(X),B(X);
			static matrix<double> oldXs(NUM_ACTUATOR,A.size1());

			if(time==0)
				oldXs.clear();
			A(0,0) = 0;
			A(0,1) = 1;
			A(1,0) = -cut[i] * cut[i];
			A(1,1) = -2 * cut[i];
			B(0) = 0;
			B(1) = cut[i] * cut[i];
			math::invert(I-SAMPLING_TIME/2*A,A_inv);
			oldX(0) = oldXs(i,0);
			oldX(1) = oldXs(i,1);
			X = prod(prod(A_inv,(I+SAMPLING_TIME/2*A)),oldX)+SAMPLING_TIME/2*prod(A_inv,B)*(pos(i)+oldpos(i));
			oldXs(i,0) = X(0);
			oldXs(i,1) = X(1);
			pos(i) = X(0);
			vel(i) = X(1);
			acc(i) = A(1,0)*X(0) + A(1,1)*X(1) + cut[i]*cut[i]*pos(i);
			//*/
		}else{
			// diff
			vel(i) = (pos(i) - oldpos(i)) / SAMPLING_TIME;
			acc(i) = (vel(i) - oldvel(i)) / SAMPLING_TIME;
			if(time == 0){
				vel(i) = 0;
				acc(i) = 0;
			}
			else if(time == SAMPLING_TIME)
				acc(i) = 0;
		}
	}

	oldpos = pos;
	oldvel = vel;
}

void iocda::GetStateVariable(double time,PLANT& plant,bool enable_resolution,vector<double>& pos,vector<double>& vel,vector<double>& acc)
{
	int i;
	vector<short> cnt(NUM_COUNTER);
	vector<double> oldpos(pos),oldvel(vel);
	int temp[NUM_COUNTER];
	double cut[NUM_COUNTER] = {CUTOFF_FREC_X,CUTOFF_FREC_Y1,CUTOFF_FREC_Y2,CUTOFF_FREC_Z,CUTOFF_FREC_C,CUTOFF_FREC_A1,CUTOFF_FREC_A2};

	plant.getstate(pos,vel,acc);

	if(enable_resolution)
	{
		temp[0] = static_cast<int>(pos(0) / RESONATE_LINER_ENC_X);
		temp[1] = static_cast<int>(pos(1) / RESONATE_LINER_ENC_Y1);
		temp[2] = static_cast<int>(pos(2) / RESONATE_LINER_ENC_Y2);
		temp[3] = static_cast<int>(pos(3) / RESONATE_LINER_ENC_Z);
		temp[4] = static_cast<int>(pos(4) / RESONATE_ROTATION_ENC_C);
		temp[5] = static_cast<int>(pos(5) / RESONATE_ROTATION_ENC_A1);
		temp[6] = static_cast<int>(pos(6) / RESONATE_ROTATION_ENC_A2);

		pos(0) = static_cast<double>(temp[0]) * RESONATE_LINER_ENC_X;
		pos(1) = static_cast<double>(temp[1]) * RESONATE_LINER_ENC_Y1;
		pos(2) = static_cast<double>(temp[2]) * RESONATE_LINER_ENC_Y2;
		pos(3) = static_cast<double>(temp[3]) * RESONATE_LINER_ENC_Z;
		pos(4) = static_cast<double>(temp[4]) * RESONATE_ROTATION_ENC_C;
		pos(5) = static_cast<double>(temp[5]) * RESONATE_ROTATION_ENC_A1;
		pos(6) = static_cast<double>(temp[6]) * RESONATE_ROTATION_ENC_A2;
	

		for(i=0;i<NUM_COUNTER;i++)
		{
			if(cut[i] != 0){
				/*
				// lpf(z convert)
				double a = 2*cut[i] / (cut[i]*SAMPLING_TIME+2),
					   b = (cut[i]*SAMPLING_TIME-2) / (cut[i]*SAMPLING_TIME+2);
				vel(i) = a * (pos(i)-oldpos(i)) - b * oldvel(i);
				acc(i) = a * (vel(i)-oldvel(i)) - b * oldacc(i);
				//*/

				// lpf(z convert,second)
				matrix<double> A(2,2),I=identity_matrix<double>(A.size1()),A_inv;
				vector<double> X(A.size1()),oldX(X),B(X);
				static matrix<double> oldXs(NUM_ACTUATOR,A.size1());

				if(time==0)
					oldXs.clear();
				A(0,0) = 0;
				A(0,1) = 1;
				A(1,0) = -cut[i] * cut[i];
				A(1,1) = -2 * cut[i];
				B(0) = 0;
				B(1) = cut[i] * cut[i];
				math::invert(I-SAMPLING_TIME/2*A,A_inv);
				oldX(0) = oldXs(i,0);
				oldX(1) = oldXs(i,1);
				X = prod(prod(A_inv,(I+SAMPLING_TIME/2*A)),oldX)+SAMPLING_TIME/2*prod(A_inv,B)*(pos(i)+oldpos(i));
				oldXs(i,0) = X(0);
				oldXs(i,1) = X(1);
				//pos(i) = X(0);
				vel(i) = X(1);
				acc(i) = A(1,0)*X(0) + A(1,1)*X(1) + cut[i]*cut[i]*pos(i);
			}else{
				// diff
				vel(i) = (pos(i) - oldpos(i)) / SAMPLING_TIME;
				acc(i) = (vel(i) - oldvel(i)) / SAMPLING_TIME;
			}
		}
	}
}

void iocda::OutputControlVariable(short AioId,vector<double>& f)
{
	int i;
	vector<double> force(f);
	float outputf[NUM_ACTUATOR];
	double max_force[NUM_LINEAR_ACTUATOR] = {MAX_FORCE_X,MAX_FORCE_Y1,MAX_FORCE_Y2,MAX_FORCE_Z};
	double max_torque[NUM_ROTATION_ACTUATOR] = {MAX_TORQUE_C,MAX_TORQUE_A1,MAX_TORQUE_A2};
	
	for(i=0;i<NUM_LINEAR_ACTUATOR;i++)
	{
		if(force(i) > max_force[i])
			force(i) = max_force[i];
		else if(force(i) < -max_force[i])
			force(i) = -max_force[i];
		outputf[i] = static_cast<float>(force(i)/max_force[i]*DA_MAX_VOLT);
	}
	for(i=0;i<NUM_ROTATION_ACTUATOR;i++)
	{
		if(force(i+NUM_LINEAR_ACTUATOR) > max_torque[i])
			force(i+NUM_LINEAR_ACTUATOR) = max_torque[i];
		else if(force(i+NUM_LINEAR_ACTUATOR) < -max_torque[i])
			force(i+NUM_LINEAR_ACTUATOR) = -max_torque[i];
		outputf[i+NUM_LINEAR_ACTUATOR] = static_cast<float>(force(i+NUM_LINEAR_ACTUATOR)/max_torque[i]*DA_MAX_VOLT);
	}

	f = force;

	for(i=0;i<NUM_ACTUATOR;i++)
		long AioRet = CAioCLI::AioSingleAoEx ( AioId , i , outputf[i] );
}

void iocda::OutputControlVariable(bool enable_resolution,vector<double>& f)
{
	int i;
	short output[NUM_ACTUATOR];
	double max_force[NUM_LINEAR_ACTUATOR] = {MAX_FORCE_X,MAX_FORCE_Y1,MAX_FORCE_Y2,MAX_FORCE_Z};
	double max_torque[NUM_ROTATION_ACTUATOR] = {MAX_TORQUE_C,MAX_TORQUE_A1,MAX_TORQUE_A2};
	double convert_coefficient[NUM_ACTUATOR],convert_offset;
	
	convert_offset = RESOLUTION_DA / 2;
	for(i=0;i<NUM_LINEAR_ACTUATOR;i++)
		convert_coefficient[i] = RESOLUTION_DA / (max_force[i] * 2);
	for(i=0;i<NUM_ROTATION_ACTUATOR;i++)
		convert_coefficient[i+NUM_LINEAR_ACTUATOR] = RESOLUTION_DA / (max_torque[i] * 2);

	for(i=0;i<NUM_LINEAR_ACTUATOR;i++)
	{
		if(f(i) > max_force[i])
			f(i) = max_force[i];
		else if(f(i) < -max_force[i])
			f(i) = -max_force[i];
		output[i] = static_cast<short>(f(i) * convert_coefficient[i] + convert_offset);
	}
	for(i=0;i<NUM_ROTATION_ACTUATOR;i++)
	{
		if(f(i+NUM_LINEAR_ACTUATOR) > max_torque[i])
			f(i+NUM_LINEAR_ACTUATOR) = max_torque[i];
		else if(f(i+NUM_LINEAR_ACTUATOR) < -max_torque[i])
			f(i+NUM_LINEAR_ACTUATOR) = -max_torque[i];
		output[i+NUM_LINEAR_ACTUATOR] = static_cast<short>(f(i+NUM_LINEAR_ACTUATOR) * convert_coefficient[i+NUM_LINEAR_ACTUATOR] + convert_offset);
	}

	if(enable_resolution){
		for(i=0;i<NUM_LINEAR_ACTUATOR;i++)
			f(i) = (output[i] - convert_offset) / convert_coefficient[i];
		for(i=0;i<NUM_ROTATION_ACTUATOR;i++)
			f(i+NUM_LINEAR_ACTUATOR) = (output[i+NUM_LINEAR_ACTUATOR] - convert_offset) / convert_coefficient[i+NUM_LINEAR_ACTUATOR];
	}
}
