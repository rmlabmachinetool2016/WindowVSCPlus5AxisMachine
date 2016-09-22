#ifndef CONTROL2D_H
#define CONTROL2D_H

namespace control2d{
	static const unsigned int INDEPENDENT		= 0;
	static const unsigned int CONTOURING		= 1;
	static const unsigned int MOD_CONTOURING	= 2;
		
	namespace conparam{
		static double pole_x,pole_y1,pole_y2;
		static double pole_t,pole_n;
		static double pole_sync_y;
		static double mass_x,mass_y1,mass_y2;
		static double c_x,c_y1,c_y2;
		static matrix<double> mass,invmass,c,kpw,kvw,kpl,kvl;
		static double masss,cs,kvs,kps;
	}

	void ControlSim2D(	unsigned int control_type,
						unsigned int max_step,
						bool enable_noise,
						bool enable_resolution,
						bool enable_sync,
						array<double,2>^ datalog);

	void SetConParam(array<double>^ conparam);
	bool IsReachPoint(double time,double max_angle,array<double,2>^ datalog);

	void GenerateReference(double time,vector<double>& rpos,vector<double>& rvel,vector<double>& racc);

	void CalcRotationMat(double time,matrix<double>& Rot,matrix<double>& Rot1,matrix<double>& Rot2);

	double CalcContouringError(vector<double> &pos);

	void IndependentCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void IndependentCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void ContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void ContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void modContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void modContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void Identification(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);
}

#endif