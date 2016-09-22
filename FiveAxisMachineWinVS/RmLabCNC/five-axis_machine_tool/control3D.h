#ifndef CONTROL3D_H
#define CONTROL3D_H

//#define REF_WORLD

namespace control3d{
	static const unsigned int INDEPENDENT		= 0;
	static const unsigned int CONTOURING		= 1;
	static const unsigned int MOD_CONTOURING	= 2;
		
	namespace conparam{
		static double			pole_x,pole_y1,pole_y2,pole_z,pole_c,pole_a1,pole_a2;
		static double			pole_t,pole_n,pole_b,pole_i,pole_j,pole_k;
		static double			pole_sync_y,pole_sync_a;
		static double			mass_x,mass_y1,mass_y2,mass_z,mass_c,mass_a1,mass_a2;
		static double			c_x,c_y1,c_y2,c_z,c_c,c_a1,c_a2;
		static matrix<double>	mass,invmass,c,kpw,kvw,kpl,kvl;
		static matrix<double>	masss,cs,kvs,kps;
		static matrix<double>	kvo,kpo,Mo,Co;

		static vector<double>	force,forceall,
								ew_log,el_log,
								pw_log,pwdot_log,pw2dot_log,
								pl_log,pldot_log,pl2dot_log,
								prw_log,prwdot_log,prw2dot_log,
								prl_log,prldot_log,prl2dot_log,
								force_log;
		static vector<double>	posall,velall,accall;
		static vector<double>	pl,pldot,pl2dot,pw,pwdot,pw2dot;
		static vector<double>	prl,prldot,prl2dot,prw,prwdot,prw2dot;
		static vector<double>	ew,ewdot,ew2dot;
		static vector<double>	es,esdot,forces,
								poss,vels,accs,
								rposs,rvels,raccs;
		static vector<double>	eF,eFa;
		static double			td;
	}

	void ControlSim3D(	unsigned int control_type,
						unsigned int max_step,
						bool enable_noise,
						bool enable_resolution,
						bool enable_sync,
						array<double,2>^ datalog);

	void SetConParam(array<double>^ conparam);
	
	void GenTWorld(double time,vector<double>& wref,vector<double>& wref1,vector<double>& wref2);
	void GenTLocal(double time,vector<double>& lref,vector<double>& lref1,vector<double>& lref2);
	void GenTWorld34(double time,vector<double>& wref3,vector<double>& wref4);
	void GenTLocal34(double time,vector<double>& lref3,vector<double>& lref4);
	void GenRW(double time,vector<double>& wref,vector<double>& wref1,vector<double>& wref2);
	void GenRW34(double time,vector<double>& wref3,vector<double>& wref4);
	void GenRL(double time,vector<double>& wref,vector<double>& wref1,vector<double>& wref2);
	void GenRL34(double time,vector<double>& wref3,vector<double>& wref4);
	static void (*GenR)(double time,vector<double>& wref,vector<double>& wref1,vector<double>& wref2)
#ifdef REF_WORLD
		= GenRW;
#else
		= GenRL;
#endif
		
	static void (*GenR34)(double time,vector<double>& wref3,vector<double>& wref4)
#ifdef REF_WORLD
		= GenRW34;
#else
		= GenRL34;
#endif

	void CalcConvertMatrixT(matrix<double>& T,matrix<double>& TT,vector<double>& q);
	void CalcJ(matrix<double>& J,vector<double>& q);
	void CalcJdot(matrix<double>& Jdot,vector<double>& q,vector<double>& qdot);

	vector<double> CalcOrientation(vector<double>& q);
	void CalcJo(matrix<double>& Jo,vector<double>& q);
	void CalcJodot(matrix<double>& Jodot,vector<double>& q,vector<double>& qdot);
	
	void CalcFrenetFrame(double time,matrix<double>& F,matrix<double>& F1,matrix<double>& F2);

	vector<double> CalcOrientationControlSignal(double time,vector<double>& pl,vector<double>& pldot,double& phi);

	void SetStateVal(vector<double>& pos,vector<double>& vel,vector<double>& acc,vector<double>& posall,vector<double>& velall,vector<double>& accall);
	void LogingData(unsigned int step,array<double,2>^ datalog);

	vector<double> SyncCon(double time,vector<double>& posall,vector<double>& velall,vector<double>& pw,vector<double>& pwdot);

	void IndependentCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void IndependentCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void ContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void ContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void modContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void modContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);
}

#endif