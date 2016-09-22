#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

using namespace boost::numeric::ublas;  // boost::numeric::ublas –¼‘O‹óŠÔ‚ðŽg—p


namespace control{
	static const unsigned int INDEPENDENT		= 0;
	static const unsigned int CONTOURING		= 1;
	static const unsigned int MOD_CONTOURING	= 2;
		
	namespace conparam{
		static double pole_x,pole_y1,pole_y2,pole_z,pole_c,pole_a1,pole_a2;
		static double pole_t,pole_n,pole_b,pole_i,pole_j,pole_k;
		static double pole_sync_y,pole_sync_a;
		static double mass_x,mass_y1,mass_y2,mass_z,mass_c,mass_a1,mass_a2;
		static double c_x,c_y1,c_y2,c_z,c_c,c_a1,c_a2;
		static matrix<double> mass,mass_inv,c,kpw,kvw,kpl,kvl;
		static matrix<double> masss,cs,kvs,kps;
		static matrix<double> kvo,kpo,Mo,Co;
	}

	void ControlSim(unsigned int control_type,
					bool select_3d,
					unsigned int max_step,
					bool enable_noise,
					bool enable_resolution,
					bool enable_sync,
					array<double,2>^ datalog);

	void SetConParam(array<double>^ conparam);
	bool IsReachPoint(double time,double max_angle,array<double,2>^ datalog);

	void GenerateReference(double time,vector<double>& rpos,vector<double>& rvel,vector<double>& racc);
	void GenerateWReference(double time,vector<double>& wref,vector<double>& wrefdot,vector<double>& wrefddot);

	void CalcConvertMatrixT(matrix<double>& T,matrix<double>& TT,vector<double>& q);
	void CalcJ(matrix<double>& J,vector<double>& q);
	void CalcJdot(vector<double>& q,vector<double>& qdot,matrix<double>& Jdot);
	void CalcJo(matrix<double>& Jo,vector<double>& q);
	void CalcJodot(matrix<double>& Jodot,vector<double>& q,vector<double>& qdot);
	void CalcFrenetFrame(vector<double>& wdot,vector<double>& wddot,matrix<double>& F);
	void CalcOrientation(vector<double>& O,vector<double>& q);

	double CalcContouringError(vector<double> &pos);

	typedef vector<double> dvector;
	typedef matrix<double> dmatrix;
	void SetStateVal(dvector& pos,dvector& vel,dvector& acc,dvector& posall,dvector& velall,dvector& accall);
	void SetReferenceVal(bool enable_sync,dvector& rpos,dvector& rvel,dvector& racc,dvector& rposall,dvector& rvelall,dvector& raccall);


	void IndependentCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void IndependentCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void ContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void ContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void modContouringCon(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void modContouringCon(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void IndependentCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void IndependentCon5(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);
	void ContouringCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void ContouringCon5(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);
	void modContouringCon5(unsigned int max_step,bool enable_noise,bool enable_resolution,bool enable_sync,array<double,2>^ datalog);
	void modContouringCon5(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);

	void Identification(unsigned int step,short AioId,short CntId,bool enable_sync,array<double,2>^ datalog);
}


#endif