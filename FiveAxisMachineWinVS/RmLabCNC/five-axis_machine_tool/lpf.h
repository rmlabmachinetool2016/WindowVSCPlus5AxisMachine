#ifndef LPF_H
#define LPF_H

namespace lpf{

	class clpf{
	private:
		matrix<double>	mA,mxf;
		vector<double>	vB,oldvx,oldvxf,vxf;
		double			cut,oldx,oldxf,xf,A,B;
	public:
		clpf();
		clpf(double cutofffreq);
		double LPFON(double x);
		vector<double> LPFOFF(vector<double> vx);
		void clear(void);
	};
	
}
typedef lpf::clpf LPF;

#endif