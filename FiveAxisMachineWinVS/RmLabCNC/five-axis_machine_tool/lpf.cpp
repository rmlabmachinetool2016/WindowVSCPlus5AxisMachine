#include "stdafx.h"
#include "lpf.h"

LPF::clpf()
{
	cut = 1;
	oldx = 0;
	oldxf = 0;
	oldvx.clear();
	oldvxf.clear();

	A = SAMPLING_TIME*cut/(2+SAMPLING_TIME*cut);
	B = (2-SAMPLING_TIME*cut)/(2+SAMPLING_TIME*cut);

	matrix<double> a(2,2),a1,a2,inva2,I=identity_matrix<double>(a.size1());
	vector<double> b(2);
	a(0,0) = 0;			a(0,1) = 1;
	a(1,0) = -cut*cut;	a(1,1) = -2*cut;
	b(0) = 0;
	b(1) = cut*cut;
	mA.resize(2,2);
	vB.resize(2);
	a1 = I + SAMPLING_TIME/2*a;
	a2 = I - SAMPLING_TIME/2*a;
	math::invert(a2,inva2);
	mA = prod(inva2,a1);
	vB = SAMPLING_TIME/2*prod(inva2,b);
}

LPF::clpf(double cutofffreq)
{
	cut = cutofffreq;
	oldx = 0;
	oldxf = 0;
	oldvx.clear();
	oldvxf.clear();

	A = SAMPLING_TIME*cut/(2+SAMPLING_TIME*cut);
	B = (2-SAMPLING_TIME*cut)/(2+SAMPLING_TIME*cut);

	matrix<double> a(2,2),a1,a2,inva2,I=identity_matrix<double>(a.size1());
	vector<double> b(2);
	a(0,0) = 0;			a(0,1) = 1;
	a(1,0) = -cut*cut;	a(1,1) = -2*cut;
	b(0) = 0;
	b(1) = cut*cut;
	mA.resize(2,2);
	vB.resize(2);
	a1 = I + SAMPLING_TIME/2*a;
	a2 = I - SAMPLING_TIME/2*a;
	math::invert(a2,inva2);
	mA = prod(inva2,a1);
	vB = SAMPLING_TIME/2*prod(inva2,b);
}

double LPF::LPFON(double x)
{
	xf = A*(x+oldx) + B*oldxf;
	oldx = x;
	oldxf = xf;
	return xf;
}

vector<double> LPF::LPFOFF(vector<double> vx)
{
	unsigned int i;
	vxf.resize(vx.size());
	for(i=0;i<vx.size();i++)
		vxf(i) = LPFON( vx(i) );
	return vxf;
}

void LPF::clear(void)
{
	oldx = 0;
	oldxf = 0;
	oldvx.clear();
	oldvxf.clear();
}
