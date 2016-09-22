#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_

#include <cmath>
#include <boost\numeric\ublas\vector.hpp>
#include <boost\numeric\ublas\matrix.hpp>

using namespace boost::numeric::ublas;

#define R 0.61803398874989484820
#define C (1-R)
#define GOLD (1+R)
#define GLIMIT 100
#define TINY (1.0e-20)
#define	SHFT2(a,b,c)	(a)=(b); (b)=(c);
#define	SHFT3(a,b,c,d)	(a)=(b); (b)=(c); (c)=(d);

#define	NMAX		50000		// ŠÖ”•]‰¿‰ñ”‚ÌãŒÀ
#define	SWAP(a, b)		{swap = (a); (a) = (b); (b) = swap;}

namespace optimization{

	double function2d(vector<double>& co,vector<double>& val);
	double function3d(vector<double>& co,vector<double>& val);

	void swap(double& a,double& b);
	void shift2(double& a,double& b,double c);
	void shift3(double& a,double& b,double& c,double d);
	double sign(double a,double b);
	double fmax(double a,double b);

	namespace golden{
		double golden(
			double ax,
			double bx,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&),
			double tol,
			double& xmin);

		void mnbrak(
			double &ax,
			double &bx,
			double &cx,
			double &fa,
			double &fb,
			double &fc,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&)
			);
		double neogolden(
			double ax,
			double bx,
			double cx,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&),
			double tol,
			double& xmin);
	};

	namespace brent{
#define		ITMAX	100			// ”½•œ‰ñ”‚ÌãŒÀ
#define		ZEPS	1.0e-10		// ‹É¬‚ª‚¿‚å‚¤‚Ç x=0 ‚É‚ ‚é‚Æ‚«‚Í‘Š‘Î¸“x tol‚Ì‘ã‚í‚è‚É‚±‚ê‚ğâ‘Î¸“x‚Æ‚·‚é
		double brent(
			double ax,
			double bx,
			double cx,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&),
			double tol,
			double& xmin);
	};

	namespace amoeba{
		void amoeba(
			matrix<double>& p,
			vector<double>& co,
			vector<double>& y,
			int ndim,
			double ftol,
			double (*funk)(vector<double>& co,vector<double>&),
			int& nfunk);

		double amotry(
			matrix<double>& p,
			vector<double>& co,
			vector<double>& y,
			vector<double>& psum,
			int ndim,
			double (*funk)(vector<double>& co,vector<double>&),
			int ihi,
			double fac);
	};

};


#endif