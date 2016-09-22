#include "stdafx.h"
#include "optimization.h"

void optimization::swap(double& a,double& b)
{
	double temp = a;
	a = b;
	b = temp;
}

void optimization::shift2(double& a,double& b,double c)
{
	a = b;
	b = c;
}

void optimization::shift3(double& a,double& b,double& c,double d)
{
	a = b;
	b = c;
	c = d;
}

double optimization::sign(double a,double b)
{
	return (b>=0 ? abs(a) : -abs(a));
}

double optimization::fmax(double a,double b)
{
	return (a>b ? a : b);
}



double optimization::golden::golden(double ax,
									double bx,
									vector<double>& co,
									vector<double>& val,
									double (*f)(vector<double>&,vector<double>&),
									double tol,
									double& xmin)
{
	double f1, f2, x0, x1, x2, x3;

	x0 = ax;							// �S�_ x0,x1,x2,x3 ���X�V���Ă���
	x1 = ax + C * (bx - ax);
	x2 = ax + R * (bx - ax); 
	x3 = bx;

	//f1 = (*f)(x1);						// �֐��̍ŏ��̕]��
	//f2 = (*f)(x2);
	val(0) = x1;
	f1 = (*f)(co,val);
	val(0) = x2;
	f2 = (*f)(co,val);

	while(fabs(x3 - x0) > tol * (fabs(x1) + fabs(x2))){	// ��������
		if(f2 < f1){								// �ꍇ�����̈��
			SHFT3(x0, x1, x2, R * x1 + C * x3)	// �e�_�̍X�V
			val(0)=x2;
			SHFT2(f1, f2, (*f)(co,val))				// �֐���]��x2
		}else{									// �ꍇ�����̂������
			SHFT3(x3, x2, x1, R * x2 + C * x0)
			val(0)=x1;
			SHFT2(f2, f1, (*f)(co,val))				// �֐���]�� x1
		}
	}

	if(f1 < f2){				// �����B�ŐV�̂Q�_�̂����ǂ�����Ԃ�
		xmin = x1;
		return f1;
	}else{
		xmin = x2;
		return f2;
	}
}

void optimization::golden::mnbrak(
			double &ax,
			double &bx,
			double &cx,
			double &fa,
			double &fb,
			double &fc,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&))
{
	double ulim,u,r,q,fu,dum;

	//fa = (*f)(ax);
	//fb = (*f)(bx);
	val(0) = ax;
	fa = (*f)(co,val);
	val(0) = bx;
	fb = (*f)(co,val);

	if(fb > fa){
		SHFT3(dum,ax,bx,dum)
		SHFT3(dum,fb,fa,dum)
	}
	cx = bx + GOLD*(bx-ax);
	val(0) = cx;
	fc = (*f)(co,val);
	while(fb > fc){
		r = (bx - ax)*(fb - fc);
		q = (bx - cx)*(fb - fa);
		u = bx - ((bx-cx)*q-(bx-ax)*r)/(2*sign(fmax(fabs(q-r),TINY),q-r));
		ulim = bx+GLIMIT*(cx-bx);

		if( (bx-u)*(u-cx) > 0 ){
			val(0)=u;
			fu = (*f)(co,val);
			if(fu < fc){
				ax=bx;
				bx=u;
				fa=fb;
				fb=fu;
				return;
			}else if(fu > fb){
				cx=u;
				fc=fu;
				return;
			}
			u = cx + GOLD*(cx - bx);
			val(0)=u;
			fu = (*f)(co,val);
		}
		else if( (cx-u)*(u-ulim) > 0 ){
			val(0)=u;
			fu=(*f)(co,val);
			if(fu < fc){
				SHFT3(bx,cx,u,cx+GOLD*(cx-bx))
				val(0)=u;
				SHFT3(fb,fc,fu,(*f)(co,val))
			}
		}
		else if( (ulim-cx)*(u-ulim) >= 0 ){
			u=ulim;
			val(0)=u;
			fu=(*f)(co,val);
		}else{
			u=cx+GOLD*(cx-bx);
			val(0)=u;
			fu=(*f)(co,val);
		}
		SHFT3(ax,bx,cx,u)
		SHFT3(fa,fb,fc,fu)
	}
}

double optimization::golden::neogolden(
			double ax,
			double bx,
			double cx,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&),
			double tol,
			double& xmin)
{
	double f1, f2, x0, x1, x2, x3;

	x0 = ax;							// �S�_ x0,x1,x2,x3 ���X�V���Ă���
	x1 = ax + C * (bx - ax);
	x2 = ax + R * (bx - ax); 
	x3 = cx;

	if(abs(cx-bx) > abs(bx-ax)){
		x1=bx;
		x2=bx + C*(cx-bx);
	}else{
		x2=bx;
		x1=bx - C*(bx-ax);
	}

	val(0) = x1;
	f1 = (*f)(co,val);
	val(0) = x2;
	f2 = (*f)(co,val);

	while(fabs(x3 - x0) > tol * (fabs(x1) + fabs(x2))){	// ��������
		if(f2 < f1){								// �ꍇ�����̈��
			SHFT3(x0, x1, x2, R * x1 + C * x3)	// �e�_�̍X�V
			val(0)=x2;
			SHFT2(f1, f2, (*f)(co,val))				// �֐���]��x2
		}else{									// �ꍇ�����̂������
			SHFT3(x3, x2, x1, R * x2 + C * x0)
			val(0)=x1;
			SHFT2(f2, f1, (*f)(co,val))				// �֐���]�� x1
		}
	}

	if(f1 < f2){				// �����B�ŐV�̂Q�_�̂����ǂ�����Ԃ�
		xmin = x1;
		return f1;
	}else{
		xmin = x2;
		return f2;
	}
}

double optimization::brent::brent(
			double ax,
			double bx,
			double cx,
			vector<double>& co,
			vector<double>& val,
			double (*f)(vector<double>&,vector<double>&),
			double tol,
			double& xmin)
{
	int iter;
	double a, b, d, etemp, fu, fv, fw, fx, p, q, r, tol1, tol2, u, v, w, x, xm;
	double e = 0.0;						/* �O�X��̍X�V�� */

	if(ax < cx){							/* a < b �ɂ��� */
		a = ax;
		b = cx;
	}else{
		a = cx;
		b = ax;
	}

	x = w = v = bx;						/* ������ */
	fw = fv = fx = (*f)(co,val);

	for(iter = 1; iter <= ITMAX; iter++)	/* �僋�[�v */
	{
		xm = 0.5 * (a + b);
		tol1 = tol * fabs(x) + ZEPS;
		tol2 = 2.0 * tol1;
		if(fabs(x - xm) <= (tol2 - 0.5 * (b - a)))	/* �������� */
		{
			xmin = x;					/* �ŗǂ̒l��Ԃ� */
			return fx;
		}
		if(fabs(e) > tol1)				/* ��������Ԃ��Ă݂� */
		{
			r = (x - w) * (fx - fv);
			q = (x - v) * (fx - fw);
			p = (x - v) * q - (x - w) * r;
			q = 2.0 * (q - r);
			if(q > 0.0)	p = -p;
			q = fabs(q);
			etemp = e;
			e = d;
			if(fabs(p) >= fabs(0.5 * q * etemp)
				|| p <= q * (a - x)
				|| p >= q * (b - x))	/* ��������Ԃ̓K�ۂ̌��� */
			{
				e = (x >= xm)? a - x: b - x;
				d = C * e;			/* ��������Ԃ͕s�K�B�傫�����̋�Ԃ� */
										/* �������� */
			}else{
				d = p / q;				/* ��������Ԃ��̑����� */
				u = x + d;
				if(u - a < tol2 || b - u < tol2)	d = sign(tol1, xm - x);
			}
		}else{
			e = (x >= xm)? a - x: b - x;
			d = C * e;
		}
		//u = x + ((fabs(d) >= tol1)? d: sign(tol1, d));
		val(0) = x + ((fabs(d) >= tol1)? d: sign(tol1, d));
		fu = (*f)(co,val);					/* �僋�[�v�ł̊֐��l�]���͂������� */
		if(fu <= fx){
			if(u >= x)	a = x;
			else		b = x;
			SHFT3(v, w, x, u);
			SHFT3(fv, fw, fx, fu);
		}else{
			if(u < x)	a = u;
			else		b = u;
			if(fu <= fw || w == x)
			{
				v = w;
				w = u;
				fv = fw;
				fw = fu;
			}
			else if(fu <= fv || v == x || v == w)
			{
				v = u;
				fv = fu;
			}
		}
	}
	//fprintf(stderr, "Error : Too many iterations in brent.\n");
	xmin = x;

	return fx;
}

/*
Nelder-Mead �̊��~�V���v���b�N�X�@�ɂ��֐� funk(x) �̑������̍ŏ����B
x[ ] �� ndim �����̃x�N�g���B���͂���s�� p[ ][ ] �� ndim+1 �̍s�́A�o���_
�ł̃V���v���b�N�X�̒��_�̍��W��\�� ndim �����̃x�N�g���B���͂���x�N�g��
ftol �͊֐��l�i�ɏ��_�̍��W�ł͂Ȃ��I�j�̎����𔻒肷�邽�߂̑��΋��e�l�B
�߂莞�� p �� y �́A�ŏI�ʒu�ł� ndim+1 �̒��_�̍��W�Ɗ֐��l�i�ǂ̊֐��l��
�ŏ��̊֐��l�Ƃ̑��ΓI�ȊO�ꂪ ftol �ȓ��j�Bnfunk �͊֐��̕]���񐔁B
*/
void optimization::amoeba::amoeba(
	matrix<double>& p,
	vector<double>& co,
	vector<double>& y,
	int ndim,
	double ftol,
	double (*funk)(vector<double>& co,vector<double>&),
	int& nfunk)
{
	int i, ihi, ilo, inhi, j, mpts = ndim + 1;
	double rtol, sum, swap, ysave, ytry;
	boost::numeric::ublas::vector<double> psum(ndim);

	for(j = 0; j < mpts; j++){
		double ppp[3];
		ppp[0]=p(j,0);ppp[1]=p(j,1);ppp[2]=p(j,2);
		y(j) = (*funk)(co,static_cast<boost::numeric::ublas::vector<double>>(row(p,j)));
	}
	//psum = vector(ndim);
	nfunk = 0;
	for(j = 0; j < ndim; j++)
	{
		sum = 0.0;
		for(i = 0; i < mpts; i++)	sum += p(i,j);
		psum[j] = sum;
	}
	for(;;)
	{
		ilo = 0;
/* �܂��V���v���b�N�X�̒��_�ɂ��ă��[�v���A�ň��A�ň��̎��A�ŗǂ̓_�𒲂ׂ�B*/
		if(y(0) > y(1))
		{
			inhi = 1;
			ihi = 0;
		}
		else
		{
			inhi = 0;
			ihi = 1;
		}
		for(i = 0; i < mpts; i++)
		{
			if(y(i) <= y(ilo))	ilo = i;
			if(y(i) > y(ihi))
			{
				inhi = ihi;
				ihi = i;
			}
			else if(y(i) > y(inhi) && i != ihi)	inhi = i;
		}
		rtol = 2.0 * fabs(y(ihi) - y(ilo)) / (fabs(y(ihi)) + fabs(y(ilo)));

/* �֐��l�̍ő�̔�����߁A���ꂪ�\���������Ȃ�I���B*/
		if(rtol < ftol)				/* �߂�ۂɂ͍ŗǂ̓_�𓪂Ɏ����čs�� */
		{
			SWAP(y(i), y(ilo));
			for(i = 0; i < ndim; i++)	SWAP(p(0,i), p(ilo,i));
			break;
		}
		if(nfunk >= NMAX)
		{
			printf("Process Stop : NMAX(=%d) exceeded.\n", NMAX);
			SWAP(y(0), y(ilo));
			for(i = 0; i < ndim; i++)	SWAP(p(0,i), p(ilo,i));
			break;
		}
		nfunk += 2;
/* �V�����������n�߂�B�܂��ň��̓_���c��̓_�̏d�S�̔��Α��ɑΏ̈ړ�����B*/
		ytry = amotry(p,co, y, psum, ndim, funk, ihi, -1.0);
		if(ytry <= y(ilo))
		{
/* ���݂̍ŗǂ̓_���ǂ��Ȃ����̂ŁA����ɂQ�{�����i��ł݂�B*/
			ytry = amotry(p,co, y, psum, ndim, funk, ihi, 2.0);
		}
		else if(ytry >= y(inhi))
		{
/* �Ώ̈ړ������_�͂Q�ԖڂɈ����_���������̂łP�����̎��k�����݂�B*/
			ysave = y(ihi);
			ytry = amotry(p,co, y, psum, ndim, funk, ihi, 0.5);
			if(ytry >= ysave)
			{
/* ����ł��ǂ��Ȃ��Ȃ�S�̂��ŗǓ_�Ɍ������Ď��k�B*/
				for(i = 0; i < mpts; i++)
				{
					if(i != ilo)
					{
						for(j = 0; j < ndim; j++)
							p(i,j) = psum[j] = 0.5 * (p(i,j) + p(ilo,j));
						y(i) = (*funk)(co,psum);
					}
				}
				nfunk += ndim;			/* �֐��̕]���񐔂𐔂���B */
				for(j = 0; j < ndim; j++)		/* psum ���Čv�Z�B */
				{
					sum = 0.0;
					for(i = 0; i < mpts; i++)	sum += p(i,j);
					psum[j] = sum;
				}
			}
		}
		else	--nfunk;				/* �]���񐔂�␳�B */
	}
}

double optimization::amoeba::amotry(
	matrix<double>& p,
	vector<double>& co,
	vector<double>& y,
	vector<double>& psum,
	int ndim,
	double (*funk)(vector<double>& co,vector<double>&),
	int ihi,
	double fac)
{
	int j;
	double fac1, fac2, ytry;

	boost::numeric::ublas::vector<double> ptry(ndim);
	fac1 = (1.0 - fac) / ndim;
	fac2 = fac1 - fac;

	for(j = 0; j < ndim; j++)
		ptry(j) = psum[j] * fac1 - p(ihi,j) * fac2;

	ytry = (*funk)(co,ptry);				/* �V�����_�Ŋ֐��]�� */
	if(ytry < y(ihi))					/* �V�����_�̕����ǂ���Ό��̍ň��� */
	{									/* �̂ĂĒu��������B */
		y(ihi) = ytry;
		for(j = 0; j < ndim; j++)
		{
			psum[j] += ptry(j) - p(ihi,j);
			p(ihi,j) = ptry(j);
		}
	}

	return ytry;
}


double optimization::function2d(vector<double>& co,vector<double>& val)
{
	using namespace define;
	double a[2],ec,time;
	vector<double> wref(2),wrefdot(wref),wrefddot(wref);

	time = val(0);
	control2d::GenerateReference(time,wref,wrefdot,wrefddot);
	a[0] = wref(0) - co(0);
	a[1] = wref(1) - co(1);
	ec = sqrt( (a[0]*a[0])+(a[1]*a[1]) );

	return ec;
}

double optimization::function3d(vector<double>& co,vector<double>& val)
{
	using namespace define;
	double a[3],ec,time;
	vector<double> wref(NUM_DOF),wrefdot(wref),wrefddot(wref);

	time = val(0);
	(*control3d::GenR)(time,wref,wrefdot,wrefddot);
	a[0] = wref(0) - co(0);
	a[1] = wref(1) - co(1);
	a[2] = wref(2) - co(2);
	ec = sqrt( (a[0]*a[0])+(a[1]*a[1])+(a[2]*a[2]) );

	return ec;
}