#include "independent_control.h"

void IndependentControl(CDAT *cdat,CPARAM *cparam)
{
	MOTOR motor;
	MATRIX kpw,kvw,cmass,cc;
	VECTOR rpos,rvel,racc,pos,vel,acc;
	VECTOR f,ew,ew1,ew2;
	VECTOR temp[10];
	unsigned int step;
	double time,theta,ec;
	int i;
	double d[10];

	for(i=0;i<10;i++)
		MakeVector(&temp[i],DIM);

	MakeMatrix(&kpw,DIM,DIM);
	MakeMatrix(&kvw,DIM,DIM);
	MakeMatrix(&cmass,DIM,DIM);
	MakeMatrix(&cc,DIM,DIM);

	MakeVector(&rpos,DIM);
	MakeVector(&rvel,DIM);
	MakeVector(&racc,DIM);
	MakeVector(&pos,DIM);
	MakeVector(&vel,DIM);
	MakeVector(&acc,DIM);
	MakeVector(&f,DIM);
	MakeVector(&ew,DIM);
	MakeVector(&ew1,DIM);
	MakeVector(&ew2,DIM);

	ZeroMatrix(&kpw);
	kpw.index[0][0]=cparam->pole_x * cparam->pole_x;
	kpw.index[1][1]=cparam->pole_y * cparam->pole_y;
	ZeroMatrix(&kvw);
	kvw.index[0][0]=2*cparam->pole_x;
	kvw.index[1][1]=2*cparam->pole_y;
	ZeroMatrix(&cmass);
	cmass.index[0][0]=cparam->cmass_x;
	cmass.index[1][1]=cparam->cmass_y;
	ZeroMatrix(&cc);
	cc.index[0][0]=cparam->cc_x;
	cc.index[1][1]=cparam->cc_y;

	InitState(&motor);

	for(step=0;step<MAX_COUNT;step++)
	{
		time = step * SAMPLE_TIME;
		theta = OMG*time;

		GetStateMotor(time,&motor,&pos,&vel,&acc);

		GenerateReference(time,&rpos,&rvel,&racc);
		//GenRefEllipse(time,&rpos,&rvel,&racc);

		SubVector(&ew,&pos,&rpos);
		SubVector(&ew1,&vel,&rvel);
		SubVector(&ew2,&acc,&racc);

		MulMatrixVector(&temp[0],&kvw,&ew1);
		MulMatrixVector(&temp[1],&kpw,&ew);
		SubVector(&temp[2],&racc,&temp[0]);
		SubVector(&temp[3],&temp[2],&temp[1]);
		MulMatrixVector(&temp[4],&cmass,&temp[3]);
		MulMatrixVector(&temp[5],&cc,&vel);
		AddVector(&f,&temp[4],&temp[5]);

		f.index[0]=Sat(f.index[0],MAX_FORCE,-MAX_FORCE);
		f.index[1]=Sat(f.index[1],MAX_FORCE,-MAX_FORCE);

		DAconvert(&f);

		MotorModel(time,&motor,&f);
		
		ec = sqrt((pos.index[0])*(pos.index[0])+pos.index[1]*pos.index[1]) - RADIUS;
		//ec = sqrt(pos.index[0]*pos.index[0]+(pos.index[1]-RADIUS)*(pos.index[1]-RADIUS)) - RADIUS;

		cdat->f[0][step]=f.index[0];
		cdat->f[1][step]=f.index[1];

		cdat->rpos[0][step]=rpos.index[0];
		cdat->rpos[1][step]=rpos.index[1];
		cdat->rvel[0][step]=rvel.index[0];
		cdat->rvel[1][step]=rvel.index[1];
		cdat->racc[0][step]=racc.index[0];
		cdat->racc[1][step]=racc.index[1];

		cdat->pos[0][step]=pos.index[0];
		cdat->pos[1][step]=pos.index[1];
		cdat->vel[0][step]=vel.index[0];
		cdat->vel[1][step]=vel.index[1];
		cdat->acc[0][step]=acc.index[0];
		cdat->acc[1][step]=acc.index[1];

		cdat->ew[0][step]=ew.index[0];
		cdat->ew[1][step]=ew.index[1];
		cdat->el[0][step]=0;
		cdat->el[1][step]=0;
		cdat->ec[step]=ec;
		cdat->ec_en[step]=0;
	}

	for(i=0;i<10;i++)
		FreeVector(&temp[i]);

	FreeMatrix(&kpw);
	FreeMatrix(&kvw);
	FreeMatrix(&cmass);
	FreeMatrix(&cc);

	FreeVector(&rpos);
	FreeVector(&rvel);
	FreeVector(&racc);
	FreeVector(&pos);
	FreeVector(&vel);
	FreeVector(&acc);
	FreeVector(&f);
	FreeVector(&ew);
	FreeVector(&ew1);
	FreeVector(&ew2);
}