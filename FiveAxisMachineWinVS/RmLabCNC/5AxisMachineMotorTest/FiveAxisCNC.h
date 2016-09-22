#pragma once
#include "StdAfx.h"
#include "FiveAxisControlModule.h"
#include "FiveAxisIOModule.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
namespace RmLabCNC{
ref class FiveAxisCNC
{
public:
	//x y1 y2 z c a1 a2	::	x y1 z c a1
	//0 1  2  3 4 5  6	::	0 1  2 3 4
	static const short LINEAR_MOTOR_X					= 0;					// Control by AIO port 0
	static const short LINEAR_MOTOR_Y1					= 1;					// Control by AIO port 1
	static const short LINEAR_MOTOR_Y2					= 2;					// Control by AIO port 2
	static const short LINEAR_MOTOR_Z					= 3;					// Control by AIO port 3
	static const short LINEAR_MOTOR_C					= 4;					// Control by AIO port 4
	static const short LINEAR_MOTOR_A1					= 5;					// Control by AIO port 5
	static const short LINEAR_MOTOR_A2					= 6;					// Control by AIO port 6

	FiveAxisCNC(void);
	~FiveAxisCNC(void);
	FiveAxisControlModule ControlModule;
	FiveAxisIOModule IOModule;
	double SAMPLING_TIME;
	void FeedbackTimer();

};
}
