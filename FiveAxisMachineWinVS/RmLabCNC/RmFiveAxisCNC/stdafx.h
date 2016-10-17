// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
#pragma once

// TODO: reference additional headers your program requires here
// Define address for counter board and DA boar
// Change from Dinhba
#define THIS_AIO_NAME  "AIO000"
#define THIS_CNT_NAME  "CNT000"   // Current 5 Axis;  5 Axis CNC "CNT000" , 3 Axis CNC "CNT001"


#define TRUE  1
#define FALSE 0
#include "GUIProcessing.h"
#include "FiveAxisCNC.h"
#include "CAioCLI.h"
#include "CCntCLI.h"