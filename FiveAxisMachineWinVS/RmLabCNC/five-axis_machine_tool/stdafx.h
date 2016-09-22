// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
#pragma once

// TODO: プログラムに必要な追加ヘッダーをここで参照してください。
#include "define.h"
#include "control2D.h"
#include "control3D.h"
#include "csv.h"
#include <boost/lexical_cast.hpp>	// for cast int. to std::string
#include <fstream>
#include <time.h>
#include "CAioCLI.h"
#include "CCntCLI.h"
#include <vcclr.h>					// for convertion String to char*
#include "optimization.h"
#include "FormIdentification.h"
#include "Identification.h"
#include "lpf.h"
