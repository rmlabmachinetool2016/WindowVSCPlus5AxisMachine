#pragma once

//==============================================================================================
//	API-AIO(WDM)用定義ファイル																	
//==============================================================================================
using namespace System;
using namespace System::Runtime::InteropServices;
using namespace System::Text;

namespace CAioCLI
{
	//----------------------------------------------------------------------------------------------
	//	外部制御信号																				
	//----------------------------------------------------------------------------------------------
	#define	AIO_AIF_CLOCK				0	//アナログ入力外部クロック
	#define	AIO_AIF_START				1	//アナログ入力外部開始トリガ
	#define	AIO_AIF_STOP				2	//アナログ入力外部停止トリガ
	#define	AIO_AOF_CLOCK				3	//アナログ出力外部クロック
	#define	AIO_AOF_START				4	//アナログ出力外部開始トリガ
	#define	AIO_AOF_STOP				5	//アナログ出力外部停止トリガ

	//----------------------------------------------------------------------------------------------
	//	入出力レンジ																				
	//----------------------------------------------------------------------------------------------
	#define	PM10						0	//±10V
	#define	PM5							1	//±5V
	#define	PM25						2	//±2.5V
	#define	PM125						3	//±1.25V
	#define	PM1							4	//±1V
	#define	PM0625						5	//±0.625V
	#define	PM05						6	//±0.5V
	#define	PM03125						7	//±0.3125V
	#define	PM025						8	//±0.25V
	#define	PM0125						9	//±0.125V
	#define	PM01						10	//±0.1V
	#define	PM005						11	//±0.05V
	#define	PM0025						12	//±0.025V
	#define	PM00125						13	//±0.0125V
	#define	PM001						14	//±0.01V
	#define	P10							50	//0～10V
	#define	P5							51	//0～5V
	#define	P4095						52	//0～4.095V
	#define	P25							53	//0～2.5V
	#define	P125						54	//0～1.25V
	#define	P1							55	//0～1V
	#define	P05							56	//0～0.5V
	#define	P025						57	//0～0.25V
	#define	P01							58	//0～0.1V
	#define	P005						59	//0～0.05V
	#define	P0025						60	//0～0.025V
	#define	P00125						61	//0～0.0125V
	#define	P001						62	//0～0.01V
	#define	P20MA						100	//0～20mA
	#define	P4TO20MA					101	//4～20mA
	#define	P1TO5						150	//1～5V

	//----------------------------------------------------------------------------------------------
	//	アナログ入力イベント																		
	//----------------------------------------------------------------------------------------------
	#define	AIE_START			0x00000002	//AD変換開始条件成立イベント
	#define	AIE_RPTEND			0x00000010	//リピート終了イベント
	#define	AIE_END				0x00000020	//デバイス動作終了イベント
	#define	AIE_DATA_NUM		0x00000080	//指定サンプリング回数格納イベント
	#define	AIE_DATA_TSF		0x00000100	//指定転送数毎イベント
	#define	AIE_OFERR			0x00010000	//オーバーフローイベント
	#define	AIE_SCERR			0x00020000	//サンプリングクロックエラーイベント
	#define	AIE_ADERR			0x00040000	//AD変換エラーイベント

	//----------------------------------------------------------------------------------------------
	//	アナログ出力イベント																		
	//----------------------------------------------------------------------------------------------
	#define	AOE_START			0x00000002	//DA変換開始条件成立イベント
	#define	AOE_RPTEND			0x00000010	//リピート終了イベント
	#define	AOE_END				0x00000020	//デバイス動作終了イベント
	#define	AOE_DATA_NUM		0x00000080	//指定サンプリング回数出力イベント
	#define	AOE_DATA_TSF		0x00000100	//指定転送数毎イベント
	#define	AOE_SCERR			0x00020000	//サンプリングクロックエラーイベント
	#define	AOE_DAERR			0x00040000	//DA変換エラーイベント

	//----------------------------------------------------------------------------------------------
	//	カウンタイベント																			
	//----------------------------------------------------------------------------------------------
	#define	CNTE_DATA_NUM		0x00000010	//比較カウント一致イベント
	#define	CNTE_ORERR			0x00010000	//カウントオーバーランイベント
	#define	CNTE_ERR			0x00020000	//カウンタ動作エラー

	//----------------------------------------------------------------------------------------------
	//	タイマイベント																				
	//----------------------------------------------------------------------------------------------
	#define	TME_INT				0x00000001	//インターバル成立イベント

	//----------------------------------------------------------------------------------------------
	//	アナログ入力ステータス																		
	//----------------------------------------------------------------------------------------------
	#define	AIS_BUSY			0x00000001	//デバイス動作中
	#define	AIS_START_TRG		0x00000002	//開始トリガ待ち
	#define	AIS_DATA_NUM		0x00000010	//指定サンプリング回数格納
	#define	AIS_OFERR			0x00010000	//オーバーフロー
	#define	AIS_SCERR			0x00020000	//サンプリングクロックエラー
	#define	AIS_AIERR			0x00040000	//AD変換エラー
	#define	AIS_DRVERR			0x00080000	//ドライバスペックエラー

	//----------------------------------------------------------------------------------------------
	//	アナログ出力ステータス																		
	//----------------------------------------------------------------------------------------------
	#define	AOS_BUSY			0x00000001	//デバイス動作中
	#define	AOS_START_TRG		0x00000002	//開始トリガ待ち
	#define	AOS_DATA_NUM		0x00000010	//指定サンプリング回数出力
	#define	AOS_SCERR			0x00020000	//サンプリングクロックエラー
	#define	AOS_AOERR			0x00040000	//DA変換エラー
	#define	AOS_DRVERR			0x00080000	//ドライバスペックエラー

	//----------------------------------------------------------------------------------------------
	//	カウンタステータス																			
	//----------------------------------------------------------------------------------------------
	#define	CNTS_BUSY			0x00000001	//カウンタ動作中
	#define	CNTS_DATA_NUM		0x00000010	//比較カウント一致
	#define	CNTS_ORERR			0x00010000	//オーバーラン
	#define	CNTS_ERR			0x00020000	//カウンタ動作エラー

	//----------------------------------------------------------------------------------------------
	//	アナログ入力メッセージ																		
	//----------------------------------------------------------------------------------------------
	#define AIOM_AIE_START			0x1000	//AD変換開始条件成立イベント
	#define AIOM_AIE_RPTEND			0x1001	//リピート終了イベント
	#define AIOM_AIE_END			0x1002	//デバイス動作終了イベント
	#define AIOM_AIE_DATA_NUM		0x1003	//指定サンプリング回数格納イベント
	#define AIOM_AIE_DATA_TSF		0x1007	//指定転送数毎イベント
	#define AIOM_AIE_OFERR			0x1004	//オーバーフローイベント
	#define AIOM_AIE_SCERR			0x1005	//サンプリングクロックエラーイベント
	#define AIOM_AIE_ADERR			0x1006	//AD変換エラーイベント

	//----------------------------------------------------------------------------------------------
	//	アナログ出力メッセージ																		
	//----------------------------------------------------------------------------------------------
	#define AIOM_AOE_START			0x1020	//DA変換開始条件成立イベント
	#define AIOM_AOE_RPTEND			0x1021	//リピート終了イベント
	#define AIOM_AOE_END			0x1022	//デバイス動作終了イベント
	#define AIOM_AOE_DATA_NUM		0x1023	//指定サンプリング回数出力イベント
	#define AIOM_AOE_DATA_TSF		0x1027	//指定転送数毎イベント
	#define AIOM_AOE_SCERR			0x1025	//サンプリングクロックエラーイベント
	#define AIOM_AOE_DAERR			0x1026	//DA変換エラーイベント

	//----------------------------------------------------------------------------------------------
	//	カウンタメッセージ																			
	//----------------------------------------------------------------------------------------------
	#define AIOM_CNTE_DATA_NUM		0x1042	//比較カウント一致イベント
	#define AIOM_CNTE_ORERR			0x1043	//カウントオーバーランイベント
	#define AIOM_CNTE_ERR			0x1044	//カウント動作エラーイベント

	//----------------------------------------------------------------------------------------------
	//	タイマメッセージ																			
	//----------------------------------------------------------------------------------------------
	#define AIOM_TME_INT			0x1060	//インターバル成立イベント

	//----------------------------------------------------------------------------------------------
	//	アナログ入力添付データ																		
	//----------------------------------------------------------------------------------------------
	#define	AIAT_AI				0x00000001	//アナログ入力付属情報
	#define	AIAT_AO0			0x00000100	//アナログ出力データ
	#define	AIAT_DIO0			0x00010000	//デジタル入出力データ
	#define	AIAT_CNT0			0x01000000	//カウンタチャネル０データ
	#define	AIAT_CNT1			0x02000000	//カウンタチャネル１データ

	//----------------------------------------------------------------------------------------------
	//	カウンタ動作モード																			
	//----------------------------------------------------------------------------------------------
	#define	CNT_LOADPRESET		0x0000001	//プリセットカウント値のロード
	#define	CNT_LOADCOMP		0x0000002	//比較カウント値のロード

	//----------------------------------------------------------------------------------------------
	//	イベントコントローラ接続先信号																
	//----------------------------------------------------------------------------------------------
	#define	AIOECU_DEST_AI_CLK			4	//アナログ入力サンプリングクロック
	#define	AIOECU_DEST_AI_START		0	//アナログ入力変換開始信号
	#define	AIOECU_DEST_AI_STOP			2	//アナログ入力変換停止信号
	#define	AIOECU_DEST_AO_CLK			36	//アナログ出力サンプリングクロック
	#define	AIOECU_DEST_AO_START		32	//アナログ出力変換開始信号
	#define	AIOECU_DEST_AO_STOP			34	//アナログ出力変換停止信号
	#define	AIOECU_DEST_CNT0_UPCLK		134	//カウンタ０アップクロック信号
	#define	AIOECU_DEST_CNT1_UPCLK		135	//カウンタ１アップクロック信号
	#define	AIOECU_DEST_CNT0_START		128	//カウンタ０、タイマ０動作開始信号
	#define	AIOECU_DEST_CNT1_START		129	//カウンタ１、タイマ１動作開始信号
	#define	AIOECU_DEST_CNT0_STOP		130	//カウンタ０、タイマ０動作停止信号
	#define	AIOECU_DEST_CNT1_STOP		131	//カウンタ１、タイマ１動作停止信号
	#define	AIOECU_DEST_MASTER1			104	//同期バスマスタ信号１
	#define	AIOECU_DEST_MASTER2			105	//同期バスマスタ信号２
	#define	AIOECU_DEST_MASTER3			106	//同期バスマスタ信号３

	//----------------------------------------------------------------------------------------------
	//	イベントコントローラ接続元信号																
	//----------------------------------------------------------------------------------------------
	#define	AIOECU_SRC_OPEN				-1	//未接続
	#define	AIOECU_SRC_AI_CLK			4	//アナログ入力内部クロック信号
	#define	AIOECU_SRC_AI_EXTCLK		146	//アナログ入力外部クロック信号
	#define	AIOECU_SRC_AI_TRGSTART		144	//アナログ入力外部トリガ開始信号
	#define	AIOECU_SRC_AI_LVSTART		28	//アナログ入力レベルトリガ開始信号
	#define	AIOECU_SRC_AI_STOP			17	//アナログ入力変換回数終了信号（遅延なし）
	#define	AIOECU_SRC_AI_STOP_DELAY	18	//アナログ入力変換回数終了信号（遅延あり）
	#define	AIOECU_SRC_AI_LVSTOP		29	//アナログ入力レベルトリガ停止信号
	#define	AIOECU_SRC_AI_TRGSTOP		145	//アナログ入力外部トリガ停止信号
	#define	AIOECU_SRC_AO_CLK			66	//アナログ出力内部クロック信号
	#define	AIOECU_SRC_AO_EXTCLK		149	//アナログ出力外部クロック信号
	#define	AIOECU_SRC_AO_TRGSTART		147	//アナログ出力外部トリガ開始信号
	#define	AIOECU_SRC_AO_STOP_FIFO		352	//アナログ出力指定回数出力終了信号（FIFO使用）
	#define	AIOECU_SRC_AO_STOP_RING		80	//アナログ出力指定回数出力終了信号（RING使用）
	#define	AIOECU_SRC_AO_TRGSTOP		148	//アナログ出力外部トリガ停止信号
	#define	AIOECU_SRC_CNT0_UPCLK		150	//カウンタ０アップクロック信号
	#define	AIOECU_SRC_CNT1_UPCLK		152	//カウンタ１アップクロック信号
	#define	AIOECU_SRC_CNT0_CMP			288	//カウンタ０比較カウント一致
	#define	AIOECU_SRC_CNT1_CMP			289	//カウンタ１比較カウント一致
	#define	AIOECU_SRC_SLAVE1			136	//同期バススレーブ信号１
	#define	AIOECU_SRC_SLAVE2			137	//同期バススレーブ信号２
	#define	AIOECU_SRC_SLAVE3			138	//同期バススレーブ信号３
	#define	AIOECU_SRC_START			384	//Ai, Ao, Cnt, Tmソフトウェア開始信号
	#define	AIOECU_SRC_STOP				385	//Ai, Ao, Cnt, Tmソフトウェア停止信号

	//----------------------------------------------------------------------------------------------
	//	マルチファンクションデバイス用定義															
	//----------------------------------------------------------------------------------------------
	//-------------------------------------------------
	// Mode definition
	//-------------------------------------------------
	// Z Phase Mode
	#define	CNT_ZPHASE_NOT_USE			1
	#define	CNT_ZPHASE_NEXT_ONE			2
	#define	CNT_ZPHASE_EVERY_TIME		3
	// Z Phase Logic
	#define	CNT_ZLOGIC_POSITIVE			0
	#define	CNT_ZLOGIC_NEGATIVE			1
	// Signal Source
	#define	CNT_SIGTYPE_ISOLATE			0
	#define	CNT_SIGTYPE_TTL				1
	#define	CNT_SIGTYPE_LINERECEIVER	2
	// Count Direction
	#define	CNT_DIR_DOWN				0
	#define	CNT_DIR_UP					1
	// 1Phase/2Phase
	#define	CNT_MODE_1PHASE				0
	#define	CNT_MODE_2PHASE				1
	#define	CNT_MODE_GATECONTROL		2
	// Mul
	#define	CNT_MUL_X1					0
	#define	CNT_MUL_X2					1
	#define	CNT_MUL_X4					2
	// Sync Clear
	#define	CNT_CLR_ASYNC				0
	#define	CNT_CLR_SYNC				1
	// Gate Control
	#define	CNT_UPDOWN					1
	#define	CNT_GATECONTROL				0

	// Generic I/O Signal
	#define	CNT_GENIO_LINERECEIVER		0
	#define	CNT_GENIO_TTL				1

	// Device Information
	#define	ICNT_DEVICE_TYPE			0	// device type.						Param1:short
	#define	ICNT_NUMBER_OF_CH			1	// ﾁｬﾈﾙ数							Param1:int
	#define	ICNT_BIT					2	// ﾋﾞｯﾄ数							Param1:int
	#define	ICNT_IS_DIGITAL_FILTER		3	// ﾃﾞｼﾞﾀﾙﾌｨﾙﾀがあるか？				Param1:BOOL(True/False)
	#define	ICNT_IS_DEFF				4	// 汎用入力の差動切り替えあるか？	Param1:BOOL(True/False)
	#define	ICNT_CNTSOURCE				5	// カウンタの信号源　　　　　　　　 Param1:int(BIT0:TTL, BIT1:PHOTO, BIT2:LINERECEIVER)

	#define	ICNT_CNTSOURCE_TTL			0x01
	#define	ICNT_CNTSOURCE_PHOTO		0x02
	#define	ICNT_CNTSOURCE_LINERECEIVER	0x04
	//-------------------------------------------------
	// Message
	//-------------------------------------------------
	#define	CNTM_COUNTUP_CH0	0x1100		// カウントアップ、チャネル番号0
	#define	CNTM_COUNTUP_CH1	0x1101		//         "                   1
	#define	CNTM_COUNTUP_CH2	0x1102		//         "                   2
	#define	CNTM_COUNTUP_CH3	0x1103		//         "                   3
	#define	CNTM_COUNTUP_CH4	0x1104		//         "                   4
	#define	CNTM_COUNTUP_CH5	0x1105		//         "                   5
	#define	CNTM_COUNTUP_CH6	0x1106		//         "                   6
	#define	CNTM_COUNTUP_CH7	0x1107		//         "                   7

	#define	CNTM_TIME_UP		0x1140		// タイムアップ
	#define	CNTM_COUNTER_ERROR	0x1141		// カウンタエラー
	#define	CNTM_CARRY_BORROW	0x1142		// キャリー／ボロー

	// Count Match Direction(UP=1, DOWN=2)
	#define	CNTM_DIR_UP			1			// アップカウントで一致
	#define	CNTM_DIR_DOWN		2			// ダウンカウントで一致


		public delegate void PAICALLBACK(short Id, short Message, long wParam, long lParam, void * Param);
		public delegate void PAOCALLBACK(short Id, short Message, long wParam, long lParam, void * Param);
		public delegate void PCNTCALLBACK(short Id, short Message, long wParam, long lParam, void * Param);
		public delegate void PTMCALLBACK(short Id, short Message, long wParam, long lParam, void * Param);
//マルチファンクションデバイス用定義
		public delegate void PCOUNTUPCALLBACK(short Id, int wParam, int lParam, void *Param);
		public delegate void PCOUNTERERRORCALLBACK(short Id, int wParam, int lParam, void *Param);
		public delegate void PCARRYBORROWCALLBACK(short Id, int wParam, int lParam, void *Param);
		public delegate void PTIMERCALLBACK(short Id, int wParam, int lParam, void *Param);

	//----------------------------------------------------------------------------------------------
	//	関数プロトタイプ																			
	//----------------------------------------------------------------------------------------------
	//////////////////////////////////////////
	//共通関数
	[DllImport("caio.dll")] long AioInit(String^ DeviceName, short * Id);
	[DllImport("caio.dll")] long AioExit(short Id);
	[DllImport("caio.dll")] long AioResetDevice(short Id);
	
//nakahira String^ → StringBuilder^ に変更
	[DllImport("caio.dll")] long AioGetErrorString(long ErrorCode, StringBuilder^ ErrorString);
	[DllImport("caio.dll")] long AioQueryDeviceName(short Index, StringBuilder^ DeviceName, StringBuilder^ Device);
	
	[DllImport("caio.dll")] long AioGetDeviceType(String^ Device, short * DeviceType);
	[DllImport("caio.dll")] long AioSetControlFilter(short Id, short Signal, float Value);
	[DllImport("caio.dll")] long AioGetControlFilter(short Id, short Signal, float * Value);
	[DllImport("caio.dll")] long AioResetProcess(short Id);

	//アナログ入力関数
	[DllImport("caio.dll")] long AioSingleAi(short Id, short AiChannel, long * AiData);
	[DllImport("caio.dll")] long AioSingleAiEx(short Id, short AiChannel, float * AiData);
	[DllImport("caio.dll")] long AioMultiAi(short Id, short AiChannels, long * AiData);
	[DllImport("caio.dll")] long AioMultiAiEx(short Id, short AiChannels, float * AiData);
	[DllImport("caio.dll")] long AioGetAiResolution(short Id, short * AiResolution);
	[DllImport("caio.dll")] long AioSetAiInputMethod(short Id, short AiInputMethod);
	[DllImport("caio.dll")] long AioGetAiInputMethod(short Id, short * AiInputMethod);
	[DllImport("caio.dll")] long AioGetAiMaxChannels(short Id, short * AiMaxChannels);
	[DllImport("caio.dll")] long AioSetAiChannels(short Id, short AiChannels);
	[DllImport("caio.dll")] long AioGetAiChannels(short Id, short * AiChannels);
	[DllImport("caio.dll")] long AioSetAiChannelSequence(short Id, short AiSequence, short AiChannel);
	[DllImport("caio.dll")] long AioGetAiChannelSequence(short Id, short AiSequence, short * AiChannel);
	
//nakahira add
	[DllImport("caio.dll")] long AioSetAiChannel(short Id, short AiChannels, short Enabled);
	[DllImport("caio.dll")] long AioGetAiChannel(short Id, short AiChannels, short * Enabled);
	
	[DllImport("caio.dll")] long AioSetAiRange(short Id, short AiChannel, short AiRange);
	[DllImport("caio.dll")] long AioSetAiRangeAll(short Id, short AiRange);
	[DllImport("caio.dll")] long AioGetAiRange(short Id, short AiChannel, short * AiRange);
	[DllImport("caio.dll")] long AioSetAiTransferMode(short Id, short AiTransferMode);
	[DllImport("caio.dll")] long AioGetAiTransferMode(short Id, short * AiTransferMode);
	[DllImport("caio.dll")] long AioSetAiDeviceBufferMode(short Id, short AiDeviceBufferMode);
	[DllImport("caio.dll")] long AioGetAiDeviceBufferMode(short Id, short * AiDeviceBufferMode);
	[DllImport("caio.dll")] long AioSetAiMemorySize(short Id, long AiMemorySize);
	[DllImport("caio.dll")] long AioGetAiMemorySize(short Id, long * AiMemorySize);
	[DllImport("caio.dll")] long AioSetAiTransferData(short Id, long DataNumber, IntPtr Buffer);
	[DllImport("caio.dll")] long AioSetAiAttachedData(short Id, long AttachedData);
	[DllImport("caio.dll")] long AioGetAiSamplingDataSize(short Id, short * DataSize);
	[DllImport("caio.dll")] long AioSetAiMemoryType(short Id, short AiMemoryType);
	[DllImport("caio.dll")] long AioGetAiMemoryType(short Id, short * AiMemoryType);
	[DllImport("caio.dll")] long AioSetAiRepeatTimes(short Id, long AiRepeatTimes);
	[DllImport("caio.dll")] long AioGetAiRepeatTimes(short Id, long * AiRepeatTimes);
	[DllImport("caio.dll")] long AioSetAiClockType(short Id, short AiClockType);
	[DllImport("caio.dll")] long AioGetAiClockType(short Id, short * AiClockType);
	[DllImport("caio.dll")] long AioSetAiSamplingClock(short Id, float AiSamplingClock);
	[DllImport("caio.dll")] long AioGetAiSamplingClock(short Id, float * AiSamplingClock);
	[DllImport("caio.dll")] long AioSetAiScanClock(short Id, float AiScanClock);
	[DllImport("caio.dll")] long AioGetAiScanClock(short Id, float * AiScanClock);
	
//nakahira AoClockEdge → AiClockEdge
	[DllImport("caio.dll")] long AioSetAiClockEdge(short Id, short AiClockEdge);
	[DllImport("caio.dll")] long AioGetAiClockEdge(short Id, short * AiClockEdge);
	
	[DllImport("caio.dll")] long AioSetAiStartTrigger(short Id, short AiStartTrigger);
	[DllImport("caio.dll")] long AioGetAiStartTrigger(short Id, short * AiStartTrigger);
	[DllImport("caio.dll")] long AioSetAiStartLevel(short Id, short AiChannel, long AiStartLevel, short AiDirection);
	[DllImport("caio.dll")] long AioSetAiStartLevelEx(short Id, short AiChannel, float AiStartLevel, short AiDirection);
	[DllImport("caio.dll")] long AioGetAiStartLevel(short Id, short AiChannel, long * AiStartLevel, short * AiDirection);
	[DllImport("caio.dll")] long AioGetAiStartLevelEx(short Id, short AiChannel, float * AiStartLevel, short * AiDirection);
	[DllImport("caio.dll")] long AioSetAiStartInRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
	[DllImport("caio.dll")] long AioSetAiStartInRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
	[DllImport("caio.dll")] long AioGetAiStartInRange(short Id, short AiChannel, long *Level1, long *Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioGetAiStartInRangeEx(short Id, short AiChannel, float *Level1, float *Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioSetAiStartOutRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
	[DllImport("caio.dll")] long AioSetAiStartOutRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
	[DllImport("caio.dll")] long AioGetAiStartOutRange(short Id, short AiChannel, long * Level1, long * Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioGetAiStartOutRangeEx(short Id, short AiChannel, float * Level1, float * Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioSetAiStopTrigger(short Id, short AiStopTrigger);
	[DllImport("caio.dll")] long AioGetAiStopTrigger(short Id, short * AiStopTrigger);
	[DllImport("caio.dll")] long AioSetAiStopTimes(short Id, long AiStopTimes);
	[DllImport("caio.dll")] long AioGetAiStopTimes(short Id, long * AiStopTimes);
	[DllImport("caio.dll")] long AioSetAiStopLevel(short Id, short AiChannel, long AiStopLevel, short AiDirection);
	[DllImport("caio.dll")] long AioSetAiStopLevelEx(short Id, short AiChannel, float AiStopLevel, short AiDirection);
	[DllImport("caio.dll")] long AioGetAiStopLevel(short Id, short AiChannel, long * AiStopLevel, short * AiDirection);
	[DllImport("caio.dll")] long AioGetAiStopLevelEx(short Id, short AiChannel, float * AiStopLevel, short * AiDirection);
	[DllImport("caio.dll")] long AioSetAiStopInRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
	[DllImport("caio.dll")] long AioSetAiStopInRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
	[DllImport("caio.dll")] long AioGetAiStopInRange(short Id, short AiChannel, long * Level1, long * Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioGetAiStopInRangeEx(short Id, short AiChannel, float * Level1, float * Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioSetAiStopOutRange(short Id, short AiChannel, long Level1, long Level2, long StateTimes);
	[DllImport("caio.dll")] long AioSetAiStopOutRangeEx(short Id, short AiChannel, float Level1, float Level2, long StateTimes);
	[DllImport("caio.dll")] long AioGetAiStopOutRange(short Id, short AiChannel, long * Level1, long * Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioGetAiStopOutRangeEx(short Id, short AiChannel, float * Level1, float * Level2, long * StateTimes);
	[DllImport("caio.dll")] long AioSetAiStopDelayTimes(short Id, long AiStopDelayTimes);
	[DllImport("caio.dll")] long AioGetAiStopDelayTimes(short Id, long * AiStopDelayTimes);
//nakahira IntPtr hWnd → int hWnd
	[DllImport("caio.dll")] long AioSetAiEvent(short Id, int hWnd, long AiEvent);

//nakahira IntPtr ^ hWnd → int * hWnd
	[DllImport("caio.dll")] long AioGetAiEvent(short Id, int * hWnd, long * AiEvent);

//nakahira int AiEvent → long AiEvent
	[DllImport("caio.dll")] long AioSetAiCallBackProc(short Id, IntPtr pAiCallBack, long AiEvent, void * Param);

	[DllImport("caio.dll")] long AioSetAiEventSamplingTimes(short Id, long AiSamplingTimes);
	[DllImport("caio.dll")] long AioGetAiEventSamplingTimes(short Id, long * AiSamplingTimes);
	[DllImport("caio.dll")] long AioSetAiEventTransferTimes(short Id, long AiTransferTimes);
	[DllImport("caio.dll")] long AioGetAiEventTransferTimes(short Id, long * AiTransferTimes);
	[DllImport("caio.dll")] long AioStartAi(short Id);
	[DllImport("caio.dll")] long AioStartAiSync(short Id, long TimeOut);
	[DllImport("caio.dll")] long AioStopAi(short Id);
	[DllImport("caio.dll")] long AioGetAiStatus(short Id, long * AiStatus);
	[DllImport("caio.dll")] long AioGetAiSamplingCount(short Id, long * AiSamplingCount);
	[DllImport("caio.dll")] long AioGetAiStopTriggerCount(short Id, long * AiStopTriggerCount);
	[DllImport("caio.dll")] long AioGetAiTransferCount(short Id, long * AiTransferCount);
	[DllImport("caio.dll")] long AioGetAiTransferLap(short Id, long * Lap);
	[DllImport("caio.dll")] long AioGetAiStopTriggerTransferCount(short Id, long * Count);
	[DllImport("caio.dll")] long AioGetAiRepeatCount(short Id, long * AiRepeatCount);
	[DllImport("caio.dll")] long AioGetAiSamplingData(short Id, long * AiSamplingTimes, long * AiData);
	[DllImport("caio.dll")] long AioGetAiSamplingDataEx(short Id, long * AiSamplingTimes, float * AiData);
	[DllImport("caio.dll")] long AioResetAiStatus(short Id);
	[DllImport("caio.dll")] long AioResetAiMemory(short Id);

	//アナログ出力関数
	[DllImport("caio.dll")] long AioSingleAo(short Id, short AoChannel, long AoData);
	[DllImport("caio.dll")] long AioSingleAoEx(short Id, short AoChannel, float AoData);
	[DllImport("caio.dll")] long AioMultiAo(short Id, short AoChannels, long * AoData);
	[DllImport("caio.dll")] long AioMultiAoEx(short Id, short AoChannels, IntPtr AoData);
	[DllImport("caio.dll")] long AioGetAoResolution(short Id, short * AoResolution);
	[DllImport("caio.dll")] long AioSetAoChannels(short Id, short AoChannels);
	[DllImport("caio.dll")] long AioGetAoChannels(short Id, short * AoChannels);
	[DllImport("caio.dll")] long AioGetAoMaxChannels(short Id, short * AoMaxChannels);
	[DllImport("caio.dll")] long AioSetAoRange(short Id, short AoChannel, short AoRange);
	[DllImport("caio.dll")] long AioSetAoRangeAll(short Id, short AoRange);
	[DllImport("caio.dll")] long AioGetAoRange(short Id, short AoChannel, short * AoRange);
	[DllImport("caio.dll")] long AioSetAoTransferMode(short Id, short AoTransferMode);
	[DllImport("caio.dll")] long AioGetAoTransferMode(short Id, short * AoTransferMode);
	[DllImport("caio.dll")] long AioSetAoDeviceBufferMode(short Id, short AoDeviceBufferMode);
	[DllImport("caio.dll")] long AioGetAoDeviceBufferMode(short Id, short * AoDeviceBufferMode);
	[DllImport("caio.dll")] long AioSetAoMemorySize(short Id, long AoMemorySize);
	[DllImport("caio.dll")] long AioGetAoMemorySize(short Id, long * AoMemorySize);
	[DllImport("caio.dll")] long AioSetAoTransferData(short Id, long DataNumber, IntPtr Buffer);
	[DllImport("caio.dll")] long AioGetAoSamplingDataSize(short Id, short * DataSize);
	[DllImport("caio.dll")] long AioSetAoMemoryType(short Id, short AoMemoryType);
	[DllImport("caio.dll")] long AioGetAoMemoryType(short Id, short * AoMemoryType);
	[DllImport("caio.dll")] long AioSetAoRepeatTimes(short Id, long AoRepeatTimes);
	[DllImport("caio.dll")] long AioGetAoRepeatTimes(short Id, long * AoRepeatTimes);
	[DllImport("caio.dll")] long AioSetAoClockType(short Id, short AoClockType);
	[DllImport("caio.dll")] long AioGetAoClockType(short Id, short * AoClockType);
	[DllImport("caio.dll")] long AioSetAoSamplingClock(short Id, float AoSamplingClock);
	[DllImport("caio.dll")] long AioGetAoSamplingClock(short Id, float * AoSamplingClock);
	[DllImport("caio.dll")] long AioSetAoClockEdge(short Id, short AoClockEdge);
	[DllImport("caio.dll")] long AioGetAoClockEdge(short Id, short * AoClockEdge);
	[DllImport("caio.dll")] long AioSetAoSamplingData(short Id, long AoSamplingTimes, long * AoData);
	[DllImport("caio.dll")] long AioSetAoSamplingDataEx(short Id, long AoSamplingTimes, float * AoData);
	[DllImport("caio.dll")] long AioGetAoSamplingTimes(short Id, long * AoSamplingTimes);
	[DllImport("caio.dll")] long AioSetAoStartTrigger(short Id, short AoStartTrigger);
	[DllImport("caio.dll")] long AioGetAoStartTrigger(short Id, short * AoStartTrigger);
	[DllImport("caio.dll")] long AioSetAoStopTrigger(short Id, short AoStopTrigger);
	[DllImport("caio.dll")] long AioGetAoStopTrigger(short Id, short * AoStopTrigger);

//nakahira IntPtr hWnd → int hWnd
	[DllImport("caio.dll")] long AioSetAoEvent(short Id, int hWnd, long AoEvent);

//nakahira IntPtr * hWnd → int * hWnd
	[DllImport("caio.dll")] long AioGetAoEvent(short Id, int * hWnd, long * AoEvent);
	
//nakahira int AoEvent → long AoEvent
	[DllImport("caio.dll")] long AioSetAoCallBackProc(short Id, IntPtr pAoCallBack, long AoEvent, void * Param);
		
	[DllImport("caio.dll")] long AioSetAoEventSamplingTimes(short Id, long AoSamplingTimes);
	[DllImport("caio.dll")] long AioGetAoEventSamplingTimes(short Id, long * AoSamplingTimes);
	[DllImport("caio.dll")] long AioSetAoEventTransferTimes(short Id, long AoTransferTimes);
	[DllImport("caio.dll")] long AioGetAoEventTransferTimes(short Id, long * AoTransferTimes);
	[DllImport("caio.dll")] long AioStartAo(short Id);
	[DllImport("caio.dll")] long AioStopAo(short Id);
	[DllImport("caio.dll")] long AioEnableAo(short Id, short AoChannel);
	[DllImport("caio.dll")] long AioDisableAo(short Id, short AoChannel);
	[DllImport("caio.dll")] long AioGetAoStatus(short Id, long * AoStatus);
	[DllImport("caio.dll")] long AioGetAoSamplingCount(short Id, long * AoSamplingCount);
	[DllImport("caio.dll")] long AioGetAoTransferCount(short Id, long * AoTransferCount);
	[DllImport("caio.dll")] long AioGetAoTransferLap(short Id, long * Lap);
	[DllImport("caio.dll")] long AioGetAoRepeatCount(short Id, long * AoRepeatCount);
	[DllImport("caio.dll")] long AioResetAoStatus(short Id);
	[DllImport("caio.dll")] long AioResetAoMemory(short Id);

	//デジタル入出力関数
	[DllImport("caio.dll")] long AioSetDiFilter(short Id, short Bit, float Value);
	[DllImport("caio.dll")] long AioGetDiFilter(short Id, short Bit, float * Value);
	[DllImport("caio.dll")] long AioInputDiBit(short Id, short DiBit, short * DiData);
	[DllImport("caio.dll")] long AioOutputDoBit(short Id, short DoBit, short DoData);
	[DllImport("caio.dll")] long AioInputDiByte(short Id, short DiPort, short * DiData);
	[DllImport("caio.dll")] long AioOutputDoByte(short Id, short DoPort, short DoData);

	//カウンタ関数
	[DllImport("caio.dll")] long AioGetCntMaxChannels(short Id, short * CntMaxChannels);
	[DllImport("caio.dll")] long AioSetCntComparisonMode(short Id, short CntChannel, short CntMode);
	[DllImport("caio.dll")] long AioGetCntComparisonMode(short Id, short CntChannel, short * CntMode);
	[DllImport("caio.dll")] long AioSetCntPresetReg(short Id, short CntChannel, long PresetNumber, long * PresetData, short Flag);
	[DllImport("caio.dll")] long AioSetCntComparisonReg(short Id, short CntChannel, long ComparisonNumber, long * ComparisonData, short Flag);
	[DllImport("caio.dll")] long AioSetCntInputSignal(short Id, short CntChannel, short CntInputSignal);
	[DllImport("caio.dll")] long AioGetCntInputSignal(short Id, short CntChannel, short * CntInputSignal);
	
//nakahira IntPtr hWnd → int hWnd
	[DllImport("caio.dll")] long AioSetCntEvent(short Id, short CntChannel, int hWnd, long CntEvent);
	
//nakahira IntPtr * hWnd → int * hWnd
	[DllImport("caio.dll")] long AioGetCntEvent(short Id, short CntChannel, int * hWnd, long * CntEvent);
	
//nakahira int CntEvent → long CntEvent, short CntChannel追加
	[DllImport("caio.dll")] long AioSetCntCallBackProc(short Id, short CntChannel, IntPtr pCntCallBack, long CntEvent, void * Param);

	[DllImport("caio.dll")] long AioSetCntFilter(short Id, short CntChannel, short Signal, float Value);
	[DllImport("caio.dll")] long AioGetCntFilter(short Id, short CntChannel, short Signal, float * Value);
	[DllImport("caio.dll")] long AioStartCnt(short Id, short CntChannel);
	[DllImport("caio.dll")] long AioStopCnt(short Id, short CntChannel);
	[DllImport("caio.dll")] long AioPresetCnt(short Id, short CntChannel, long PresetData);
	[DllImport("caio.dll")] long AioGetCntStatus(short Id, short CntChannel, long * CntStatus);
	[DllImport("caio.dll")] long AioGetCntCount(short Id, short CntChannel, long * Count);
	[DllImport("caio.dll")] long AioResetCntStatus(short Id, short CntChannel, long CntStatus);
	
	//タイマ関数

//nakahira IntPtr hWnd → int hWnd
	[DllImport("caio.dll")] long AioSetTmEvent(short Id, short TimerId, int hWnd, long TmEvent);

//nakahira IntPtr * hWnd → int * hWnd
	[DllImport("caio.dll")] long AioGetTmEvent(short Id, short TimerId, int * hWnd, long * TmEvent);
	
//nakahira int TmEvent → long TmEvent, short TimerId追加
	[DllImport("caio.dll")] long AioSetTmCallBackProc(short Id, short TimerId, IntPtr pTmCallBack, long TmEvent, void * Param);

	[DllImport("caio.dll")] long AioStartTmTimer(short Id, short TimerId, float Interval);
	[DllImport("caio.dll")] long AioStopTmTimer(short Id, short TimerId);
	[DllImport("caio.dll")] long AioStartTmCount(short Id, short TimerId);
	[DllImport("caio.dll")] long AioStopTmCount(short Id, short TimerId);
	[DllImport("caio.dll")] long AioLapTmCount(short Id, short TimerId, long * Lap);
	[DllImport("caio.dll")] long AioResetTmCount(short Id, short TimerId);
	[DllImport("caio.dll")] long AioTmWait(short Id, short TimerId, long Wait);
	
	//イベントコントローラ
	[DllImport("caio.dll")] long AioSetEcuSignal(short Id, short Destination, short Source);
	[DllImport("caio.dll")] long AioGetEcuSignal(short Id, short Destination, short * Source);
	
//マルチファンクションデバイス用関数定義
	// Prototype definition
	//-------------------------------------------------
	// Setting function (set)
	[DllImport("caio.dll")] long AioSetCntmZMode(short Id, short ChNo, short Mode);
	[DllImport("caio.dll")] long AioSetCntmZLogic(short Id, short ChNo, short ZLogic);
	[DllImport("caio.dll")] long AioSelectCntmChannelSignal(short Id, short ChNo, short SigType);
	[DllImport("caio.dll")] long AioSetCntmCountDirection(short Id, short ChNo, short Dir);
	[DllImport("caio.dll")] long AioSetCntmOperationMode(short Id, short ChNo, short Phase, short Mul, short SyncClr);
	[DllImport("caio.dll")] long AioSetCntmDigitalFilter(short Id, short ChNo, short FilterValue);
	[DllImport("caio.dll")] long AioSetCntmPulseWidth(short Id, short ChNo, short PlsWidth);
	[DllImport("caio.dll")] long AioSetCntmDIType(short Id, short ChNo, short InputType);
	[DllImport("caio.dll")] long AioSetCntmOutputHardwareEvent(short Id, short ChNo, short OutputLogic, unsigned long EventType, short PulseWidth);
	[DllImport("caio.dll")] long AioSetCntmInputHardwareEvent(short Id, short ChNo, unsigned long EventType, short RF0, short RF1, short Reserved);
	[DllImport("caio.dll")] long AioSetCntmCountMatchHardwareEvent(short Id, short ChNo, short RegisterNo, unsigned long EventType, short Reserved);
	[DllImport("caio.dll")] long AioSetCntmPresetRegister(short Id, short ChNo, unsigned long PresetData, short Reserved);
	// Setting function (get)
	[DllImport("caio.dll")] long AioGetCntmZMode(short Id, short ChNo, short *Mode);
	[DllImport("caio.dll")] long AioGetCntmZLogic(short Id, short ChNo, short *ZLogic);
	[DllImport("caio.dll")] long AioGetCntmChannelSignal(short Id, short ChNo, short *SigType);
	[DllImport("caio.dll")] long AioGetCntmCountDirection(short Id, short ChNo, short *Dir);
	[DllImport("caio.dll")] long AioGetCntmOperationMode(short Id, short ChNo, short *Phase, short *Mul, short *SyncClr);
	[DllImport("caio.dll")] long AioGetCntmDigitalFilter(short Id, short ChNo, short *FilterValue);
	[DllImport("caio.dll")] long AioGetCntmPulseWidth(short Id, short ChNo, short *PlsWidth);
	// Counter function
	[DllImport("caio.dll")] long AioCntmStartCount(short Id, short *ChNo, short ChNum);
	[DllImport("caio.dll")] long AioCntmStopCount(short Id, short *ChNo, short ChNum);
	[DllImport("caio.dll")] long AioCntmPreset(short Id, short *ChNo, short ChNum, unsigned long *PresetData);
	[DllImport("caio.dll")] long AioCntmZeroClearCount(short Id, short *ChNo, short ChNum);
	[DllImport("caio.dll")] long AioCntmReadCount(short Id, short *ChNo, short ChNum, unsigned long *CntDat);
	[DllImport("caio.dll")] long AioCntmReadStatus(short Id, short ChNo, short *Sts);
	[DllImport("caio.dll")] long AioCntmReadStatusEx(short Id, short ChNo, unsigned long *Sts);
	// Notify function
	[DllImport("caio.dll")] long AioCntmNotifyCountUp(short Id, short ChNo, short RegNo, unsigned long Count, int hWnd);
	[DllImport("caio.dll")] long AioCntmStopNotifyCountUp(short Id, short ChNo, short RegNo);
	[DllImport("caio.dll")] long AioCntmCountUpCallbackProc(short Id , IntPtr CallBackProc, void *Param);
	[DllImport("caio.dll")] long AioCntmNotifyCounterError(short Id, int hWnd);
	[DllImport("caio.dll")] long AioCntmStopNotifyCounterError(short Id);
	[DllImport("caio.dll")] long AioCntmCounterErrorCallbackProc(short Id , IntPtr CallBackProc , void *Param);
	[DllImport("caio.dll")] long AioCntmNotifyCarryBorrow(short Id, int hWnd);
	[DllImport("caio.dll")] long AioCntmStopNotifyCarryBorrow(short Id);
	[DllImport("caio.dll")] long AioCntmCarryBorrowCallbackProc(short Id, IntPtr CallBackProc, void *Param);
	[DllImport("caio.dll")] long AioCntmNotifyTimer(short Id, unsigned long TimeValue, int hWnd);
	[DllImport("caio.dll")] long AioCntmStopNotifyTimer(short Id);
	[DllImport("caio.dll")] long AioCntmTimerCallbackProc(short Id , IntPtr CallBackProc , void *Param);
	// General purpose input function
	[DllImport("caio.dll")] long AioCntmInputDIByte(short Id, short Reserved, Byte *bData);
	[DllImport("caio.dll")] long AioCntmOutputDOBit(short Id, short ChNo, short Reserved, Byte OutData);
}