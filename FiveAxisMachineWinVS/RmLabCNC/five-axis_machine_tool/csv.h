#ifndef _CSV_H_
#define _CSV_H_

#include <cstdio>
#include <cerrno>


namespace csv{
	static bool G_iWriteErrFlg	= false;
	static int G_iWriteErrNo	= 0;
	static char *LINE_SEP		= "\n";

	void proc_write_err(void);
	int write_char(FILE *fp, char ch);
	int write_str(FILE *fp, const char *cpStr);
	/* 区切り文字出力 */
	int write_sep(FILE *fp, int hasNext);
	/*
	 * CSV形式で値を出力
	 *「"」で囲う場合
	 * 引数:
	 *   *fp     出力先ファイル
	 *   hasNext 同一行に後続の値があればTRUE ->「,」を出力
	 *           なければFALSE -> 改行を出力
	 *   *cpStr  出力する値
	 */
	void write_val(FILE *fp, int hasNext, const char *cpStr);
}

#endif