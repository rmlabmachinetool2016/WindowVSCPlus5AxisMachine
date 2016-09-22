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
	/* ��؂蕶���o�� */
	int write_sep(FILE *fp, int hasNext);
	/*
	 * CSV�`���Œl���o��
	 *�u"�v�ň͂��ꍇ
	 * ����:
	 *   *fp     �o�͐�t�@�C��
	 *   hasNext ����s�Ɍ㑱�̒l�������TRUE ->�u,�v���o��
	 *           �Ȃ����FALSE -> ���s���o��
	 *   *cpStr  �o�͂���l
	 */
	void write_val(FILE *fp, int hasNext, const char *cpStr);
}

#endif