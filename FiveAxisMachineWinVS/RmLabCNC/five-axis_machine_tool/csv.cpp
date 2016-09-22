#include "stdafx.h"
#include "csv.h"

void csv::proc_write_err(void)
{
    G_iWriteErrFlg = true;
    G_iWriteErrNo = errno;
}

int csv::write_char(FILE *fp, char ch)
{
    int iRc = 0;

    errno = 0;
    iRc = fputc(ch, fp);
    if (EOF == iRc) {
        proc_write_err();
        return -1;
    }

    return 0;
}

int csv::write_str(FILE *fp, const char *cpStr)
{
    int iRc = 0;

    errno = 0;
    iRc = fputs(cpStr, fp);
    if (EOF == iRc) {
        proc_write_err();
        return -1;
    }

    return 0;
}

/* ��؂蕶���o�� */
int csv::write_sep(FILE *fp, int hasNext)
{
    int iRc = 0;
    char *cpSep = NULL; /* ��؂蕶�� */

    /* �s�̓r���Ȃ�J���}��؂� */
    if (hasNext) {
        cpSep = ",";
    }
    /* �s�̍Ō�Ȃ���s */
    else {
        cpSep = LINE_SEP;
    }

    /* �t�@�C���֏o�� */
    iRc = write_str(fp, cpSep);
    if (0 != iRc) {
        return -1;
    }

    return 0;
}

/*
 * CSV�`���Œl���o��
 *�u"�v�ň͂��ꍇ
 * ����:
 *   *fp     �o�͐�t�@�C��
 *   hasNext ����s�Ɍ㑱�̒l�������TRUE ->�u,�v���o��
 *           �Ȃ����FALSE -> ���s���o��
 *   *cpStr  �o�͂���l
 */
void csv::write_val(FILE *fp, int hasNext, const char *cpStr)
{
    int iRc = 0;

    if (G_iWriteErrFlg) {
        return;
    }

    /* �_�u���N�H�[�e�[�V�����ň͂��i�J���j */
    iRc = write_str(fp, "\"");
    if (0 != iRc) {
        return;
    }

    /* 1byte�P�� */
    while ('\0' != *cpStr) {
        iRc = write_char(fp, *cpStr);
        if (0 != iRc) {
            return;
        }

        /* �u"�v�Ȃ�u""�v�ɕϊ� */
        /* �u"�v(0x22)�̓V�t�gJIS�͈̔͊O�̂���2byte�����̍l���͕s�v */
        if ('"' == *cpStr) {
            iRc = write_char(fp, '"');
            if (0 != iRc) {
                return;
            }
        }

        cpStr++;
    }

    /* �_�u���N�H�[�e�[�V�����ň͂��i���j */
    iRc = write_str(fp, "\"");
    if (0 != iRc) {
        return;
    }

    /* ��؂蕶����t�� */
    iRc = write_sep(fp, hasNext);
    if (0 != iRc) {
        return;
    }
}
