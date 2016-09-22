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

/* 区切り文字出力 */
int csv::write_sep(FILE *fp, int hasNext)
{
    int iRc = 0;
    char *cpSep = NULL; /* 区切り文字 */

    /* 行の途中ならカンマ区切り */
    if (hasNext) {
        cpSep = ",";
    }
    /* 行の最後なら改行 */
    else {
        cpSep = LINE_SEP;
    }

    /* ファイルへ出力 */
    iRc = write_str(fp, cpSep);
    if (0 != iRc) {
        return -1;
    }

    return 0;
}

/*
 * CSV形式で値を出力
 *「"」で囲う場合
 * 引数:
 *   *fp     出力先ファイル
 *   hasNext 同一行に後続の値があればTRUE ->「,」を出力
 *           なければFALSE -> 改行を出力
 *   *cpStr  出力する値
 */
void csv::write_val(FILE *fp, int hasNext, const char *cpStr)
{
    int iRc = 0;

    if (G_iWriteErrFlg) {
        return;
    }

    /* ダブルクォーテーションで囲う（開き） */
    iRc = write_str(fp, "\"");
    if (0 != iRc) {
        return;
    }

    /* 1byte単位 */
    while ('\0' != *cpStr) {
        iRc = write_char(fp, *cpStr);
        if (0 != iRc) {
            return;
        }

        /* 「"」なら「""」に変換 */
        /* 「"」(0x22)はシフトJISの範囲外のため2byte文字の考慮は不要 */
        if ('"' == *cpStr) {
            iRc = write_char(fp, '"');
            if (0 != iRc) {
                return;
            }
        }

        cpStr++;
    }

    /* ダブルクォーテーションで囲う（閉じ） */
    iRc = write_str(fp, "\"");
    if (0 != iRc) {
        return;
    }

    /* 区切り文字を付加 */
    iRc = write_sep(fp, hasNext);
    if (0 != iRc) {
        return;
    }
}
