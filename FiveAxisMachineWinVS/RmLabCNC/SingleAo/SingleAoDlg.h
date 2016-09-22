//API-AIO(WDM)用定義ファイル	
#include "CAioCLI.h"
#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
using namespace System::Text;
using namespace CAioCLI;

namespace SingleAo {

	/// <summary>
	/// SingleAoDlg の概要
	///
	/// 警告: このクラスの名前を変更する場合、このクラスが依存するすべての .resx ファイルに関連付けられた
	///          マネージ リソース コンパイラ ツールに対して 'Resource File Name' プロパティを
	///          変更する必要があります。この変更を行わないと、
	///          デザイナと、このフォームに関連付けられたローカライズ済みリソースとが、
	///          正しく相互に利用できなくなります。
	/// </summary>
	public ref class SingleAoDlg : public System::Windows::Forms::Form
	{
	public:
		SingleAoDlg(void)
		{
			InitializeComponent();
			//
			//TODO: ここにコンストラクタ コードを追加します
			//
		}

	protected:
		/// <summary>
		/// 使用中のリソースをすべてクリーンアップします。
		/// </summary>
		~SingleAoDlg()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::ComboBox^  Combo_AioSetRange;
	protected: 
	private: System::Windows::Forms::TextBox^  Text_ErrorString;
	private: System::Windows::Forms::TextBox^  Text_AoChannel;
	private: System::Windows::Forms::TextBox^  Text_Device;
	private: System::Windows::Forms::Button^  Button_AioExit;
	private: System::Windows::Forms::Button^  Button__AioSingleAo;
	private: System::Windows::Forms::Button^  Button_AioInit;
	private: System::Windows::Forms::Label^  Label_Cdata;
	private: System::Windows::Forms::Label^  _Static_0;
	private: System::Windows::Forms::Label^  _Static_3;
	private: System::Windows::Forms::TextBox^  Text_Volt;

	private:
		/// <summary>
		/// 必要なデザイナ変数です。
		/// </summary>

		System::ComponentModel::Container ^components;

		//ID
		short	Id;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// デザイナ サポートに必要なメソッドです。このメソッドの内容を
		/// コード エディタで変更しないでください。
		/// </summary>
		void InitializeComponent(void)
		{
			this->Combo_AioSetRange = (gcnew System::Windows::Forms::ComboBox());
			this->Text_ErrorString = (gcnew System::Windows::Forms::TextBox());
			this->Text_AoChannel = (gcnew System::Windows::Forms::TextBox());
			this->Text_Device = (gcnew System::Windows::Forms::TextBox());
			this->Button_AioExit = (gcnew System::Windows::Forms::Button());
			this->Button__AioSingleAo = (gcnew System::Windows::Forms::Button());
			this->Button_AioInit = (gcnew System::Windows::Forms::Button());
			this->Label_Cdata = (gcnew System::Windows::Forms::Label());
			this->_Static_0 = (gcnew System::Windows::Forms::Label());
			this->_Static_3 = (gcnew System::Windows::Forms::Label());
			this->Text_Volt = (gcnew System::Windows::Forms::TextBox());
			this->SuspendLayout();
			// 
			// Combo_AioSetRange
			// 
			this->Combo_AioSetRange->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->Combo_AioSetRange->FormattingEnabled = true;
			this->Combo_AioSetRange->Location = System::Drawing::Point(119, 51);
			this->Combo_AioSetRange->Name = L"Combo_AioSetRange";
			this->Combo_AioSetRange->Size = System::Drawing::Size(134, 20);
			this->Combo_AioSetRange->TabIndex = 14;
			this->Combo_AioSetRange->SelectedIndexChanged += gcnew System::EventHandler(this, &SingleAoDlg::Combo_AioSetRange_SelectedIndexChanged);
			// 
			// Text_ErrorString
			// 
			this->Text_ErrorString->BackColor = System::Drawing::SystemColors::Menu;
			this->Text_ErrorString->Location = System::Drawing::Point(12, 166);
			this->Text_ErrorString->Multiline = true;
			this->Text_ErrorString->Name = L"Text_ErrorString";
			this->Text_ErrorString->ReadOnly = true;
			this->Text_ErrorString->Size = System::Drawing::Size(332, 40);
			this->Text_ErrorString->TabIndex = 16;
			// 
			// Text_AoChannel
			// 
			this->Text_AoChannel->Location = System::Drawing::Point(119, 102);
			this->Text_AoChannel->Name = L"Text_AoChannel";
			this->Text_AoChannel->Size = System::Drawing::Size(54, 19);
			this->Text_AoChannel->TabIndex = 15;
			this->Text_AoChannel->Text = L"0";
			// 
			// Text_Device
			// 
			this->Text_Device->Location = System::Drawing::Point(12, 12);
			this->Text_Device->Name = L"Text_Device";
			this->Text_Device->Size = System::Drawing::Size(87, 19);
			this->Text_Device->TabIndex = 7;
			this->Text_Device->Text = L"AIO000";
			// 
			// Button_AioExit
			// 
			this->Button_AioExit->Location = System::Drawing::Point(12, 127);
			this->Button_AioExit->Name = L"Button_AioExit";
			this->Button_AioExit->Size = System::Drawing::Size(87, 24);
			this->Button_AioExit->TabIndex = 13;
			this->Button_AioExit->Text = L"終了処理";
			this->Button_AioExit->UseVisualStyleBackColor = true;
			this->Button_AioExit->Click += gcnew System::EventHandler(this, &SingleAoDlg::Button_AioExit_Click);
			// 
			// Button__AioSingleAo
			// 
			this->Button__AioSingleAo->Location = System::Drawing::Point(12, 87);
			this->Button__AioSingleAo->Name = L"Button__AioSingleAo";
			this->Button__AioSingleAo->Size = System::Drawing::Size(87, 24);
			this->Button__AioSingleAo->TabIndex = 11;
			this->Button__AioSingleAo->Text = L"簡易出力";
			this->Button__AioSingleAo->UseVisualStyleBackColor = true;
			this->Button__AioSingleAo->Click += gcnew System::EventHandler(this, &SingleAoDlg::Button__AioSingleAo_Click);
			// 
			// Button_AioInit
			// 
			this->Button_AioInit->Location = System::Drawing::Point(12, 47);
			this->Button_AioInit->Name = L"Button_AioInit";
			this->Button_AioInit->Size = System::Drawing::Size(87, 24);
			this->Button_AioInit->TabIndex = 9;
			this->Button_AioInit->Text = L"初期化";
			this->Button_AioInit->UseVisualStyleBackColor = true;
			this->Button_AioInit->Click += gcnew System::EventHandler(this, &SingleAoDlg::Button_AioInit_Click);
			// 
			// Label_Cdata
			// 
			this->Label_Cdata->AutoSize = true;
			this->Label_Cdata->Location = System::Drawing::Point(192, 87);
			this->Label_Cdata->Name = L"Label_Cdata";
			this->Label_Cdata->Size = System::Drawing::Size(29, 12);
			this->Label_Cdata->TabIndex = 12;
			this->Label_Cdata->Text = L"電圧";
			// 
			// _Static_0
			// 
			this->_Static_0->AutoSize = true;
			this->_Static_0->Location = System::Drawing::Point(120, 87);
			this->_Static_0->Name = L"_Static_0";
			this->_Static_0->Size = System::Drawing::Size(42, 12);
			this->_Static_0->TabIndex = 10;
			this->_Static_0->Text = L"チャネル";
			// 
			// _Static_3
			// 
			this->_Static_3->AutoSize = true;
			this->_Static_3->Location = System::Drawing::Point(120, 36);
			this->_Static_3->Name = L"_Static_3";
			this->_Static_3->Size = System::Drawing::Size(33, 12);
			this->_Static_3->TabIndex = 8;
			this->_Static_3->Text = L"レンジ";
			// 
			// Text_Volt
			// 
			this->Text_Volt->Location = System::Drawing::Point(194, 102);
			this->Text_Volt->Name = L"Text_Volt";
			this->Text_Volt->Size = System::Drawing::Size(59, 19);
			this->Text_Volt->TabIndex = 17;
			this->Text_Volt->Text = L"0";
			// 
			// SingleAoDlg
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(357, 221);
			this->Controls->Add(this->Text_Volt);
			this->Controls->Add(this->Combo_AioSetRange);
			this->Controls->Add(this->Text_ErrorString);
			this->Controls->Add(this->Text_AoChannel);
			this->Controls->Add(this->Text_Device);
			this->Controls->Add(this->Button_AioExit);
			this->Controls->Add(this->Button__AioSingleAo);
			this->Controls->Add(this->Button_AioInit);
			this->Controls->Add(this->Label_Cdata);
			this->Controls->Add(this->_Static_0);
			this->Controls->Add(this->_Static_3);
			this->MaximizeBox = false;
			this->Name = L"SingleAoDlg";
			this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
			this->Text = L"指定チャネル1回アナログ出力";
			this->Load += gcnew System::EventHandler(this, &SingleAoDlg::SingleAoDlg_Load);
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	
//--------------------------------------------------------------------------------------------------
//	フォームロード
//--------------------------------------------------------------------------------------------------
private: System::Void SingleAoDlg_Load(System::Object^  sender, System::EventArgs^  e) {
	//初期デバイス名の表示
	Text_Device->Text = "AIO000";

	//アナログ入力レンジ用コンボボックスの作成
	Combo_AioSetRange->Items->Add("-10 - +10V");
	Combo_AioSetRange->Items->Add("-5 - +5V");
	Combo_AioSetRange->Items->Add("-2.5 - +2.5V");
	Combo_AioSetRange->Items->Add("-1.25 - +1.25V");
	Combo_AioSetRange->Items->Add("-1 - +1V");
	Combo_AioSetRange->Items->Add("-0.625 - +0.625V");
	Combo_AioSetRange->Items->Add("-0.5 - +0.5V");
	Combo_AioSetRange->Items->Add("-0.3125 - +0.3125V");
	Combo_AioSetRange->Items->Add("-0.25 - +0.25V");
	Combo_AioSetRange->Items->Add("-0.125 - +0.125V");
	Combo_AioSetRange->Items->Add("-0.1 - +0.1V");
	Combo_AioSetRange->Items->Add("-0.05 - +0.05V");
	Combo_AioSetRange->Items->Add("-0.025 - +0.025V");
	Combo_AioSetRange->Items->Add("-0.0125 - +0.0125V");
	Combo_AioSetRange->Items->Add("-0.01 - +0.01V");
	Combo_AioSetRange->Items->Add("0 - +10V");
	Combo_AioSetRange->Items->Add("0 - +5V");
	Combo_AioSetRange->Items->Add("0 - +4.095V");
	Combo_AioSetRange->Items->Add("0 - +2.5V");
	Combo_AioSetRange->Items->Add("0 - +1.25V");
	Combo_AioSetRange->Items->Add("0 - +1V");
	Combo_AioSetRange->Items->Add("0 - +0.5V");
	Combo_AioSetRange->Items->Add("0 - +0.25V");
	Combo_AioSetRange->Items->Add("0 - +0.1V");
	Combo_AioSetRange->Items->Add("0 - +0.05V");
	Combo_AioSetRange->Items->Add("0 - +0.025V");
	Combo_AioSetRange->Items->Add("0 - +0.0125V");
	Combo_AioSetRange->Items->Add("0 - +0.01V");
 	Combo_AioSetRange->Items->Add("0 - +20mA");
 	Combo_AioSetRange->Items->Add("4 - +20mA");
	Combo_AioSetRange->Items->Add("1 - +5V");

	return;
}

//--------------------------------------------------------------------------------------------------
//	デバイスの初期化
//--------------------------------------------------------------------------------------------------
private: System::Void Button_AioInit_Click(System::Object^  sender, System::EventArgs^  e) {
	long	Ret;												//関数の戻り値
	long	Ret2;												//関数の戻り値
	StringBuilder	^ErrorString	= gcnew StringBuilder(256);	//エラーコード文字列

	//初期化処理
	String^	DeviceName;
	DeviceName = Text_Device->Text;
	pin_ptr<short>pId = &static_cast<short>(Id);
	Ret = AioInit(DeviceName, pId);
	if (Ret != 0){
		Ret2 = AioGetErrorString(Ret, ErrorString);
		Text_ErrorString->Text = "AioInit = " + Ret + " : " + ErrorString;
		return;
	}

	Text_ErrorString->Text = "初期化処理 : 正常終了";
}

//--------------------------------------------------------------------------------------------------
//	アナログ出力レンジの設定
//--------------------------------------------------------------------------------------------------
private: System::Void Combo_AioSetRange_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
	short	AoRange;
	short	Index;
	long	Ret;												//関数の戻り値
	long	Ret2;												//関数の戻り値
	StringBuilder	^ErrorString	= gcnew StringBuilder(256);	//エラーコード文字列

	Index = Combo_AioSetRange->SelectedIndex;
	switch(Index){
		case 0:	AoRange = PM10;		break;
		case 1:	AoRange = PM5;		break;
		case 2:	AoRange = PM25;		break;
		case 3:	AoRange = PM125;	break;
		case 4:	AoRange = PM1;		break;
		case 5:	AoRange = PM0625;	break;
		case 6:	AoRange = PM05;		break;
		case 7:	AoRange = PM03125;	break;
		case 8:	AoRange = PM025;	break;
		case 9:	AoRange = PM0125;	break;
		case 10:AoRange = PM01;		break;
		case 11:AoRange = PM005;	break;
		case 12:AoRange = PM0025;	break;
		case 13:AoRange = PM00125;	break;
		case 14:AoRange = PM001;	break;
		case 15:AoRange = P10;		break;
		case 16:AoRange = P5;		break;
		case 17:AoRange = P4095;	break;
		case 18:AoRange = P25;		break;
		case 19:AoRange = P125;		break;
		case 20:AoRange = P1;		break;
		case 21:AoRange = P05;		break;
		case 22:AoRange = P025;		break;
		case 23:AoRange = P01;		break;
		case 24:AoRange = P005;		break;
		case 25:AoRange = P0025;	break;
		case 26:AoRange = P00125;	break;
		case 27:AoRange = P001;		break;
		case 28:AoRange = P20MA;	break;
		case 29:AoRange = P4TO20MA;	break;
		case 30:AoRange = P1TO5;	break;
	}

	//出力レンジの設定
	Ret = AioSetAoRangeAll(Id, AoRange);
	if (Ret != 0){
		Ret2 = AioGetErrorString(Ret, ErrorString);
		Text_ErrorString->Text = "AioSetAoRangeAll = " + Ret + " : " + ErrorString;
		return;
	}

	if ((AoRange == P20MA) || (AoRange == P4TO20MA)){
		Label_Cdata->Text = "電流";
	}
	else{
		Label_Cdata->Text = "電圧";
	}

	Text_ErrorString->Text = "レンジ設定 : 正常終了";
}

//--------------------------------------------------------------------------------------------------
//	指定チャネルを1回アナログ出力
//--------------------------------------------------------------------------------------------------
private: System::Void Button__AioSingleAo_Click(System::Object^  sender, System::EventArgs^  e) {
	long	Ret;												//関数の戻り値
	long	Ret2;												//関数の戻り値
	StringBuilder	^ErrorString	= gcnew StringBuilder(256);	//エラーコード文字列

	//アナログ出力
	short	sAoChannel;
	float	sAoVolt;
	sAoChannel = Convert::ToInt16(Text_AoChannel->Text);
	sAoVolt = Convert::ToInt16(Text_Volt->Text);
	Ret = AioSingleAoEx(Id, sAoChannel, sAoVolt);
	if (Ret != 0){
		Ret2 = AioGetErrorString(Ret, ErrorString);
		Text_ErrorString->Text = "AioSingleAoEx = " + Ret + " : " + ErrorString;
		return;
	}

	Text_ErrorString->Text = "アナログ出力 : 正常終了";
}

//--------------------------------------------------------------------------------------------------
//	デバイスの終了処理
//--------------------------------------------------------------------------------------------------
private: System::Void Button_AioExit_Click(System::Object^  sender, System::EventArgs^  e) {
	long	Ret;												//関数の戻り値
	long	Ret2;												//関数の戻り値
	StringBuilder	^ErrorString	= gcnew StringBuilder(256);	//エラーコード文字列

	//デバイスの終了処理
	Ret = AioExit(Id);
	if (Ret != 0){
		Ret2 = AioGetErrorString(Ret, ErrorString);
		Text_ErrorString->Text = "AioExit = " + Ret + " : " + ErrorString;
		return;
	}

	Text_ErrorString->Text ="終了処理 : 正常終了";
}
};
}
