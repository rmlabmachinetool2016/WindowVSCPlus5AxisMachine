#pragma once
#include "Identification.h"

namespace fiveaxis_machine_tool {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// FormIdentification の概要
	/// </summary>
	public ref class FormIdentification : public System::Windows::Forms::Form
	{
	public:
		FormIdentification(void)
		{
			InitializeComponent();
			//
			//TODO: ここにコンストラクター コードを追加します
			//
		}

	protected:
		/// <summary>
		/// 使用中のリソースをすべてクリーンアップします。
		/// </summary>
		~FormIdentification()
		{
			if (components)
			{
				delete components;
			}
		}

	protected: 
	private: System::Windows::Forms::Button^  buttonStart;
	private: System::Windows::Forms::RadioButton^  radioButtonX;
	private: System::Windows::Forms::TextBox^  textBoxTime;
	private: System::Windows::Forms::GroupBox^  groupBoxAxis;
	private: System::Windows::Forms::RadioButton^  radioButtonA1;
	private: System::Windows::Forms::RadioButton^  radioButtonC;
	private: System::Windows::Forms::RadioButton^  radioButtonZ;
	private: System::Windows::Forms::RadioButton^  radioButtonY2;
	private: System::Windows::Forms::RadioButton^  radioButtonY1;
	private: System::Windows::Forms::RadioButton^  radioButtonA2;
	private: System::Windows::Forms::TextBox^  textBoxNotify;
	private: System::Windows::Forms::Label^  labelSec;
	private: System::Windows::Forms::TextBox^  textBoxGain;
	private: System::Windows::Forms::TextBox^  textBoxInputFreq;
	private: System::Windows::Forms::TextBox^  textBoxCutOffFreq;
	private: System::Windows::Forms::Label^  gain;
	private: System::Windows::Forms::Label^  rads;
	private: System::Windows::Forms::Label^  cutoffreq;

	private:
		/// <summary>
		/// 必要なデザイナー変数です。
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// デザイナー サポートに必要なメソッドです。このメソッドの内容を
		/// コード エディターで変更しないでください。
		/// </summary>
		void InitializeComponent(void)
		{
			this->buttonStart = (gcnew System::Windows::Forms::Button());
			this->radioButtonX = (gcnew System::Windows::Forms::RadioButton());
			this->textBoxTime = (gcnew System::Windows::Forms::TextBox());
			this->groupBoxAxis = (gcnew System::Windows::Forms::GroupBox());
			this->radioButtonA2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonA1 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonC = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonZ = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonY2 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonY1 = (gcnew System::Windows::Forms::RadioButton());
			this->textBoxNotify = (gcnew System::Windows::Forms::TextBox());
			this->labelSec = (gcnew System::Windows::Forms::Label());
			this->textBoxGain = (gcnew System::Windows::Forms::TextBox());
			this->textBoxInputFreq = (gcnew System::Windows::Forms::TextBox());
			this->textBoxCutOffFreq = (gcnew System::Windows::Forms::TextBox());
			this->gain = (gcnew System::Windows::Forms::Label());
			this->rads = (gcnew System::Windows::Forms::Label());
			this->cutoffreq = (gcnew System::Windows::Forms::Label());
			this->groupBoxAxis->SuspendLayout();
			this->SuspendLayout();
			// 
			// buttonStart
			// 
			this->buttonStart->Location = System::Drawing::Point(115, 12);
			this->buttonStart->Name = L"buttonStart";
			this->buttonStart->Size = System::Drawing::Size(165, 48);
			this->buttonStart->TabIndex = 0;
			this->buttonStart->Text = L"Start";
			this->buttonStart->UseVisualStyleBackColor = true;
			this->buttonStart->Click += gcnew System::EventHandler(this, &FormIdentification::buttonStart_Click);
			// 
			// radioButtonX
			// 
			this->radioButtonX->AutoSize = true;
			this->radioButtonX->Checked = true;
			this->radioButtonX->Location = System::Drawing::Point(18, 24);
			this->radioButtonX->Name = L"radioButtonX";
			this->radioButtonX->Size = System::Drawing::Size(59, 16);
			this->radioButtonX->TabIndex = 1;
			this->radioButtonX->TabStop = true;
			this->radioButtonX->Text = L"X-Axis";
			this->radioButtonX->UseVisualStyleBackColor = true;
			// 
			// textBoxTime
			// 
			this->textBoxTime->Location = System::Drawing::Point(115, 66);
			this->textBoxTime->Name = L"textBoxTime";
			this->textBoxTime->Size = System::Drawing::Size(40, 19);
			this->textBoxTime->TabIndex = 2;
			this->textBoxTime->Text = L"10";
			// 
			// groupBoxAxis
			// 
			this->groupBoxAxis->Controls->Add(this->radioButtonA2);
			this->groupBoxAxis->Controls->Add(this->radioButtonA1);
			this->groupBoxAxis->Controls->Add(this->radioButtonC);
			this->groupBoxAxis->Controls->Add(this->radioButtonZ);
			this->groupBoxAxis->Controls->Add(this->radioButtonY2);
			this->groupBoxAxis->Controls->Add(this->radioButtonY1);
			this->groupBoxAxis->Controls->Add(this->radioButtonX);
			this->groupBoxAxis->Location = System::Drawing::Point(12, 12);
			this->groupBoxAxis->Name = L"groupBoxAxis";
			this->groupBoxAxis->Size = System::Drawing::Size(97, 188);
			this->groupBoxAxis->TabIndex = 3;
			this->groupBoxAxis->TabStop = false;
			this->groupBoxAxis->Text = L"SelectAxis";
			// 
			// radioButtonA2
			// 
			this->radioButtonA2->AutoSize = true;
			this->radioButtonA2->Location = System::Drawing::Point(17, 156);
			this->radioButtonA2->Name = L"radioButtonA2";
			this->radioButtonA2->Size = System::Drawing::Size(66, 16);
			this->radioButtonA2->TabIndex = 7;
			this->radioButtonA2->TabStop = true;
			this->radioButtonA2->Text = L"A2-Axis";
			this->radioButtonA2->UseVisualStyleBackColor = true;
			// 
			// radioButtonA1
			// 
			this->radioButtonA1->AutoSize = true;
			this->radioButtonA1->Location = System::Drawing::Point(18, 134);
			this->radioButtonA1->Name = L"radioButtonA1";
			this->radioButtonA1->Size = System::Drawing::Size(66, 16);
			this->radioButtonA1->TabIndex = 6;
			this->radioButtonA1->TabStop = true;
			this->radioButtonA1->Text = L"A1-Axis";
			this->radioButtonA1->UseVisualStyleBackColor = true;
			// 
			// radioButtonC
			// 
			this->radioButtonC->AutoSize = true;
			this->radioButtonC->Location = System::Drawing::Point(18, 112);
			this->radioButtonC->Name = L"radioButtonC";
			this->radioButtonC->Size = System::Drawing::Size(60, 16);
			this->radioButtonC->TabIndex = 5;
			this->radioButtonC->TabStop = true;
			this->radioButtonC->Text = L"C-Axis";
			this->radioButtonC->UseVisualStyleBackColor = true;
			// 
			// radioButtonZ
			// 
			this->radioButtonZ->AutoSize = true;
			this->radioButtonZ->Location = System::Drawing::Point(18, 90);
			this->radioButtonZ->Name = L"radioButtonZ";
			this->radioButtonZ->Size = System::Drawing::Size(59, 16);
			this->radioButtonZ->TabIndex = 4;
			this->radioButtonZ->TabStop = true;
			this->radioButtonZ->Text = L"Z-Axis";
			this->radioButtonZ->UseVisualStyleBackColor = true;
			// 
			// radioButtonY2
			// 
			this->radioButtonY2->AutoSize = true;
			this->radioButtonY2->Location = System::Drawing::Point(18, 68);
			this->radioButtonY2->Name = L"radioButtonY2";
			this->radioButtonY2->Size = System::Drawing::Size(65, 16);
			this->radioButtonY2->TabIndex = 3;
			this->radioButtonY2->TabStop = true;
			this->radioButtonY2->Text = L"Y2-Axis";
			this->radioButtonY2->UseVisualStyleBackColor = true;
			// 
			// radioButtonY1
			// 
			this->radioButtonY1->AutoSize = true;
			this->radioButtonY1->Location = System::Drawing::Point(18, 46);
			this->radioButtonY1->Name = L"radioButtonY1";
			this->radioButtonY1->Size = System::Drawing::Size(65, 16);
			this->radioButtonY1->TabIndex = 2;
			this->radioButtonY1->TabStop = true;
			this->radioButtonY1->Text = L"Y1-Axis";
			this->radioButtonY1->UseVisualStyleBackColor = true;
			// 
			// textBoxNotify
			// 
			this->textBoxNotify->Location = System::Drawing::Point(115, 124);
			this->textBoxNotify->Multiline = true;
			this->textBoxNotify->Name = L"textBoxNotify";
			this->textBoxNotify->ReadOnly = true;
			this->textBoxNotify->ScrollBars = System::Windows::Forms::ScrollBars::Vertical;
			this->textBoxNotify->Size = System::Drawing::Size(165, 76);
			this->textBoxNotify->TabIndex = 8;
			this->textBoxNotify->Text = L"Model Identification";
			// 
			// labelSec
			// 
			this->labelSec->AutoSize = true;
			this->labelSec->Location = System::Drawing::Point(162, 69);
			this->labelSec->Name = L"labelSec";
			this->labelSec->Size = System::Drawing::Size(23, 12);
			this->labelSec->TabIndex = 9;
			this->labelSec->Text = L"sec";
			// 
			// textBoxGain
			// 
			this->textBoxGain->Location = System::Drawing::Point(210, 66);
			this->textBoxGain->Name = L"textBoxGain";
			this->textBoxGain->Size = System::Drawing::Size(40, 19);
			this->textBoxGain->TabIndex = 10;
			this->textBoxGain->Text = L"0.2";
			// 
			// textBoxInputFreq
			// 
			this->textBoxInputFreq->Location = System::Drawing::Point(115, 94);
			this->textBoxInputFreq->Name = L"textBoxInputFreq";
			this->textBoxInputFreq->Size = System::Drawing::Size(40, 19);
			this->textBoxInputFreq->TabIndex = 11;
			this->textBoxInputFreq->Text = L"6.28";
			// 
			// textBoxCutOffFreq
			// 
			this->textBoxCutOffFreq->Location = System::Drawing::Point(210, 95);
			this->textBoxCutOffFreq->Name = L"textBoxCutOffFreq";
			this->textBoxCutOffFreq->Size = System::Drawing::Size(40, 19);
			this->textBoxCutOffFreq->TabIndex = 12;
			this->textBoxCutOffFreq->Text = L"2";
			// 
			// gain
			// 
			this->gain->AutoSize = true;
			this->gain->Location = System::Drawing::Point(256, 69);
			this->gain->Name = L"gain";
			this->gain->Size = System::Drawing::Size(26, 12);
			this->gain->TabIndex = 13;
			this->gain->Text = L"gain";
			// 
			// rads
			// 
			this->rads->AutoSize = true;
			this->rads->Location = System::Drawing::Point(162, 98);
			this->rads->Name = L"rads";
			this->rads->Size = System::Drawing::Size(33, 12);
			this->rads->TabIndex = 14;
			this->rads->Text = L"rad/s";
			// 
			// cutoffreq
			// 
			this->cutoffreq->AutoSize = true;
			this->cutoffreq->Location = System::Drawing::Point(256, 98);
			this->cutoffreq->Name = L"cutoffreq";
			this->cutoffreq->Size = System::Drawing::Size(33, 12);
			this->cutoffreq->TabIndex = 15;
			this->cutoffreq->Text = L"rad/s";
			// 
			// FormIdentification
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(292, 211);
			this->Controls->Add(this->cutoffreq);
			this->Controls->Add(this->rads);
			this->Controls->Add(this->gain);
			this->Controls->Add(this->textBoxCutOffFreq);
			this->Controls->Add(this->textBoxInputFreq);
			this->Controls->Add(this->textBoxGain);
			this->Controls->Add(this->labelSec);
			this->Controls->Add(this->textBoxNotify);
			this->Controls->Add(this->groupBoxAxis);
			this->Controls->Add(this->textBoxTime);
			this->Controls->Add(this->buttonStart);
			this->Name = L"FormIdentification";
			this->Text = L"FormIdentification";
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &FormIdentification::FormIdentification_FormClosed);
			this->Load += gcnew System::EventHandler(this, &FormIdentification::FormIdentification_Load);
			this->groupBoxAxis->ResumeLayout(false);
			this->groupBoxAxis->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

	public:
		unsigned int				selected_axis;
		bool						is_be_controling;
		bool						is_stop;
		short						CntId,AioId;
		long						AioRet,AioRet2,CntRet,CntRet2;
		double						time;
		array<double>				^ param;
		unsigned int				step,max_step;
		array<double,2>				^ datalog;
		unsigned int				i,j;
		String						^ AioDeviceName,^ CntDeviceName;
		GCHandle					hGC_Timer;
		CCntCLI::PTIMERCALLBACK^	dele_Timer;
		IntPtr						pTimer;

private: System::Void buttonStart_Click(System::Object^  sender, System::EventArgs^  e) {
			 using namespace CAioCLI;
			 using namespace CCntCLI;

			 StringBuilder	^ErrorString = gcnew StringBuilder(256);	//エラーコード文字列
			 pin_ptr<short>pAioId = &static_cast<short>(AioId);
			 pin_ptr<short>pCntId = &static_cast<short>(CntId);
			 short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};

			 if(is_be_controling)
				 return;
			 else
				 is_be_controling = true;

			 max_step = static_cast<unsigned int>(double::Parse(textBoxTime->Text) / SAMPLING_TIME);

			 if(radioButtonX->Checked)
				 selected_axis = 0;
			 else if(radioButtonY1->Checked)
				 selected_axis = 1;
			 else if(radioButtonY2->Checked)
				 selected_axis = 2;
			 else if(radioButtonZ->Checked)
				 selected_axis = 3;
			 else if(radioButtonC->Checked)
				 selected_axis = 4;
			 else if(radioButtonA1->Checked)
				 selected_axis = 5;
			 else
				 selected_axis = 6;

			 param[0] = double::Parse(textBoxGain->Text);
			 param[1] = double::Parse(textBoxInputFreq->Text);
			 param[2] = double::Parse(textBoxCutOffFreq->Text);

			 // データログ変数をクリア
			 for(i=0;i<NUM_DUMPDATA;i++){
				 for(j=0;j<MAX_COUNT;j++){
					 datalog[i,j] = 0;
				 }
			 }
			 step = 0;	// ステップの初期化
			 is_stop = false;

			 SetNotifyDelegate^ deleSetNotifyText = gcnew SetNotifyDelegate(this, &FormIdentification::SetNotifyText);
			 this->Invoke(deleSetNotifyText, "計測中(Don't close this window)");

			 // 初期化処理
			 AioRet = AioInit(AioDeviceName, pAioId);
			 CntRet = CntInit(CntDeviceName, pCntId);
			 // ＤＡボードのプロセスリセット
			 AioRet = AioResetProcess ( AioId );
			 // デバイスのリセット
			 AioRet = AioResetDevice(AioId);
			 CntRet = CntResetDevice(CntId);
			 // 出力レンジの設定
			 AioRet = AioSetAoRangeAll(AioId, PM10);
			 // カウンタ入力シグナルが差動であると指定
			 for(i=0;i<NUM_COUNTER;i++)
			 {
				 CntRet = CntSelectChannelSignal(CntId,i,CNT_SIGTYPE_LINERECEIVER);
				 CntRet = CntSetOperationMode(CntId,i,CNT_MODE_2PHASE,CNT_MUL_X4,CNT_CLR_ASYNC);
				 CntRet = CntSetZMode(CntId,i,CNT_ZPHASE_NOT_USE);
				 CntRet = CntSetCountDirection (CntId,i,CNT_DIR_UP);
			 }

			 CntRet = CntStartCount ( CntId , ChNo , NUM_COUNTER );
			 CntRet = CntZeroClearCount ( CntId , ChNo , NUM_COUNTER );

			 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
			 CntRet	= CntTimerCallbackProc(CntId, pTimer, nullptr);
			 CntRet = CntNotifyTimer ( CntId , static_cast<unsigned long>(SAMPLING_TIME*1000) , 0 );
		 }

private: System::Void FormIdentification_Load(System::Object^  sender, System::EventArgs^  e) {
				 is_be_controling = false;
				 datalog = gcnew array<double,2>(NUM_DUMPDATA,MAX_COUNT);
				 param = gcnew array<double>(3);

				 // デリゲート初期化
				 dele_Timer	= gcnew CCntCLI::PTIMERCALLBACK(this, &FormIdentification::TimerCallBackProc);
				 // ガベージコレクションにより破棄されないようにデリゲートへの参照を追加
				 hGC_Timer	= GCHandle::Alloc(dele_Timer);
		 }

private: void TimerCallBackProc(short m_Id, int wParam, int lParam, void * Param){		 
			 time = step * SAMPLING_TIME;
			 
			 Identification::Identification(step,AioId,CntId,selected_axis,param,datalog);

			 // 設定時間が経過したら制御停止
			 if(++step > max_step-1){
				 if(is_stop == false){
					 is_stop = true;
					 ControlStop();
				 }
			 }
		 }

private: void ControlStop(void){
			 using namespace CAioCLI;
			 using namespace CCntCLI;
			 
			 SetNotifyDelegate^ deleSetNotifyText = gcnew SetNotifyDelegate(this, &FormIdentification::SetNotifyText);
			 short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};

			 CntRet = CntStopNotifyTimer ( CntId );
			 // Stoping counter board and DA board
			 CntRet = CntStopCount ( CntId , ChNo , NUM_COUNTER );
			 for(i=0;i<NUM_ACTUATOR;i++)
				 AioRet = AioSingleAoEx(AioId,i,0);
			 // ＤＡボードのクローズ
			 AioRet = AioExit(AioId);
			 // カウンタボードのクローズ
			 CntRet = CntExit(CntId);

			 is_be_controling = false;

			 double M,Fri;
			 Identification::CalcModelParam(M,Fri,selected_axis,param,datalog);
			 String^ str = "";
			 str += "M :"+M.ToString("f12")+"\r\n";
			 str += "C :"+Fri.ToString("f12")+"\r\n";
			 
			 this->Invoke(deleSetNotifyText, str);
		 }

delegate void SetNotifyDelegate(String^ str);
void SetNotifyText(String^ str){textBoxNotify->Text = str;}

private: System::Void FormIdentification_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) {hGC_Timer.Free();}
};
}
