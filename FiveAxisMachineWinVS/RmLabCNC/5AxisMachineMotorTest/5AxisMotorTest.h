#pragma once

namespace My5AxisMachineMotorTest {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
//	using namespace System::Drawing::Color;
//	using namespace My5AxisMachineMotorTest::GUIMainProcess;
	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class MotorTestForm : public System::Windows::Forms::Form
	{
	public:
		MotorTestForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MotorTestForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  buttonStartSend;
	private: System::Windows::Forms::Button^  buttonStopSend;
	protected: 

	protected: 

	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisX;
	private: System::Windows::Forms::Label^  labelTorqueAxisX;

	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::TextBox^  textBoxRuntimeAxisX;
	private: System::Windows::Forms::Button^  buttonIOConection;
	private: System::Windows::Forms::Label^  labelConectStatus;


	private: System::Windows::Forms::Timer^  timerMotorTest;
	private: System::ComponentModel::IContainer^  components;
	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisY1;
	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisY2;
	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisA1;


	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisC;

	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisZ;
	private: System::Windows::Forms::HScrollBar^  hScrollBarAxisA2;
	private: System::Windows::Forms::Label^  labelTorqueAxisY1;
	private: System::Windows::Forms::Label^  labelTorqueAxisY2;
	private: System::Windows::Forms::Label^  labelTorqueAxisZ;
	private: System::Windows::Forms::Label^  labelTorqueAxisC;
	private: System::Windows::Forms::Label^  labelTorqueAxisA1;
	private: System::Windows::Forms::Label^  labelTorqueAxisA2;




	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
	private: System::Windows::Forms::TextBox^  textBoxConnectStatus;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->buttonStartSend = (gcnew System::Windows::Forms::Button());
			this->buttonStopSend = (gcnew System::Windows::Forms::Button());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->hScrollBarAxisX = (gcnew System::Windows::Forms::HScrollBar());
			this->labelTorqueAxisX = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->textBoxRuntimeAxisX = (gcnew System::Windows::Forms::TextBox());
			this->buttonIOConection = (gcnew System::Windows::Forms::Button());
			this->labelConectStatus = (gcnew System::Windows::Forms::Label());
			this->timerMotorTest = (gcnew System::Windows::Forms::Timer(this->components));
			this->textBoxConnectStatus = (gcnew System::Windows::Forms::TextBox());
			this->hScrollBarAxisY1 = (gcnew System::Windows::Forms::HScrollBar());
			this->hScrollBarAxisY2 = (gcnew System::Windows::Forms::HScrollBar());
			this->hScrollBarAxisA1 = (gcnew System::Windows::Forms::HScrollBar());
			this->hScrollBarAxisC = (gcnew System::Windows::Forms::HScrollBar());
			this->hScrollBarAxisZ = (gcnew System::Windows::Forms::HScrollBar());
			this->hScrollBarAxisA2 = (gcnew System::Windows::Forms::HScrollBar());
			this->labelTorqueAxisY1 = (gcnew System::Windows::Forms::Label());
			this->labelTorqueAxisY2 = (gcnew System::Windows::Forms::Label());
			this->labelTorqueAxisZ = (gcnew System::Windows::Forms::Label());
			this->labelTorqueAxisC = (gcnew System::Windows::Forms::Label());
			this->labelTorqueAxisA1 = (gcnew System::Windows::Forms::Label());
			this->labelTorqueAxisA2 = (gcnew System::Windows::Forms::Label());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// buttonStartSend
			// 
			this->buttonStartSend->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 24, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonStartSend->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->buttonStartSend->Location = System::Drawing::Point(108, 328);
			this->buttonStartSend->Name = L"buttonStartSend";
			this->buttonStartSend->Size = System::Drawing::Size(103, 56);
			this->buttonStartSend->TabIndex = 0;
			this->buttonStartSend->Text = L"Start";
			this->buttonStartSend->UseVisualStyleBackColor = true;
			this->buttonStartSend->Click += gcnew System::EventHandler(this, &MotorTestForm::buttonStartSend_Click);
			// 
			// buttonStopSend
			// 
			this->buttonStopSend->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 24, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonStopSend->ForeColor = System::Drawing::Color::Red;
			this->buttonStopSend->Location = System::Drawing::Point(309, 328);
			this->buttonStopSend->Name = L"buttonStopSend";
			this->buttonStopSend->Size = System::Drawing::Size(146, 56);
			this->buttonStopSend->TabIndex = 1;
			this->buttonStopSend->Text = L"Stop";
			this->buttonStopSend->UseVisualStyleBackColor = true;
			this->buttonStopSend->Click += gcnew System::EventHandler(this, &MotorTestForm::buttonStopSend_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->ImageLocation = L"5AxisMachineMiniLocked.jpg";
			this->pictureBox1->Location = System::Drawing::Point(693, 17);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(251, 231);
			this->pictureBox1->TabIndex = 2;
			this->pictureBox1->TabStop = false;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label1->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->label1->Location = System::Drawing::Point(12, 25);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(27, 25);
			this->label1->TabIndex = 3;
			this->label1->Text = L"X";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label2->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label2->Location = System::Drawing::Point(12, 59);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(41, 25);
			this->label2->TabIndex = 4;
			this->label2->Text = L"Y1";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label3->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label3->Location = System::Drawing::Point(12, 95);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(41, 25);
			this->label3->TabIndex = 5;
			this->label3->Text = L"Y2";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label4->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->label4->Location = System::Drawing::Point(12, 132);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(26, 25);
			this->label4->TabIndex = 6;
			this->label4->Text = L"Z";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label5->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label5->Location = System::Drawing::Point(12, 217);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(40, 25);
			this->label5->TabIndex = 7;
			this->label5->Text = L"A1";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label6->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label6->Location = System::Drawing::Point(12, 254);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(40, 25);
			this->label6->TabIndex = 8;
			this->label6->Text = L"A2";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label7->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->label7->Location = System::Drawing::Point(12, 172);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(28, 25);
			this->label7->TabIndex = 9;
			this->label7->Text = L"C";
			// 
			// hScrollBarAxisX
			// 
			this->hScrollBarAxisX->Location = System::Drawing::Point(58, 28);
			this->hScrollBarAxisX->Maximum = 36;
			this->hScrollBarAxisX->Minimum = -36;
			this->hScrollBarAxisX->Name = L"hScrollBarAxisX";
			this->hScrollBarAxisX->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisX->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisX->TabIndex = 11;
			this->hScrollBarAxisX->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisX_ValueChanged);
			// 
			// labelTorqueAxisX
			// 
			this->labelTorqueAxisX->AutoSize = true;
			this->labelTorqueAxisX->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisX->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisX->Location = System::Drawing::Point(551, 28);
			this->labelTorqueAxisX->Name = L"labelTorqueAxisX";
			this->labelTorqueAxisX->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisX->TabIndex = 12;
			this->labelTorqueAxisX->Text = L"0";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label9->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->label9->Location = System::Drawing::Point(576, 8);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(111, 20);
			this->label9->TabIndex = 13;
			this->label9->Text = L"Run Time (s)";
			// 
			// textBoxRuntimeAxisX
			// 
			this->textBoxRuntimeAxisX->Location = System::Drawing::Point(597, 32);
			this->textBoxRuntimeAxisX->Name = L"textBoxRuntimeAxisX";
			this->textBoxRuntimeAxisX->Size = System::Drawing::Size(76, 20);
			this->textBoxRuntimeAxisX->TabIndex = 14;
			this->textBoxRuntimeAxisX->Text = L"2";
			// 
			// buttonIOConection
			// 
			this->buttonIOConection->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonIOConection->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->buttonIOConection->Location = System::Drawing::Point(750, 320);
			this->buttonIOConection->Name = L"buttonIOConection";
			this->buttonIOConection->Size = System::Drawing::Size(163, 56);
			this->buttonIOConection->TabIndex = 15;
			this->buttonIOConection->Text = L"Conect to CNC";
			this->buttonIOConection->UseVisualStyleBackColor = true;
			this->buttonIOConection->Click += gcnew System::EventHandler(this, &MotorTestForm::buttonIOConection_Click);
			// 
			// labelConectStatus
			// 
			this->labelConectStatus->AutoSize = true;
			this->labelConectStatus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->labelConectStatus->ForeColor = System::Drawing::Color::Red;
			this->labelConectStatus->Location = System::Drawing::Point(328, 292);
			this->labelConectStatus->Name = L"labelConectStatus";
			this->labelConectStatus->Size = System::Drawing::Size(127, 16);
			this->labelConectStatus->TabIndex = 16;
			this->labelConectStatus->Text = L"Status: Disconected";
			// 
			// timerMotorTest
			// 
			this->timerMotorTest->Tick += gcnew System::EventHandler(this, &MotorTestForm::timerMotorTest_Tick);
			// 
			// textBoxConnectStatus
			// 
			this->textBoxConnectStatus->Location = System::Drawing::Point(703, 256);
			this->textBoxConnectStatus->Multiline = true;
			this->textBoxConnectStatus->Name = L"textBoxConnectStatus";
			this->textBoxConnectStatus->Size = System::Drawing::Size(232, 58);
			this->textBoxConnectStatus->TabIndex = 17;
			// 
			// hScrollBarAxisY1
			// 
			this->hScrollBarAxisY1->Location = System::Drawing::Point(58, 59);
			this->hScrollBarAxisY1->Maximum = 36;
			this->hScrollBarAxisY1->Minimum = -36;
			this->hScrollBarAxisY1->Name = L"hScrollBarAxisY1";
			this->hScrollBarAxisY1->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisY1->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisY1->TabIndex = 18;
			this->hScrollBarAxisY1->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisY1_ValueChanged);
			// 
			// hScrollBarAxisY2
			// 
			this->hScrollBarAxisY2->Location = System::Drawing::Point(58, 95);
			this->hScrollBarAxisY2->Maximum = 36;
			this->hScrollBarAxisY2->Minimum = -36;
			this->hScrollBarAxisY2->Name = L"hScrollBarAxisY2";
			this->hScrollBarAxisY2->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisY2->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisY2->TabIndex = 19;
			this->hScrollBarAxisY2->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisY2_ValueChanged);
			// 
			// hScrollBarAxisA1
			// 
			this->hScrollBarAxisA1->Location = System::Drawing::Point(58, 217);
			this->hScrollBarAxisA1->Maximum = 50;
			this->hScrollBarAxisA1->Minimum = -50;
			this->hScrollBarAxisA1->Name = L"hScrollBarAxisA1";
			this->hScrollBarAxisA1->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisA1->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisA1->TabIndex = 22;
			this->hScrollBarAxisA1->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisA1_ValueChanged);
			// 
			// hScrollBarAxisC
			// 
			this->hScrollBarAxisC->Location = System::Drawing::Point(58, 172);
			this->hScrollBarAxisC->Maximum = 20;
			this->hScrollBarAxisC->Minimum = -20;
			this->hScrollBarAxisC->Name = L"hScrollBarAxisC";
			this->hScrollBarAxisC->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisC->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisC->TabIndex = 21;
			this->hScrollBarAxisC->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisC_ValueChanged);
			// 
			// hScrollBarAxisZ
			// 
			this->hScrollBarAxisZ->Location = System::Drawing::Point(58, 132);
			this->hScrollBarAxisZ->Maximum = 36;
			this->hScrollBarAxisZ->Minimum = -36;
			this->hScrollBarAxisZ->Name = L"hScrollBarAxisZ";
			this->hScrollBarAxisZ->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisZ->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisZ->TabIndex = 20;
			this->hScrollBarAxisZ->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisZ_ValueChanged);
			// 
			// hScrollBarAxisA2
			// 
			this->hScrollBarAxisA2->Location = System::Drawing::Point(58, 256);
			this->hScrollBarAxisA2->Maximum = 50;
			this->hScrollBarAxisA2->Minimum = -50;
			this->hScrollBarAxisA2->Name = L"hScrollBarAxisA2";
			this->hScrollBarAxisA2->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->hScrollBarAxisA2->Size = System::Drawing::Size(490, 16);
			this->hScrollBarAxisA2->TabIndex = 23;
			this->hScrollBarAxisA2->ValueChanged += gcnew System::EventHandler(this, &MotorTestForm::hScrollBarAxisA2_ValueChanged);
			// 
			// labelTorqueAxisY1
			// 
			this->labelTorqueAxisY1->AutoSize = true;
			this->labelTorqueAxisY1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisY1->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisY1->Location = System::Drawing::Point(551, 53);
			this->labelTorqueAxisY1->Name = L"labelTorqueAxisY1";
			this->labelTorqueAxisY1->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisY1->TabIndex = 24;
			this->labelTorqueAxisY1->Text = L"0";
			// 
			// labelTorqueAxisY2
			// 
			this->labelTorqueAxisY2->AutoSize = true;
			this->labelTorqueAxisY2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisY2->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisY2->Location = System::Drawing::Point(551, 86);
			this->labelTorqueAxisY2->Name = L"labelTorqueAxisY2";
			this->labelTorqueAxisY2->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisY2->TabIndex = 25;
			this->labelTorqueAxisY2->Text = L"0";
			// 
			// labelTorqueAxisZ
			// 
			this->labelTorqueAxisZ->AutoSize = true;
			this->labelTorqueAxisZ->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisZ->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisZ->Location = System::Drawing::Point(551, 123);
			this->labelTorqueAxisZ->Name = L"labelTorqueAxisZ";
			this->labelTorqueAxisZ->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisZ->TabIndex = 26;
			this->labelTorqueAxisZ->Text = L"0";
			// 
			// labelTorqueAxisC
			// 
			this->labelTorqueAxisC->AutoSize = true;
			this->labelTorqueAxisC->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisC->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisC->Location = System::Drawing::Point(551, 163);
			this->labelTorqueAxisC->Name = L"labelTorqueAxisC";
			this->labelTorqueAxisC->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisC->TabIndex = 27;
			this->labelTorqueAxisC->Text = L"0";
			// 
			// labelTorqueAxisA1
			// 
			this->labelTorqueAxisA1->AutoSize = true;
			this->labelTorqueAxisA1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisA1->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisA1->Location = System::Drawing::Point(551, 208);
			this->labelTorqueAxisA1->Name = L"labelTorqueAxisA1";
			this->labelTorqueAxisA1->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisA1->TabIndex = 28;
			this->labelTorqueAxisA1->Text = L"0";
			// 
			// labelTorqueAxisA2
			// 
			this->labelTorqueAxisA2->AutoSize = true;
			this->labelTorqueAxisA2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTorqueAxisA2->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->labelTorqueAxisA2->Location = System::Drawing::Point(551, 247);
			this->labelTorqueAxisA2->Name = L"labelTorqueAxisA2";
			this->labelTorqueAxisA2->Size = System::Drawing::Size(25, 25);
			this->labelTorqueAxisA2->TabIndex = 29;
			this->labelTorqueAxisA2->Text = L"0";
			// 
			// MotorTestForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(967, 396);
			this->Controls->Add(this->labelTorqueAxisA2);
			this->Controls->Add(this->labelTorqueAxisA1);
			this->Controls->Add(this->labelTorqueAxisC);
			this->Controls->Add(this->labelTorqueAxisZ);
			this->Controls->Add(this->labelTorqueAxisY2);
			this->Controls->Add(this->labelTorqueAxisY1);
			this->Controls->Add(this->hScrollBarAxisA2);
			this->Controls->Add(this->hScrollBarAxisA1);
			this->Controls->Add(this->hScrollBarAxisC);
			this->Controls->Add(this->hScrollBarAxisZ);
			this->Controls->Add(this->hScrollBarAxisY2);
			this->Controls->Add(this->hScrollBarAxisY1);
			this->Controls->Add(this->textBoxConnectStatus);
			this->Controls->Add(this->labelConectStatus);
			this->Controls->Add(this->buttonIOConection);
			this->Controls->Add(this->textBoxRuntimeAxisX);
			this->Controls->Add(this->label9);
			this->Controls->Add(this->labelTorqueAxisX);
			this->Controls->Add(this->hScrollBarAxisX);
			this->Controls->Add(this->label7);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->buttonStopSend);
			this->Controls->Add(this->buttonStartSend);
			this->Name = L"MotorTestForm";
			this->Text = L"5AxisMotorTest";
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &MotorTestForm::MotorTestForm_FormClosed);
			this->Load += gcnew System::EventHandler(this, &MotorTestForm::MotorTestForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
public:
	GCHandle					hGC_Timer;
	CCntCLI::PTIMERCALLBACK^	dele_Timer;
	IntPtr						pTimer;
	int						hMainWnd;
private:
		/// <summary>
		/// Required designer variable.
		GUIMainProcess::GUIProcessing NewGUIProcessing;
		RmLabCNC::FiveAxisCNC  RmLabFiveAxisCNC;
		/// </summary>

private: System::Void buttonStartSend_Click(System::Object^  sender, System::EventArgs^  e) {
     System::String^ ErrorRp;

// 		 timerMotorTest->Interval = 50;
// 		 timerMotorTest->Start();
		 buttonStartSend->Enabled = 0;
		 buttonStopSend->Enabled = 1;
		 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
		 NewGUIProcessing.maxTime = System::Convert::ToDouble(textBoxRuntimeAxisX->Text);
		 NewGUIProcessing.SAMPLING_TIME = 0.05;// 100ms;
		 NewGUIProcessing.step = 0;
		 RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = NewGUIProcessing.SAMPLING_TIME;
//		  RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_X,3.0);

		 RmLabFiveAxisCNC.IOModule.StartCounter(pTimer,ErrorRp,this->hMainWnd);


// 		 CntRet	= CCntCLI::CntTimerCallbackProc(CntId, pTimer, nullptr);
// 		 CntRet = CCntCLI::CntNotifyTimer ( CntId , static_cast<unsigned long>(NewGUIProcessing.SAMPLING_TIME*1000),0);

	 }
private: System::Void timerMotorTest_Tick(System::Object^  sender, System::EventArgs^  e) {
				 labelTorqueAxisX->Text = hScrollBarAxisX->Value.ToString();
			 }
private: System::Void buttonStopSend_Click(System::Object^  sender, System::EventArgs^  e) {
			 System::String^ ErrorRp;
/*			 timerMotorTest->Stop();*/
			 buttonStartSend->Enabled = 1;
			 buttonStopSend->Enabled = 0;
			 RmLabFiveAxisCNC.IOModule.StopAllMotor();
			 RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);
			 NewGUIProcessing.step = 0;
			 hScrollBarAxisX->Value = 0;
			 hScrollBarAxisY1->Value = 0;
			 hScrollBarAxisY2->Value = 0;
			 hScrollBarAxisZ->Value = 0;
			 hScrollBarAxisC->Value = 0;
			 hScrollBarAxisA1->Value = 0;
			 hScrollBarAxisA2->Value = 0;

		 }
private: System::Void buttonIOConection_Click(System::Object^  sender, System::EventArgs^  e) {
			
			System::String^ ErrorRp;
//			 FiveAxisControlModule newFiveAxisControl;
//			 newFiveAxisControl.Control2D();
/*			 FiveAxisControl.Control2D();*/
			 if (NewGUIProcessing.bConnectStatus )
			 {
//				 NewIOModule.DisconnectToCNC(ErrorRp) ;
				 RmLabFiveAxisCNC.IOModule.DisconnectToCNC(ErrorRp);
				 if (ErrorRp == "OK")
				 {
					 textBoxConnectStatus->Text = "Disconnected to CNC";
					 textBoxConnectStatus->ForeColor =  SystemColors::ControlText.Red;
					 this->pictureBox1->ImageLocation = L"5AxisMachineMiniLocked.jpg";
					 buttonIOConection->Text = "Connect to CNC";
					 buttonIOConection->ForeColor = System::Drawing::Color::FromArgb(0, 192, 0);
					 NewGUIProcessing.bConnectStatus = !NewGUIProcessing.bConnectStatus;
				 }
				 else
				 {
					  textBoxConnectStatus->Text = "Error"+ErrorRp;
				 }

			 } 
			 else
			 {
	//			 labelConectStatus->Text = "Connecting to CNC";
	//			 labelConectStatus->ForeColor =  System::Drawing::Color::FromArgb(0, 192, 0);

				RmLabFiveAxisCNC.IOModule.ConnectToCNC(ErrorRp) ;
//				labelConectStatus->Text = labelConectStatus->Text +ErrorRp;
				if ( ErrorRp == "OK")
				{
					textBoxConnectStatus->Text = "Connecting to CNC";
					 this->pictureBox1->ImageLocation = L"5AxisMachineMini.jpg";
					 buttonIOConection->Text = "Disconnect to CNC";
					 buttonIOConection->ForeColor = SystemColors::ControlText.Red;
					 NewGUIProcessing.bConnectStatus = !NewGUIProcessing.bConnectStatus;
				} 
				else
				{
					 textBoxConnectStatus->Text = "Error"+ErrorRp;
				}
				  
			 } 
			 
		 }
void TimerCallBackProc(short m_Id, int wParam, int lParam, void * Param){
	System::String^ ErrorRp;
//            if (RmLabFiveAxisCNC.GcodeFinished())
//            {
// 			    RmLabFiveAxisCNC.IOModule.StopAllMotor();
// 				RmLabFiveAxisCNC.IOModule.StopCounter();
//            } 
// 		   else
// 		   {
// 			  RmLabFiveAxisCNC.FeedbackTimer();
// 		   }



	      NewGUIProcessing.time = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
	       textBoxConnectStatus->Text = String::Format("Time = {0:C2}",NewGUIProcessing.time);
	       RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_X,hScrollBarAxisX->Value);
		   RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_Y1,hScrollBarAxisY1->Value);
		   RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_Y2,hScrollBarAxisY2->Value);
		   RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_Z,hScrollBarAxisZ->Value);
		   RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_C,hScrollBarAxisC->Value/10.0);
		   RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_A1,hScrollBarAxisA1->Value/10.0);
		   RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_A2,hScrollBarAxisA2->Value/10.0);



		   if (NewGUIProcessing.time >NewGUIProcessing.maxTime)
		   {
			   buttonStartSend->Enabled = 1;
			   buttonStopSend->Enabled = 0;

			   hScrollBarAxisX->Value = 0;
			   hScrollBarAxisY1->Value = 0;
			   hScrollBarAxisY2->Value = 0;
			   hScrollBarAxisZ->Value = 0;
			   hScrollBarAxisC->Value = 0;
			   hScrollBarAxisA1->Value = 0;
			   hScrollBarAxisA2->Value = 0;

			   RmLabFiveAxisCNC.IOModule.StopAllMotor();
			   RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);
			   NewGUIProcessing.step = 0;
		   }
		   else
		   {
			   NewGUIProcessing.step = NewGUIProcessing.step +1;
		   }
		   
			 return;
		 }

private: System::Void MotorTestForm_Load(System::Object^  sender, System::EventArgs^  e) {
			 //-----------------------------------
			 // ウィンドウハンドルのコピー
			 //-----------------------------------
			 hMainWnd	= this->Handle.ToInt32();
			 //-----------------------------------
			 // デリゲート初期化
			 //-----------------------------------

			 dele_Timer	= gcnew CCntCLI::PTIMERCALLBACK(this, &MotorTestForm::TimerCallBackProc);
			 //-----------------------------------
			 // ガベージコレクションにより破棄されないようにデリゲートへの参照を追加
			 //-----------------------------------
			 hGC_Timer	= GCHandle::Alloc(dele_Timer);
			 NewGUIProcessing.bConnectStatus = 0;
			 NewGUIProcessing.time = 0.0;
			 NewGUIProcessing.maxTime = 3.0;
			 NewGUIProcessing.step = 0;
		 }
private: System::Void MotorTestForm_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) {
			 	System::String^ ErrorRp;
			  RmLabFiveAxisCNC.IOModule.DisconnectToCNC(ErrorRp);
			 //-----------------------------------
			 // ガベージコレクションにより破棄されないようにデリゲートへの参照を追加
			 //-----------------------------------

			 hGC_Timer	= GCHandle::Alloc(dele_Timer);
		 }
private: System::Void hScrollBarAxisX_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
			 		   labelTorqueAxisX->Text = hScrollBarAxisX->Value.ToString();
		 }
private: System::Void hScrollBarAxisY1_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		labelTorqueAxisY1->Text = hScrollBarAxisY1->Value.ToString();
		 }
private: System::Void hScrollBarAxisY2_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		 labelTorqueAxisY2->Text = hScrollBarAxisY2->Value.ToString();
		 }
private: System::Void hScrollBarAxisZ_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		 labelTorqueAxisZ->Text = hScrollBarAxisZ->Value.ToString();
		 }
private: System::Void hScrollBarAxisC_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		 labelTorqueAxisC->Text = hScrollBarAxisC->Value.ToString();
		 }
private: System::Void hScrollBarAxisA1_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		 labelTorqueAxisA1->Text = hScrollBarAxisA1->Value.ToString();
		 }
private: System::Void hScrollBarAxisA2_ValueChanged(System::Object^  sender, System::EventArgs^  e) {
		 labelTorqueAxisA2->Text = hScrollBarAxisA2->Value.ToString();
		 }
};
}

