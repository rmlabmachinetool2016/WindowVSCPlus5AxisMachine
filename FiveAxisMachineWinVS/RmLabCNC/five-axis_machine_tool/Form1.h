#pragma once
#include "Form2.h"
#include <boost/timer.hpp>

namespace fiveaxis_machine_tool {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Form1 の概要
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
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
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}
	
	protected: 	
	private: System::Windows::Forms::Button^  ButtonStart;
	private: System::Windows::Forms::Button^  buttonGraph;
	private: System::Windows::Forms::Button^  buttonDump;
	private: System::Windows::Forms::TextBox^  textBoxPoleX;
	private: System::Windows::Forms::TextBox^  textBoxMassX;
	private: System::Windows::Forms::TextBox^  textBoxFrictionX;
	private: System::Windows::Forms::TextBox^  textBoxPoleY1;
	private: System::Windows::Forms::TextBox^  textBoxMassY1;
	private: System::Windows::Forms::TextBox^  textBoxFrictionY1;
	private: System::Windows::Forms::TextBox^  textBoxPoleY2;
	private: System::Windows::Forms::TextBox^  textBoxMassY2;
	private: System::Windows::Forms::TextBox^  textBoxFrictionY2;
	private: System::Windows::Forms::TextBox^  textBoxPoleZ;
	private: System::Windows::Forms::TextBox^  textBoxMassZ;
	private: System::Windows::Forms::TextBox^  textBoxFrictionZ;
	private: System::Windows::Forms::TextBox^  textBoxPoleC;
	private: System::Windows::Forms::TextBox^  textBoxInertiaC;
	private: System::Windows::Forms::TextBox^  textBoxFrictionC;
	private: System::Windows::Forms::TextBox^  textBoxPoleA1;
	private: System::Windows::Forms::TextBox^  textBoxInertiaA1;
	private: System::Windows::Forms::TextBox^  textBoxFrictionA1;
	private: System::Windows::Forms::TextBox^  textBoxPoleA2;
	private: System::Windows::Forms::TextBox^  textBoxInertiaA2;
	private: System::Windows::Forms::TextBox^  textBoxFrictionA2;
	private: System::Windows::Forms::CheckBox^  checkBoxNoise;
	private: System::Windows::Forms::CheckBox^  checkBoxResolution;
	private: System::Windows::Forms::RadioButton^  radioButtonSelectInd;
	private: System::Windows::Forms::RadioButton^  radioButtonSelectCont;
	private: System::Windows::Forms::RadioButton^  radioButtonSelectmodCont;
	private: System::Windows::Forms::RadioButton^  radioButtonSelectSim;
	private: System::Windows::Forms::RadioButton^  radioButtonSelectExp;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::GroupBox^  groupBox6;
	private: System::Windows::Forms::GroupBox^  groupBox7;
	private: System::Windows::Forms::GroupBox^  groupBox8;
	private: System::Windows::Forms::GroupBox^  groupBox9;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label3;	
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::Label^  label16;
	private: System::Windows::Forms::Label^  label17;
	private: System::Windows::Forms::Label^  label18;
	private: System::Windows::Forms::Label^  label19;
	private: System::Windows::Forms::Label^  label20;
	private: System::Windows::Forms::Label^  label21;
	private: System::Windows::Forms::TextBox^  textBoxProfile;
	private: System::Windows::Forms::Button^  BottunStop;
	private: System::Windows::Forms::RadioButton^  radioTime;
	private: System::Windows::Forms::RadioButton^  radioAngle;
	private: System::Windows::Forms::GroupBox^  groupBox10;
	private: System::Windows::Forms::TextBox^  textBoxMaxCount;
	private: System::Windows::Forms::TextBox^  textBoxRad;
	private: System::Windows::Forms::Label^  label23;
	private: System::Windows::Forms::Label^  label22;
	private: System::Windows::Forms::TextBox^  AIOText;
	private: System::Windows::Forms::TextBox^  CNTText;
	private: System::Windows::Forms::Label^  AIOName;
	private: System::Windows::Forms::Label^  CNTName;
	private: System::Windows::Forms::Label^  label26;
	private: System::Windows::Forms::TextBox^  textBoxPoleB;
	private: System::Windows::Forms::TextBox^  textBoxPoleT;
	private: System::Windows::Forms::TextBox^  textBoxPoleN;
	private: System::Windows::Forms::Label^  label24;
	private: System::Windows::Forms::GroupBox^  groupBox11;
	private: System::Windows::Forms::Label^  label25;
	private: System::Windows::Forms::GroupBox^  groupBox12;
	private: System::Windows::Forms::Label^  label27;
	private: System::Windows::Forms::TextBox^  textBoxPoleSyncA;
	private: System::Windows::Forms::TextBox^  textBoxPoleSyncY;
	private: System::Windows::Forms::Label^  label28;
	private: System::Windows::Forms::CheckBox^  checkBoxSync;
	private: System::Windows::Forms::ProgressBar^  progressBar1;
	private: System::Windows::Forms::Label^  label29;
	private: System::Windows::Forms::TextBox^  textBoxPoleJ;
	private: System::Windows::Forms::TextBox^  textBoxPoleI;
	private: System::Windows::Forms::Label^  label30;
	private: System::Windows::Forms::TextBox^  textBoxPoleK;
	private: System::Windows::Forms::Label^  label31;
	private: System::Windows::Forms::GroupBox^  groupBox13;
	private: System::Windows::Forms::RadioButton^  radioButtonSelect3D;
	private: System::Windows::Forms::RadioButton^  radioButtonSelect2D;
	private: System::Windows::Forms::Button^  ButtonIdentification;
private: System::Windows::Forms::Label^  label32Test;

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
			this->ButtonStart = (gcnew System::Windows::Forms::Button());
			this->textBoxPoleX = (gcnew System::Windows::Forms::TextBox());
			this->textBoxMassX = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionX = (gcnew System::Windows::Forms::TextBox());
			this->checkBoxNoise = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxResolution = (gcnew System::Windows::Forms::CheckBox());
			this->radioButtonSelectInd = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonSelectCont = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonSelectmodCont = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->textBoxMassY1 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleY1 = (gcnew System::Windows::Forms::TextBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionY1 = (gcnew System::Windows::Forms::TextBox());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->textBoxMassY2 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleY2 = (gcnew System::Windows::Forms::TextBox());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionY2 = (gcnew System::Windows::Forms::TextBox());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->textBoxMassZ = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleZ = (gcnew System::Windows::Forms::TextBox());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionZ = (gcnew System::Windows::Forms::TextBox());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->textBoxInertiaC = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleC = (gcnew System::Windows::Forms::TextBox());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionC = (gcnew System::Windows::Forms::TextBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->textBoxInertiaA1 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleA1 = (gcnew System::Windows::Forms::TextBox());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionA1 = (gcnew System::Windows::Forms::TextBox());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->textBoxInertiaA2 = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleA2 = (gcnew System::Windows::Forms::TextBox());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->textBoxFrictionA2 = (gcnew System::Windows::Forms::TextBox());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->radioButtonSelectSim = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonSelectExp = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox8 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
			this->buttonGraph = (gcnew System::Windows::Forms::Button());
			this->buttonDump = (gcnew System::Windows::Forms::Button());
			this->textBoxProfile = (gcnew System::Windows::Forms::TextBox());
			this->BottunStop = (gcnew System::Windows::Forms::Button());
			this->radioTime = (gcnew System::Windows::Forms::RadioButton());
			this->radioAngle = (gcnew System::Windows::Forms::RadioButton());
			this->groupBox10 = (gcnew System::Windows::Forms::GroupBox());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->textBoxRad = (gcnew System::Windows::Forms::TextBox());
			this->textBoxMaxCount = (gcnew System::Windows::Forms::TextBox());
			this->AIOText = (gcnew System::Windows::Forms::TextBox());
			this->CNTText = (gcnew System::Windows::Forms::TextBox());
			this->AIOName = (gcnew System::Windows::Forms::Label());
			this->CNTName = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->textBoxPoleB = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleT = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleN = (gcnew System::Windows::Forms::TextBox());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->groupBox11 = (gcnew System::Windows::Forms::GroupBox());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->textBoxPoleJ = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleI = (gcnew System::Windows::Forms::TextBox());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->textBoxPoleK = (gcnew System::Windows::Forms::TextBox());
			this->label31 = (gcnew System::Windows::Forms::Label());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->groupBox12 = (gcnew System::Windows::Forms::GroupBox());
			this->checkBoxSync = (gcnew System::Windows::Forms::CheckBox());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->textBoxPoleSyncA = (gcnew System::Windows::Forms::TextBox());
			this->textBoxPoleSyncY = (gcnew System::Windows::Forms::TextBox());
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->progressBar1 = (gcnew System::Windows::Forms::ProgressBar());
			this->groupBox13 = (gcnew System::Windows::Forms::GroupBox());
			this->radioButtonSelect3D = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonSelect2D = (gcnew System::Windows::Forms::RadioButton());
			this->ButtonIdentification = (gcnew System::Windows::Forms::Button());
			this->label32Test = (gcnew System::Windows::Forms::Label());
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->groupBox3->SuspendLayout();
			this->groupBox4->SuspendLayout();
			this->groupBox5->SuspendLayout();
			this->groupBox6->SuspendLayout();
			this->groupBox7->SuspendLayout();
			this->groupBox8->SuspendLayout();
			this->groupBox9->SuspendLayout();
			this->groupBox10->SuspendLayout();
			this->groupBox11->SuspendLayout();
			this->groupBox12->SuspendLayout();
			this->groupBox13->SuspendLayout();
			this->SuspendLayout();
			// 
			// ButtonStart
			// 
			this->ButtonStart->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->ButtonStart->Location = System::Drawing::Point(634, 12);
			this->ButtonStart->Name = L"ButtonStart";
			this->ButtonStart->Size = System::Drawing::Size(148, 52);
			this->ButtonStart->TabIndex = 0;
			this->ButtonStart->Text = L"Start";
			this->ButtonStart->UseVisualStyleBackColor = true;
			this->ButtonStart->Click += gcnew System::EventHandler(this, &Form1::ButtonStart_Click);
			// 
			// textBoxPoleX
			// 
			this->textBoxPoleX->Cursor = System::Windows::Forms::Cursors::IBeam;
			this->textBoxPoleX->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleX->Name = L"textBoxPoleX";
			this->textBoxPoleX->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleX->TabIndex = 1;
			// 
			// textBoxMassX
			// 
			this->textBoxMassX->Location = System::Drawing::Point(220, 18);
			this->textBoxMassX->Name = L"textBoxMassX";
			this->textBoxMassX->Size = System::Drawing::Size(70, 19);
			this->textBoxMassX->TabIndex = 4;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(36, 21);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(27, 12);
			this->label1->TabIndex = 15;
			this->label1->Text = L"Pole";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(181, 21);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(32, 12);
			this->label9->TabIndex = 21;
			this->label9->Text = L"Mass";
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Location = System::Drawing::Point(334, 21);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(44, 12);
			this->label21->TabIndex = 36;
			this->label21->Text = L"Friction";
			// 
			// textBoxFrictionX
			// 
			this->textBoxFrictionX->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionX->Name = L"textBoxFrictionX";
			this->textBoxFrictionX->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionX->TabIndex = 29;
			// 
			// checkBoxNoise
			// 
			this->checkBoxNoise->AutoSize = true;
			this->checkBoxNoise->Location = System::Drawing::Point(520, 75);
			this->checkBoxNoise->Name = L"checkBoxNoise";
			this->checkBoxNoise->Size = System::Drawing::Size(91, 16);
			this->checkBoxNoise->TabIndex = 43;
			this->checkBoxNoise->Text = L"Enable Noise";
			this->checkBoxNoise->UseVisualStyleBackColor = true;
			this->checkBoxNoise->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxNoise_CheckedChanged);
			// 
			// checkBoxResolution
			// 
			this->checkBoxResolution->AutoSize = true;
			this->checkBoxResolution->Location = System::Drawing::Point(520, 98);
			this->checkBoxResolution->Name = L"checkBoxResolution";
			this->checkBoxResolution->Size = System::Drawing::Size(116, 16);
			this->checkBoxResolution->TabIndex = 44;
			this->checkBoxResolution->Text = L"Enable Resolution";
			this->checkBoxResolution->UseVisualStyleBackColor = true;
			this->checkBoxResolution->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxResolution_CheckedChanged);
			// 
			// radioButtonSelectInd
			// 
			this->radioButtonSelectInd->AutoSize = true;
			this->radioButtonSelectInd->Checked = true;
			this->radioButtonSelectInd->Location = System::Drawing::Point(10, 17);
			this->radioButtonSelectInd->Name = L"radioButtonSelectInd";
			this->radioButtonSelectInd->Size = System::Drawing::Size(110, 17);
			this->radioButtonSelectInd->TabIndex = 45;
			this->radioButtonSelectInd->TabStop = true;
			this->radioButtonSelectInd->Text = L"Independent Con.";
			this->radioButtonSelectInd->UseVisualStyleBackColor = true;
			this->radioButtonSelectInd->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButtonSelectInd_CheckedChanged);
			// 
			// radioButtonSelectCont
			// 
			this->radioButtonSelectCont->AutoSize = true;
			this->radioButtonSelectCont->Location = System::Drawing::Point(10, 39);
			this->radioButtonSelectCont->Name = L"radioButtonSelectCont";
			this->radioButtonSelectCont->Size = System::Drawing::Size(101, 17);
			this->radioButtonSelectCont->TabIndex = 46;
			this->radioButtonSelectCont->Text = L"Contouring Con.";
			this->radioButtonSelectCont->UseVisualStyleBackColor = true;
			// 
			// radioButtonSelectmodCont
			// 
			this->radioButtonSelectmodCont->AutoSize = true;
			this->radioButtonSelectmodCont->Location = System::Drawing::Point(10, 61);
			this->radioButtonSelectmodCont->Name = L"radioButtonSelectmodCont";
			this->radioButtonSelectmodCont->Size = System::Drawing::Size(121, 17);
			this->radioButtonSelectmodCont->TabIndex = 47;
			this->radioButtonSelectmodCont->Text = L"modContouring Con.";
			this->radioButtonSelectmodCont->UseVisualStyleBackColor = true;
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->textBoxMassX);
			this->groupBox1->Controls->Add(this->textBoxPoleX);
			this->groupBox1->Controls->Add(this->label9);
			this->groupBox1->Controls->Add(this->textBoxFrictionX);
			this->groupBox1->Controls->Add(this->label21);
			this->groupBox1->Location = System::Drawing::Point(5, 10);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(470, 47);
			this->groupBox1->TabIndex = 48;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"X-Axis";
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->label2);
			this->groupBox2->Controls->Add(this->textBoxMassY1);
			this->groupBox2->Controls->Add(this->textBoxPoleY1);
			this->groupBox2->Controls->Add(this->label3);
			this->groupBox2->Controls->Add(this->textBoxFrictionY1);
			this->groupBox2->Controls->Add(this->label4);
			this->groupBox2->Location = System::Drawing::Point(5, 63);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(470, 47);
			this->groupBox2->TabIndex = 49;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Y1-Axis";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(36, 21);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(27, 12);
			this->label2->TabIndex = 15;
			this->label2->Text = L"Pole";
			// 
			// textBoxMassY1
			// 
			this->textBoxMassY1->Location = System::Drawing::Point(220, 18);
			this->textBoxMassY1->Name = L"textBoxMassY1";
			this->textBoxMassY1->Size = System::Drawing::Size(70, 19);
			this->textBoxMassY1->TabIndex = 4;
			// 
			// textBoxPoleY1
			// 
			this->textBoxPoleY1->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleY1->Name = L"textBoxPoleY1";
			this->textBoxPoleY1->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleY1->TabIndex = 1;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(181, 21);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(32, 12);
			this->label3->TabIndex = 21;
			this->label3->Text = L"Mass";
			// 
			// textBoxFrictionY1
			// 
			this->textBoxFrictionY1->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionY1->Name = L"textBoxFrictionY1";
			this->textBoxFrictionY1->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionY1->TabIndex = 29;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(334, 21);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(44, 12);
			this->label4->TabIndex = 36;
			this->label4->Text = L"Friction";
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->label5);
			this->groupBox3->Controls->Add(this->textBoxMassY2);
			this->groupBox3->Controls->Add(this->textBoxPoleY2);
			this->groupBox3->Controls->Add(this->label6);
			this->groupBox3->Controls->Add(this->textBoxFrictionY2);
			this->groupBox3->Controls->Add(this->label7);
			this->groupBox3->Location = System::Drawing::Point(5, 116);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(470, 47);
			this->groupBox3->TabIndex = 49;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Y2-Axis";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(36, 21);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(27, 12);
			this->label5->TabIndex = 15;
			this->label5->Text = L"Pole";
			// 
			// textBoxMassY2
			// 
			this->textBoxMassY2->Location = System::Drawing::Point(220, 18);
			this->textBoxMassY2->Name = L"textBoxMassY2";
			this->textBoxMassY2->Size = System::Drawing::Size(70, 19);
			this->textBoxMassY2->TabIndex = 4;
			// 
			// textBoxPoleY2
			// 
			this->textBoxPoleY2->Enabled = false;
			this->textBoxPoleY2->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleY2->Name = L"textBoxPoleY2";
			this->textBoxPoleY2->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleY2->TabIndex = 1;
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(181, 21);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(32, 12);
			this->label6->TabIndex = 21;
			this->label6->Text = L"Mass";
			// 
			// textBoxFrictionY2
			// 
			this->textBoxFrictionY2->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionY2->Name = L"textBoxFrictionY2";
			this->textBoxFrictionY2->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionY2->TabIndex = 29;
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(334, 21);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(44, 12);
			this->label7->TabIndex = 36;
			this->label7->Text = L"Friction";
			// 
			// groupBox4
			// 
			this->groupBox4->Controls->Add(this->label8);
			this->groupBox4->Controls->Add(this->textBoxMassZ);
			this->groupBox4->Controls->Add(this->textBoxPoleZ);
			this->groupBox4->Controls->Add(this->label10);
			this->groupBox4->Controls->Add(this->textBoxFrictionZ);
			this->groupBox4->Controls->Add(this->label11);
			this->groupBox4->Location = System::Drawing::Point(5, 169);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(470, 47);
			this->groupBox4->TabIndex = 49;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"Z-Axis";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(36, 21);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(27, 12);
			this->label8->TabIndex = 15;
			this->label8->Text = L"Pole";
			// 
			// textBoxMassZ
			// 
			this->textBoxMassZ->Location = System::Drawing::Point(220, 18);
			this->textBoxMassZ->Name = L"textBoxMassZ";
			this->textBoxMassZ->Size = System::Drawing::Size(70, 19);
			this->textBoxMassZ->TabIndex = 4;
			// 
			// textBoxPoleZ
			// 
			this->textBoxPoleZ->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleZ->Name = L"textBoxPoleZ";
			this->textBoxPoleZ->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleZ->TabIndex = 1;
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(181, 21);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(32, 12);
			this->label10->TabIndex = 21;
			this->label10->Text = L"Mass";
			// 
			// textBoxFrictionZ
			// 
			this->textBoxFrictionZ->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionZ->Name = L"textBoxFrictionZ";
			this->textBoxFrictionZ->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionZ->TabIndex = 29;
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(334, 21);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(44, 12);
			this->label11->TabIndex = 36;
			this->label11->Text = L"Friction";
			// 
			// groupBox5
			// 
			this->groupBox5->Controls->Add(this->label12);
			this->groupBox5->Controls->Add(this->textBoxInertiaC);
			this->groupBox5->Controls->Add(this->textBoxPoleC);
			this->groupBox5->Controls->Add(this->label13);
			this->groupBox5->Controls->Add(this->textBoxFrictionC);
			this->groupBox5->Controls->Add(this->label14);
			this->groupBox5->Location = System::Drawing::Point(5, 222);
			this->groupBox5->Name = L"groupBox5";
			this->groupBox5->Size = System::Drawing::Size(470, 47);
			this->groupBox5->TabIndex = 49;
			this->groupBox5->TabStop = false;
			this->groupBox5->Text = L"C-Axis";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(36, 21);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(27, 12);
			this->label12->TabIndex = 15;
			this->label12->Text = L"Pole";
			// 
			// textBoxInertiaC
			// 
			this->textBoxInertiaC->Location = System::Drawing::Point(220, 18);
			this->textBoxInertiaC->Name = L"textBoxInertiaC";
			this->textBoxInertiaC->Size = System::Drawing::Size(70, 19);
			this->textBoxInertiaC->TabIndex = 4;
			// 
			// textBoxPoleC
			// 
			this->textBoxPoleC->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleC->Name = L"textBoxPoleC";
			this->textBoxPoleC->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleC->TabIndex = 1;
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(181, 21);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(37, 12);
			this->label13->TabIndex = 21;
			this->label13->Text = L"Inertia";
			// 
			// textBoxFrictionC
			// 
			this->textBoxFrictionC->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionC->Name = L"textBoxFrictionC";
			this->textBoxFrictionC->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionC->TabIndex = 29;
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(334, 21);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(44, 12);
			this->label14->TabIndex = 36;
			this->label14->Text = L"Friction";
			// 
			// groupBox6
			// 
			this->groupBox6->Controls->Add(this->label15);
			this->groupBox6->Controls->Add(this->textBoxInertiaA1);
			this->groupBox6->Controls->Add(this->textBoxPoleA1);
			this->groupBox6->Controls->Add(this->label16);
			this->groupBox6->Controls->Add(this->textBoxFrictionA1);
			this->groupBox6->Controls->Add(this->label17);
			this->groupBox6->Location = System::Drawing::Point(5, 275);
			this->groupBox6->Name = L"groupBox6";
			this->groupBox6->Size = System::Drawing::Size(470, 47);
			this->groupBox6->TabIndex = 50;
			this->groupBox6->TabStop = false;
			this->groupBox6->Text = L"A1-Axis";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(36, 21);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(27, 12);
			this->label15->TabIndex = 15;
			this->label15->Text = L"Pole";
			// 
			// textBoxInertiaA1
			// 
			this->textBoxInertiaA1->Location = System::Drawing::Point(220, 18);
			this->textBoxInertiaA1->Name = L"textBoxInertiaA1";
			this->textBoxInertiaA1->Size = System::Drawing::Size(70, 19);
			this->textBoxInertiaA1->TabIndex = 4;
			// 
			// textBoxPoleA1
			// 
			this->textBoxPoleA1->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleA1->Name = L"textBoxPoleA1";
			this->textBoxPoleA1->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleA1->TabIndex = 1;
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(181, 21);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(37, 12);
			this->label16->TabIndex = 21;
			this->label16->Text = L"Inertia";
			// 
			// textBoxFrictionA1
			// 
			this->textBoxFrictionA1->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionA1->Name = L"textBoxFrictionA1";
			this->textBoxFrictionA1->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionA1->TabIndex = 29;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(334, 21);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(44, 12);
			this->label17->TabIndex = 36;
			this->label17->Text = L"Friction";
			// 
			// groupBox7
			// 
			this->groupBox7->Controls->Add(this->label18);
			this->groupBox7->Controls->Add(this->textBoxInertiaA2);
			this->groupBox7->Controls->Add(this->textBoxPoleA2);
			this->groupBox7->Controls->Add(this->label19);
			this->groupBox7->Controls->Add(this->textBoxFrictionA2);
			this->groupBox7->Controls->Add(this->label20);
			this->groupBox7->Location = System::Drawing::Point(5, 328);
			this->groupBox7->Name = L"groupBox7";
			this->groupBox7->Size = System::Drawing::Size(470, 47);
			this->groupBox7->TabIndex = 51;
			this->groupBox7->TabStop = false;
			this->groupBox7->Text = L"A2-Axis";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(36, 21);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(27, 12);
			this->label18->TabIndex = 15;
			this->label18->Text = L"Pole";
			// 
			// textBoxInertiaA2
			// 
			this->textBoxInertiaA2->Location = System::Drawing::Point(220, 18);
			this->textBoxInertiaA2->Name = L"textBoxInertiaA2";
			this->textBoxInertiaA2->Size = System::Drawing::Size(70, 19);
			this->textBoxInertiaA2->TabIndex = 4;
			// 
			// textBoxPoleA2
			// 
			this->textBoxPoleA2->Enabled = false;
			this->textBoxPoleA2->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleA2->Name = L"textBoxPoleA2";
			this->textBoxPoleA2->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleA2->TabIndex = 1;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Location = System::Drawing::Point(181, 21);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(37, 12);
			this->label19->TabIndex = 21;
			this->label19->Text = L"Inertia";
			// 
			// textBoxFrictionA2
			// 
			this->textBoxFrictionA2->Location = System::Drawing::Point(386, 18);
			this->textBoxFrictionA2->Name = L"textBoxFrictionA2";
			this->textBoxFrictionA2->Size = System::Drawing::Size(70, 19);
			this->textBoxFrictionA2->TabIndex = 29;
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Location = System::Drawing::Point(334, 21);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(44, 12);
			this->label20->TabIndex = 36;
			this->label20->Text = L"Friction";
			// 
			// radioButtonSelectSim
			// 
			this->radioButtonSelectSim->AutoSize = true;
			this->radioButtonSelectSim->Checked = true;
			this->radioButtonSelectSim->Location = System::Drawing::Point(11, 18);
			this->radioButtonSelectSim->Name = L"radioButtonSelectSim";
			this->radioButtonSelectSim->Size = System::Drawing::Size(73, 17);
			this->radioButtonSelectSim->TabIndex = 53;
			this->radioButtonSelectSim->TabStop = true;
			this->radioButtonSelectSim->Text = L"Simulation";
			this->radioButtonSelectSim->UseVisualStyleBackColor = true;
			this->radioButtonSelectSim->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButtonSelectSim_CheckedChanged);
			// 
			// radioButtonSelectExp
			// 
			this->radioButtonSelectExp->AutoSize = true;
			this->radioButtonSelectExp->Location = System::Drawing::Point(11, 40);
			this->radioButtonSelectExp->Name = L"radioButtonSelectExp";
			this->radioButtonSelectExp->Size = System::Drawing::Size(77, 17);
			this->radioButtonSelectExp->TabIndex = 54;
			this->radioButtonSelectExp->Text = L"Experiment";
			this->radioButtonSelectExp->UseVisualStyleBackColor = true;
			// 
			// groupBox8
			// 
			this->groupBox8->Controls->Add(this->radioButtonSelectExp);
			this->groupBox8->Controls->Add(this->radioButtonSelectSim);
			this->groupBox8->Location = System::Drawing::Point(672, 249);
			this->groupBox8->Name = L"groupBox8";
			this->groupBox8->Size = System::Drawing::Size(110, 63);
			this->groupBox8->TabIndex = 55;
			this->groupBox8->TabStop = false;
			this->groupBox8->Text = L"Select Work";
			// 
			// groupBox9
			// 
			this->groupBox9->Controls->Add(this->radioButtonSelectmodCont);
			this->groupBox9->Controls->Add(this->radioButtonSelectCont);
			this->groupBox9->Controls->Add(this->radioButtonSelectInd);
			this->groupBox9->Location = System::Drawing::Point(518, 247);
			this->groupBox9->Name = L"groupBox9";
			this->groupBox9->Size = System::Drawing::Size(148, 89);
			this->groupBox9->TabIndex = 56;
			this->groupBox9->TabStop = false;
			this->groupBox9->Text = L"Select Control";
			// 
			// buttonGraph
			// 
			this->buttonGraph->Location = System::Drawing::Point(518, 342);
			this->buttonGraph->Name = L"buttonGraph";
			this->buttonGraph->Size = System::Drawing::Size(126, 36);
			this->buttonGraph->TabIndex = 57;
			this->buttonGraph->Text = L"Draw Graph";
			this->buttonGraph->UseVisualStyleBackColor = true;
			this->buttonGraph->Click += gcnew System::EventHandler(this, &Form1::buttonGraph_Click);
			// 
			// buttonDump
			// 
			this->buttonDump->Location = System::Drawing::Point(653, 342);
			this->buttonDump->Name = L"buttonDump";
			this->buttonDump->Size = System::Drawing::Size(130, 36);
			this->buttonDump->TabIndex = 58;
			this->buttonDump->Text = L"Data Dump";
			this->buttonDump->UseVisualStyleBackColor = true;
			this->buttonDump->Click += gcnew System::EventHandler(this, &Form1::buttonDump_Click);
			// 
			// textBoxProfile
			// 
			this->textBoxProfile->AcceptsReturn = true;
			this->textBoxProfile->Location = System::Drawing::Point(518, 388);
			this->textBoxProfile->Multiline = true;
			this->textBoxProfile->Name = L"textBoxProfile";
			this->textBoxProfile->ReadOnly = true;
			this->textBoxProfile->ScrollBars = System::Windows::Forms::ScrollBars::Both;
			this->textBoxProfile->Size = System::Drawing::Size(262, 146);
			this->textBoxProfile->TabIndex = 59;
			this->textBoxProfile->Text = L"Profile of program...\r\n";
			this->textBoxProfile->WordWrap = false;
			// 
			// BottunStop
			// 
			this->BottunStop->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 14.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->BottunStop->Location = System::Drawing::Point(634, 70);
			this->BottunStop->Name = L"BottunStop";
			this->BottunStop->Size = System::Drawing::Size(148, 51);
			this->BottunStop->TabIndex = 60;
			this->BottunStop->Text = L"Stop";
			this->BottunStop->UseVisualStyleBackColor = true;
			this->BottunStop->Click += gcnew System::EventHandler(this, &Form1::ControlStopBottun_Click);
			// 
			// radioTime
			// 
			this->radioTime->AutoSize = true;
			this->radioTime->Checked = true;
			this->radioTime->Location = System::Drawing::Point(10, 17);
			this->radioTime->Name = L"radioTime";
			this->radioTime->Size = System::Drawing::Size(48, 17);
			this->radioTime->TabIndex = 61;
			this->radioTime->TabStop = true;
			this->radioTime->Text = L"Time";
			this->radioTime->UseVisualStyleBackColor = true;
			// 
			// radioAngle
			// 
			this->radioAngle->AutoSize = true;
			this->radioAngle->Location = System::Drawing::Point(138, 17);
			this->radioAngle->Name = L"radioAngle";
			this->radioAngle->Size = System::Drawing::Size(117, 17);
			this->radioAngle->TabIndex = 62;
			this->radioAngle->TabStop = true;
			this->radioAngle->Text = L"Angle(unpopulated)";
			this->radioAngle->UseVisualStyleBackColor = true;
			this->radioAngle->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButtonAngle_CheckedChanged);
			// 
			// groupBox10
			// 
			this->groupBox10->Controls->Add(this->label23);
			this->groupBox10->Controls->Add(this->label22);
			this->groupBox10->Controls->Add(this->textBoxRad);
			this->groupBox10->Controls->Add(this->textBoxMaxCount);
			this->groupBox10->Controls->Add(this->radioTime);
			this->groupBox10->Controls->Add(this->radioAngle);
			this->groupBox10->Location = System::Drawing::Point(518, 126);
			this->groupBox10->Name = L"groupBox10";
			this->groupBox10->Size = System::Drawing::Size(264, 64);
			this->groupBox10->TabIndex = 63;
			this->groupBox10->TabStop = false;
			this->groupBox10->Text = L"Select Finish Condition";
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->Location = System::Drawing::Point(219, 42);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(39, 12);
			this->label23->TabIndex = 66;
			this->label23->Text = L"degree";
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Location = System::Drawing::Point(91, 42);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(23, 12);
			this->label22->TabIndex = 65;
			this->label22->Text = L"sec";
			// 
			// textBoxRad
			// 
			this->textBoxRad->Enabled = false;
			this->textBoxRad->Location = System::Drawing::Point(143, 39);
			this->textBoxRad->Name = L"textBoxRad";
			this->textBoxRad->Size = System::Drawing::Size(70, 19);
			this->textBoxRad->TabIndex = 63;
			// 
			// textBoxMaxCount
			// 
			this->textBoxMaxCount->Location = System::Drawing::Point(16, 39);
			this->textBoxMaxCount->Name = L"textBoxMaxCount";
			this->textBoxMaxCount->Size = System::Drawing::Size(70, 19);
			this->textBoxMaxCount->TabIndex = 64;
			this->textBoxMaxCount->Leave += gcnew System::EventHandler(this, &Form1::textBoxMaxCount_TextChanged);
			// 
			// AIOText
			// 
			this->AIOText->Location = System::Drawing::Point(580, 18);
			this->AIOText->Name = L"AIOText";
			this->AIOText->Size = System::Drawing::Size(48, 19);
			this->AIOText->TabIndex = 64;
			// 
			// CNTText
			// 
			this->CNTText->Location = System::Drawing::Point(580, 45);
			this->CNTText->Name = L"CNTText";
			this->CNTText->Size = System::Drawing::Size(48, 19);
			this->CNTText->TabIndex = 65;
			// 
			// AIOName
			// 
			this->AIOName->AutoSize = true;
			this->AIOName->Location = System::Drawing::Point(521, 21);
			this->AIOName->Name = L"AIOName";
			this->AIOName->Size = System::Drawing::Size(53, 12);
			this->AIOName->TabIndex = 66;
			this->AIOName->Text = L"DA board";
			// 
			// CNTName
			// 
			this->CNTName->AutoSize = true;
			this->CNTName->Location = System::Drawing::Point(497, 48);
			this->CNTName->Name = L"CNTName";
			this->CNTName->Size = System::Drawing::Size(77, 12);
			this->CNTName->TabIndex = 67;
			this->CNTName->Text = L"Counter board";
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->Location = System::Drawing::Point(339, 21);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(39, 12);
			this->label26->TabIndex = 36;
			this->label26->Text = L"Pole B";
			// 
			// textBoxPoleB
			// 
			this->textBoxPoleB->Enabled = false;
			this->textBoxPoleB->Location = System::Drawing::Point(386, 18);
			this->textBoxPoleB->Name = L"textBoxPoleB";
			this->textBoxPoleB->Size = System::Drawing::Size(70, 19);
			this->textBoxPoleB->TabIndex = 29;
			// 
			// textBoxPoleT
			// 
			this->textBoxPoleT->Enabled = false;
			this->textBoxPoleT->Location = System::Drawing::Point(71, 18);
			this->textBoxPoleT->Name = L"textBoxPoleT";
			this->textBoxPoleT->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleT->TabIndex = 1;
			// 
			// textBoxPoleN
			// 
			this->textBoxPoleN->Enabled = false;
			this->textBoxPoleN->Location = System::Drawing::Point(220, 18);
			this->textBoxPoleN->Name = L"textBoxPoleN";
			this->textBoxPoleN->Size = System::Drawing::Size(70, 19);
			this->textBoxPoleN->TabIndex = 4;
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->Location = System::Drawing::Point(29, 21);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(38, 12);
			this->label24->TabIndex = 15;
			this->label24->Text = L"Pole T";
			// 
			// groupBox11
			// 
			this->groupBox11->Controls->Add(this->label29);
			this->groupBox11->Controls->Add(this->textBoxPoleJ);
			this->groupBox11->Controls->Add(this->textBoxPoleI);
			this->groupBox11->Controls->Add(this->label30);
			this->groupBox11->Controls->Add(this->textBoxPoleK);
			this->groupBox11->Controls->Add(this->label31);
			this->groupBox11->Controls->Add(this->label24);
			this->groupBox11->Controls->Add(this->textBoxPoleN);
			this->groupBox11->Controls->Add(this->textBoxPoleT);
			this->groupBox11->Controls->Add(this->label25);
			this->groupBox11->Controls->Add(this->textBoxPoleB);
			this->groupBox11->Controls->Add(this->label26);
			this->groupBox11->Location = System::Drawing::Point(5, 379);
			this->groupBox11->Name = L"groupBox11";
			this->groupBox11->Size = System::Drawing::Size(470, 85);
			this->groupBox11->TabIndex = 52;
			this->groupBox11->TabStop = false;
			this->groupBox11->Text = L"Poles of Contouring control";
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->Location = System::Drawing::Point(29, 54);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(34, 12);
			this->label29->TabIndex = 39;
			this->label29->Text = L"Pole I";
			// 
			// textBoxPoleJ
			// 
			this->textBoxPoleJ->Enabled = false;
			this->textBoxPoleJ->Location = System::Drawing::Point(220, 51);
			this->textBoxPoleJ->Name = L"textBoxPoleJ";
			this->textBoxPoleJ->Size = System::Drawing::Size(70, 19);
			this->textBoxPoleJ->TabIndex = 38;
			// 
			// textBoxPoleI
			// 
			this->textBoxPoleI->Enabled = false;
			this->textBoxPoleI->Location = System::Drawing::Point(71, 51);
			this->textBoxPoleI->Name = L"textBoxPoleI";
			this->textBoxPoleI->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleI->TabIndex = 37;
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->Location = System::Drawing::Point(179, 54);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(38, 12);
			this->label30->TabIndex = 40;
			this->label30->Text = L"Pole J";
			// 
			// textBoxPoleK
			// 
			this->textBoxPoleK->Enabled = false;
			this->textBoxPoleK->Location = System::Drawing::Point(386, 51);
			this->textBoxPoleK->Name = L"textBoxPoleK";
			this->textBoxPoleK->Size = System::Drawing::Size(70, 19);
			this->textBoxPoleK->TabIndex = 41;
			// 
			// label31
			// 
			this->label31->AutoSize = true;
			this->label31->Location = System::Drawing::Point(339, 54);
			this->label31->Name = L"label31";
			this->label31->Size = System::Drawing::Size(38, 12);
			this->label31->TabIndex = 42;
			this->label31->Text = L"Pole K";
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->Location = System::Drawing::Point(179, 21);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(39, 12);
			this->label25->TabIndex = 21;
			this->label25->Text = L"Pole N";
			// 
			// groupBox12
			// 
			this->groupBox12->Controls->Add(this->checkBoxSync);
			this->groupBox12->Controls->Add(this->label27);
			this->groupBox12->Controls->Add(this->textBoxPoleSyncA);
			this->groupBox12->Controls->Add(this->textBoxPoleSyncY);
			this->groupBox12->Controls->Add(this->label28);
			this->groupBox12->Location = System::Drawing::Point(5, 470);
			this->groupBox12->Name = L"groupBox12";
			this->groupBox12->Size = System::Drawing::Size(470, 47);
			this->groupBox12->TabIndex = 53;
			this->groupBox12->TabStop = false;
			this->groupBox12->Text = L"Poles of syncronous control";
			// 
			// checkBoxSync
			// 
			this->checkBoxSync->AutoSize = true;
			this->checkBoxSync->Checked = true;
			this->checkBoxSync->CheckState = System::Windows::Forms::CheckState::Checked;
			this->checkBoxSync->Location = System::Drawing::Point(31, 19);
			this->checkBoxSync->Name = L"checkBoxSync";
			this->checkBoxSync->Size = System::Drawing::Size(89, 17);
			this->checkBoxSync->TabIndex = 22;
			this->checkBoxSync->Text = L"Enable Sync.";
			this->checkBoxSync->UseVisualStyleBackColor = true;
			this->checkBoxSync->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBoxSync_CheckedChanged);
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->Location = System::Drawing::Point(148, 20);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(67, 12);
			this->label27->TabIndex = 15;
			this->label27->Text = L"Sync Pole Y";
			// 
			// textBoxPoleSyncA
			// 
			this->textBoxPoleSyncA->Location = System::Drawing::Point(385, 17);
			this->textBoxPoleSyncA->Name = L"textBoxPoleSyncA";
			this->textBoxPoleSyncA->Size = System::Drawing::Size(70, 19);
			this->textBoxPoleSyncA->TabIndex = 4;
			// 
			// textBoxPoleSyncY
			// 
			this->textBoxPoleSyncY->Location = System::Drawing::Point(220, 18);
			this->textBoxPoleSyncY->Name = L"textBoxPoleSyncY";
			this->textBoxPoleSyncY->Size = System::Drawing::Size(69, 19);
			this->textBoxPoleSyncY->TabIndex = 1;
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->Location = System::Drawing::Point(311, 20);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(68, 12);
			this->label28->TabIndex = 21;
			this->label28->Text = L"Sync Pole A";
			// 
			// progressBar1
			// 
			this->progressBar1->Location = System::Drawing::Point(5, 524);
			this->progressBar1->Name = L"progressBar1";
			this->progressBar1->Size = System::Drawing::Size(470, 10);
			this->progressBar1->Style = System::Windows::Forms::ProgressBarStyle::Continuous;
			this->progressBar1->TabIndex = 68;
			// 
			// groupBox13
			// 
			this->groupBox13->Controls->Add(this->radioButtonSelect3D);
			this->groupBox13->Controls->Add(this->radioButtonSelect2D);
			this->groupBox13->Location = System::Drawing::Point(518, 196);
			this->groupBox13->Name = L"groupBox13";
			this->groupBox13->Size = System::Drawing::Size(265, 45);
			this->groupBox13->TabIndex = 69;
			this->groupBox13->TabStop = false;
			this->groupBox13->Text = L"Select DOF";
			// 
			// radioButtonSelect3D
			// 
			this->radioButtonSelect3D->AutoSize = true;
			this->radioButtonSelect3D->Location = System::Drawing::Point(152, 17);
			this->radioButtonSelect3D->Name = L"radioButtonSelect3D";
			this->radioButtonSelect3D->Size = System::Drawing::Size(70, 17);
			this->radioButtonSelect3D->TabIndex = 1;
			this->radioButtonSelect3D->Text = L"3D-7DOF";
			this->radioButtonSelect3D->UseVisualStyleBackColor = true;
			this->radioButtonSelect3D->CheckedChanged += gcnew System::EventHandler(this, &Form1::radioButtonSelect3D_CheckedChanged);
			// 
			// radioButtonSelect2D
			// 
			this->radioButtonSelect2D->AutoSize = true;
			this->radioButtonSelect2D->Checked = true;
			this->radioButtonSelect2D->Location = System::Drawing::Point(29, 17);
			this->radioButtonSelect2D->Name = L"radioButtonSelect2D";
			this->radioButtonSelect2D->Size = System::Drawing::Size(70, 17);
			this->radioButtonSelect2D->TabIndex = 0;
			this->radioButtonSelect2D->TabStop = true;
			this->radioButtonSelect2D->Text = L"2D-3DOF";
			this->radioButtonSelect2D->UseVisualStyleBackColor = true;
			// 
			// ButtonIdentification
			// 
			this->ButtonIdentification->Location = System::Drawing::Point(672, 318);
			this->ButtonIdentification->Name = L"ButtonIdentification";
			this->ButtonIdentification->Size = System::Drawing::Size(111, 23);
			this->ButtonIdentification->TabIndex = 70;
			this->ButtonIdentification->Text = L"Identification";
			this->ButtonIdentification->UseVisualStyleBackColor = true;
			this->ButtonIdentification->Click += gcnew System::EventHandler(this, &Form1::ButtonIdentification_Click);
			// 
			// label32Test
			// 
			this->label32Test->AutoSize = true;
			this->label32Test->Location = System::Drawing::Point(154, 547);
			this->label32Test->Name = L"label32Test";
			this->label32Test->Size = System::Drawing::Size(41, 12);
			this->label32Test->TabIndex = 71;
			this->label32Test->Text = L"label32";
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(792, 577);
			this->Controls->Add(this->label32Test);
			this->Controls->Add(this->ButtonIdentification);
			this->Controls->Add(this->groupBox13);
			this->Controls->Add(this->progressBar1);
			this->Controls->Add(this->groupBox12);
			this->Controls->Add(this->groupBox11);
			this->Controls->Add(this->CNTName);
			this->Controls->Add(this->AIOName);
			this->Controls->Add(this->CNTText);
			this->Controls->Add(this->AIOText);
			this->Controls->Add(this->BottunStop);
			this->Controls->Add(this->textBoxProfile);
			this->Controls->Add(this->buttonDump);
			this->Controls->Add(this->buttonGraph);
			this->Controls->Add(this->groupBox9);
			this->Controls->Add(this->groupBox8);
			this->Controls->Add(this->groupBox7);
			this->Controls->Add(this->groupBox6);
			this->Controls->Add(this->groupBox5);
			this->Controls->Add(this->groupBox4);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->checkBoxResolution);
			this->Controls->Add(this->checkBoxNoise);
			this->Controls->Add(this->ButtonStart);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->groupBox10);
			this->Font = (gcnew System::Drawing::Font(L"MS UI Gothic", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(128)));
			this->Name = L"Form1";
			this->Text = L"Five-axis machine tool sim.";
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &Form1::Form1_FormClosed);
			this->Load += gcnew System::EventHandler(this, &Form1::Form1_Load);
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			this->groupBox5->ResumeLayout(false);
			this->groupBox5->PerformLayout();
			this->groupBox6->ResumeLayout(false);
			this->groupBox6->PerformLayout();
			this->groupBox7->ResumeLayout(false);
			this->groupBox7->PerformLayout();
			this->groupBox8->ResumeLayout(false);
			this->groupBox8->PerformLayout();
			this->groupBox9->ResumeLayout(false);
			this->groupBox9->PerformLayout();
			this->groupBox10->ResumeLayout(false);
			this->groupBox10->PerformLayout();
			this->groupBox11->ResumeLayout(false);
			this->groupBox11->PerformLayout();
			this->groupBox12->ResumeLayout(false);
			this->groupBox12->PerformLayout();
			this->groupBox13->ResumeLayout(false);
			this->groupBox13->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
public:
			 bool						enable_sim;
			 bool						enable_noise;
			 bool						enable_resolution;
			 bool						enable_sync;
			 unsigned int				control_type;
			 bool						is_clicked_start;
			 bool						is_stop;
			 bool						is_be_controling;
			 bool						is_finish_condition_time;
			 bool						select_3d;

			 array<double,2>			^ graph1,^ graph2,^ graph3,^ graph4,^graph5,^graph6,^graph7,^graph8,^graph9,^graph10,^graph11,^graph12,^graph13;
			 array<double,2>			^ datalog;
			 GCHandle					hGC_Timer;
			 CCntCLI::PTIMERCALLBACK^	dele_Timer;
			 IntPtr						pTimer;
			 int						hMainWnd;
			 short						CntId,AioId;
			 long						AioRet,AioRet2,CntRet,CntRet2;		//関数の戻り値

			 double						time;
			 unsigned int				step,max_step;
			 double						max_angle;
			 long						lap;
			 short						TimerId;
			 

private: System::Void ButtonStart_Click(System::Object^  sender, System::EventArgs^  e) {
			 using namespace define;
			 using namespace CAioCLI;
			 using namespace CCntCLI;

			 StringBuilder	^ErrorString	= gcnew StringBuilder(256);	//エラーコード文字列
			 SetNotifyDelegate^ deleSetNotifyText	= gcnew SetNotifyDelegate(this, &Form1::SetNotifyText);
			 SetNotifyClearDelegate^ deleSetNotifyClear	= gcnew SetNotifyClearDelegate(this, &Form1::SetNotifyClear);
			 String^	AioDeviceName,^ CntDeviceName;
			 AioDeviceName = AIOText->Text;
			 CntDeviceName = CNTText->Text;
			 pin_ptr<short>pAioId = &static_cast<short>(AioId);
			 pin_ptr<short>pCntId = &static_cast<short>(CntId);
			 short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};


			 if(is_be_controling)
				 return;
			 else
				 is_be_controling = true;

			 max_step = static_cast<unsigned int>(double::Parse(textBoxMaxCount->Text) / SAMPLING_TIME);
			 max_angle = double::Parse(textBoxRad->Text);
			 if(radioAngle->Checked)
				 is_finish_condition_time = false;
			 else
				 is_finish_condition_time = true;

			 if(radioButtonSelectInd->Checked)
				 control_type = control2d::INDEPENDENT;
			 else if(radioButtonSelectCont->Checked)
				 control_type = control2d::CONTOURING;
			 else
				 control_type = control2d::MOD_CONTOURING;

			 if(radioButtonSelectSim->Checked)
				 enable_sim = true;
			 else
				 enable_sim = false;

			 if(checkBoxSync->Checked)
				 enable_sync = true;
			 else
				 enable_sync = false;

			 is_stop = false;	// 制御終了ハンドラ実行中の割り込み発生を防ぐフラグ（たぶんいらない・・・）

			 //textBoxProfile->Clear();
			 this->Invoke(deleSetNotifyClear);
			 if(enable_sim)
				 this->Invoke(deleSetNotifyText, String::Format("Start Simulation.\r\n"));
			 else
				 this->Invoke(deleSetNotifyText, String::Format("Start Experiment.\r\n"));
			 

			 // データログ変数をクリア
			 for(unsigned int i=0;i<NUM_DUMPDATA;i++){
				 for(unsigned int j=0;j<MAX_COUNT;j++){
					 datalog[i,j] = 0;
				 }
			 }
			 label32Test->Text = String::Format("AioId= {0:d}",AioId);
			 if(!enable_sim)
			 {
				 // 初期化処理
				 AioRet = AioInit(AioDeviceName, pAioId);
				 if (AioRet != 0){
					 AioRet2 = AioGetErrorString(AioRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("AioInit= {0:d}: {1}",AioRet,ErrorString));
					 return;
				 }
				 CntRet = CntInit(CntDeviceName, pCntId);
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntInit= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }

				 // ＤＡボードのプロセスリセット
				 AioRet = AioResetProcess ( AioId );
				 if (AioRet != 0){
					 AioRet2 = AioGetErrorString(AioRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("AioResetProcess= {0:d}: {1}",AioRet,ErrorString));
					 return;
				 }

				 // デバイスのリセット
				 AioRet = AioResetDevice(AioId);
				 if (AioRet != 0){
					 AioRet2 = AioGetErrorString(AioRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("AioResetDevice= {0:d}: {1}",AioRet,ErrorString));
					 return;
				 }
				 CntRet = CntResetDevice(CntId);
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntResetDevice= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }
			 
				 // 出力レンジの設定
				 AioRet = AioSetAoRangeAll(AioId, PM10);
				 if (AioRet != 0){
					 AioRet2 = AioGetErrorString(AioRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("AioSetAoRangeAll= {0:d}: {1}",AioRet,ErrorString));
					 return;
				 }
				 // カウンタ入力シグナルが差動であると指定
				 for(unsigned int i=0;i<NUM_COUNTER;i++)
				 {
					 CntRet = CntSelectChannelSignal(CntId,i,CNT_SIGTYPE_LINERECEIVER);
					 if (CntRet != 0){
						 CntRet2 = CntGetErrorString(CntRet, ErrorString);
						 this->Invoke(deleSetNotifyText, String::Format("CntSelectChannelSignal= {0:d}: {1}",CntRet,ErrorString));
						 return;
					 }
					 CntRet = CntSetOperationMode(CntId,i,CNT_MODE_2PHASE,CNT_MUL_X4,CNT_CLR_ASYNC);
					 if (CntRet != 0){
						 CntRet2 = CntGetErrorString(CntRet, ErrorString);
						 this->Invoke(deleSetNotifyText, String::Format("CntSetOperationMode= {0:d}: {1}",CntRet,ErrorString));
						 return;
					 }
					 CntRet = CntSetZMode(CntId,i,CNT_ZPHASE_NOT_USE);
					 if (CntRet != 0){
						 CntRet2 = CntGetErrorString(CntRet, ErrorString);
						 this->Invoke(deleSetNotifyText, String::Format("CntSetZMode= {0:d}: {1}",CntRet,ErrorString));
						 return;
					 }
					 CntRet = CntSetCountDirection (CntId,i,CNT_DIR_UP);
					 if (CntRet != 0){
						 CntRet2 = CntGetErrorString(CntRet, ErrorString);
						 this->Invoke(deleSetNotifyText, String::Format("CntSetCountDirection= {0:d}: {1}",CntRet,ErrorString));
						 return;
					 }
				 }
				 this->Invoke(deleSetNotifyText, "初期化処理 : 正常終了\r\n");
			 }

			 // get control parameter
			 array<double>^ conparam = gcnew array<double>(NUM_ACTUATOR*3+6+2);
			 conparam[0] = double::Parse(textBoxMassX->Text);
			 conparam[1] = double::Parse(textBoxMassY1->Text);
			 conparam[2] = double::Parse(textBoxMassY2->Text);
			 conparam[3] = double::Parse(textBoxMassZ->Text);
			 conparam[4] = double::Parse(textBoxInertiaC->Text);
			 conparam[5] = double::Parse(textBoxInertiaA1->Text);
			 conparam[6] = double::Parse(textBoxInertiaA2->Text);
			 conparam[7] = double::Parse(textBoxFrictionX->Text);
			 conparam[8] = double::Parse(textBoxFrictionY1->Text);
			 conparam[9] = double::Parse(textBoxFrictionY2->Text);
			 conparam[10] = double::Parse(textBoxFrictionZ->Text);
			 conparam[11] = double::Parse(textBoxFrictionC->Text);
			 conparam[12] = double::Parse(textBoxFrictionA1->Text);
			 conparam[13] = double::Parse(textBoxFrictionA2->Text);
			 conparam[14] = double::Parse(textBoxPoleX->Text);
			 conparam[15] = double::Parse(textBoxPoleY1->Text);
			 conparam[16] = double::Parse(textBoxPoleY2->Text);
			 conparam[17] = double::Parse(textBoxPoleZ->Text);
			 conparam[18] = double::Parse(textBoxPoleC->Text);
			 conparam[19] = double::Parse(textBoxPoleA1->Text);
			 conparam[20] = double::Parse(textBoxPoleA2->Text);
			 conparam[21] = double::Parse(textBoxPoleT->Text);
			 conparam[22] = double::Parse(textBoxPoleN->Text);
			 conparam[23] = double::Parse(textBoxPoleB->Text);
			 conparam[24] = double::Parse(textBoxPoleI->Text);
			 conparam[25] = double::Parse(textBoxPoleJ->Text);
			 conparam[26] = double::Parse(textBoxPoleK->Text);
			 conparam[27] = double::Parse(textBoxPoleSyncY->Text);
			 conparam[28] = double::Parse(textBoxPoleSyncA->Text);
			 control2d::SetConParam(conparam);
			 control3d::SetConParam(conparam);

			 step = 0;	// ステップの初期化

			 // Starting counter board
			 if(!enable_sim)
			 {
				 CntRet = CntStartCount ( CntId , ChNo , NUM_COUNTER );
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntStartCount= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }
				 CntRet = CntZeroClearCount ( CntId , ChNo , NUM_COUNTER );
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntZeroClearCount= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }
				 unsigned long PresetData[NUM_COUNTER];
				 double pos_pre[NUM_COUNTER] = {INIT_POS_X,INIT_POS_Y1,INIT_POS_Y2,INIT_POS_Z,INIT_ANGLE_C,INIT_ANGLE_A1,INIT_ANGLE_A2};
				 double resonate[NUM_COUNTER] = {RESONATE_LINER_ENC_X,RESONATE_LINER_ENC_Y1,RESONATE_LINER_ENC_Y2,RESONATE_LINER_ENC_Z,RESONATE_ROTATION_ENC_C,RESONATE_ROTATION_ENC_A1,RESONATE_ROTATION_ENC_A2};
				 long dir[NUM_COUNTER]={-1,1,-1,-1,1,1,-1};
				 for(unsigned int i=0;i<NUM_COUNTER;i++){
					 pos_pre[i] = pos_pre[i]*dir[i]/resonate[i];
					 PresetData[i] = static_cast<unsigned long>(pos_pre[i]);
				 }
				 CntRet = CntPreset( CntId , ChNo , NUM_COUNTER , PresetData );
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntPreset= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }

				 // reset power tester
				 CntRet = CntOutputDOBit ( CntId , 1 , 0 , 1 );
				 System::Threading::Thread::Sleep(200);
				 CntRet = CntOutputDOBit ( CntId , 1 , 0 , 0 );

				 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
				 CntRet	= CntTimerCallbackProc(CntId, pTimer, nullptr);
				 CntRet = CntNotifyTimer ( CntId , static_cast<unsigned long>(SAMPLING_TIME*1000),0);
				 
				 // start power tester
				 CntRet = CntOutputDOBit ( CntId , 0 , 0 , 1 );
			 } else {
				 boost::timer t;

				 if(select_3d)
					 control3d::ControlSim3D(control_type,max_step,enable_noise,enable_resolution,enable_sync,datalog);
				 else
					 control2d::ControlSim2D(control_type,max_step,enable_noise,enable_resolution,enable_sync,datalog);

				 double e_time = t.elapsed();
				 double ave_time = e_time / max_step * 1000;
				 String^ str;
				 str = "Elapsed time : "+e_time.ToString("f2")+"sec\r\n";
				 str += "Elapsed time each loop(ave) : "+ave_time.ToString("f4")+"msec\r\n";
				 this->Invoke(deleSetNotifyText, str);

				 ControlStop();
			 }
		 }
private: System::Void radioButtonSelectSim_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 if(radioButtonSelectSim->Checked){
				 checkBoxNoise->Enabled = true;
				 checkBoxResolution->Enabled = true;
			 }else{
				 checkBoxNoise->Enabled = false;
				 checkBoxResolution->Enabled = false;
			 }
		 }
private: System::Void buttonGraph_Click(System::Object^  sender, System::EventArgs^  e) {
			 using namespace define;

			 if(is_be_controling)
				 return;
			 if(max_step == 0)
				 return;

			 Form2^ Form2Obj = gcnew Form2(); // 子ﾌｫｰﾑ生成

			 // 子フォームへグラフデータを渡す
			 Form2Obj->num_data = max_step;
			 Form2Obj->graph1 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph2 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph3 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph4 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph5 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph6 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph7 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph8 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph9 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph10 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph11 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph12 = gcnew array<double,2>(2,MAX_COUNT);
			 Form2Obj->graph13 = gcnew array<double,2>(2,MAX_COUNT);
			 for(unsigned int j=0;j<2;j++)
			 {
				 for(unsigned int i=0;i<MAX_COUNT;i++)
				 {
					Form2Obj->graph1[j,i] = graph1[j,i];
					Form2Obj->graph2[j,i] = graph2[j,i];
					Form2Obj->graph3[j,i] = graph3[j,i];
					Form2Obj->graph4[j,i] = graph4[j,i];
					Form2Obj->graph5[j,i] = graph5[j,i];
					Form2Obj->graph6[j,i] = graph6[j,i];
					Form2Obj->graph7[j,i] = graph7[j,i];
					Form2Obj->graph8[j,i] = graph8[j,i];
					Form2Obj->graph9[j,i] = graph9[j,i];
					Form2Obj->graph10[j,i] = graph10[j,i];
					Form2Obj->graph11[j,i] = graph11[j,i];
					Form2Obj->graph12[j,i] = graph12[j,i];
					Form2Obj->graph13[j,i] = graph13[j,i];
				 }
			 }

			 Form2Obj->ShowDialog(); // 子ﾌｫｰﾑを表示（モーダル）
		 }
private: System::Void checkBoxNoise_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 enable_noise = checkBoxNoise->Checked;
		 }
private: System::Void checkBoxResolution_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 enable_resolution = checkBoxResolution->Checked;
		 }
private: System::Void buttonDump_Click(System::Object^  sender, System::EventArgs^  e) {
			 if(is_be_controling)
				 return;
				
			 OutputFile();
			 //OutputExcelFile();
		 }
private: void OutputFile(void){
			 using namespace csv;
			 using namespace define;

			 SetNotifyDelegate^ deleSetNotifyText	= gcnew SetNotifyDelegate(this, &Form1::SetNotifyText);
			 time_t t;
			 struct tm t_st;
			 FILE *fp;
			 std::string filename;
			 std::string dotcsv(".csv");
			 char f2c[256];
			 errno_t err;
			 
			 // コメント
			 this->Invoke(deleSetNotifyText, "Now writing...\r\n");

			 _time64(&t);					// 現在時刻の取得
			 err = localtime_s(&t_st,&t);	// 現在時刻を構造体に変換
			 t_st.tm_year += 1900;
			 t_st.tm_mon += 1;

			 // creating file name
			 filename = boost::lexical_cast<std::string>(t_st.tm_year);
			 filename += '_';
			 if(t_st.tm_mon < 10)
				 filename += '0';
			 filename += boost::lexical_cast<std::string>(t_st.tm_mon);
			 if(t_st.tm_mday < 10)
				 filename += '0';
			 filename += boost::lexical_cast<std::string>(t_st.tm_mday);
			 filename += '_';
			 if(t_st.tm_hour < 10)
				 filename += '0';
			 filename += boost::lexical_cast<std::string>(t_st.tm_hour);
			 if(t_st.tm_min < 10)
				 filename += '0';
			 filename += boost::lexical_cast<std::string>(t_st.tm_min);
			 if(t_st.tm_sec < 10)
				 filename += '0';
			 filename += boost::lexical_cast<std::string>(t_st.tm_sec);
			 filename += dotcsv;

			 // file open
			 if ((err = fopen_s(&fp,filename.c_str(), "w")) !=0 ) {
				 this->Invoke(deleSetNotifyText, "File open error!");
				 return;
			 }

			 // writing data labels
			 for(unsigned int i=0;i<NUM_DUMPDATA;i++)
				 write_val(fp,true,define::label[i]);
			 write_sep(fp,false);

			 // writing datas
			 double progress;
			 SetProgressDelegate^ deleSetProgress = gcnew SetProgressDelegate(this, &Form1::SetProgress);
			 for(unsigned int j=0;j<max_step;j++)
			 {
				 for(unsigned int i=0;i<NUM_DUMPDATA;i++)
				 {
					 sprintf_s(f2c,"%.15f",datalog[i,j]);
					 write_val(fp,true,f2c);
				 }
				 write_sep(fp,false);
				 progress = static_cast<double>(j)/max_step*100;
				 this->Invoke(deleSetProgress, static_cast<int>(progress));
			 }

			 double *ec,*en_ec,*en_ecmod,*esy,*esa;
			 ec = new double[max_step];
			 en_ec = new double[max_step];
			 en_ecmod = new double[max_step];
			 esy = new double[max_step];
			 esa = new double[max_step];
			 for(unsigned int i=0;i<max_step;i++){
				 ec[i] = datalog[1,i];
				 en_ec[i] = datalog[2,i];
				 en_ecmod[i] = datalog[3,i];
				 esy[i] = datalog[8,i];
				 esa[i] = datalog[9,i];
			 }

			 unsigned int samp1 = static_cast<unsigned int>(2*PI / OMG / SAMPLING_TIME);

			 write_val(fp,true,"average contouring error");
			 write_sep(fp,false);
			 sprintf_s(f2c,"%.12f",AverageAbs(ec,max_step));
			 write_val(fp,true,f2c);
			 if(max_step > samp1){
				 sprintf_s(f2c,"%.12f",AverageAbs(&ec[samp1],max_step-samp1));
				 write_val(fp,true,f2c);
			 }
			 write_sep(fp,false);

			 write_val(fp,true,"average normal error - contouring error");
			 write_sep(fp,false);
			 sprintf_s(f2c,"%.12f",AverageAbs(en_ec,max_step));
			 write_val(fp,true,f2c);
			 if(max_step > samp1){
				 sprintf_s(f2c,"%.12f",AverageAbs(&en_ec[samp1],max_step-samp1));
				 write_val(fp,true,f2c);
			 }
			 write_sep(fp,false);

			 write_val(fp,true,"average normal error - contouring error(mod)");
			 write_sep(fp,false);
			 sprintf_s(f2c,"%.12f",AverageAbs(en_ecmod,max_step));
			 write_val(fp,true,f2c);
			 if(max_step > samp1){
				 sprintf_s(f2c,"%.12f",AverageAbs(&en_ecmod[samp1],max_step-samp1));
				 write_val(fp,true,f2c);
			 }
			 write_sep(fp,false);

			 write_val(fp,true,"average synchronous error(y)");
			 write_sep(fp,false);
			 sprintf_s(f2c,"%.12f",AverageAbs(esy,max_step));
			 write_val(fp,true,f2c);
			 if(max_step > samp1){
				 sprintf_s(f2c,"%.12f",AverageAbs(&esy[samp1],max_step-samp1));
				 write_val(fp,true,f2c);
			 }
			 write_sep(fp,false);

			 write_val(fp,true,"average synchronous error(a)");
			 write_sep(fp,false);
			 sprintf_s(f2c,"%.12f",AverageAbs(esa,max_step));
			 write_val(fp,true,f2c);
			 if(max_step > samp1){
				 sprintf_s(f2c,"%.12f",AverageAbs(&esa[samp1],max_step-samp1));
				 write_val(fp,true,f2c);
			 }
			 write_sep(fp,false);

			 delete ec,en_ec,en_ecmod,esy,esa;

			 if (G_iWriteErrFlg) {
				 textBoxProfile->Text = "File write error !!";
				 return;
			 }

			 fclose(fp);	// file close
			 
			 this->Invoke(deleSetNotifyText, "Data output!\r\n");
		 }
private: void OutputExcelFile(void){
			 using namespace csv;
			 using namespace define;
			 using namespace Microsoft::Office::Interop::Excel;

			 // file open
			 Microsoft::Office::Interop::Excel::Application^ xlapp = gcnew Microsoft::Office::Interop::Excel::ApplicationClass();  
			 Workbooks^ wbs = xlapp->Workbooks;
			 Workbook^ wb = wbs->Add(Type::Missing);
			 Sheets^ wss = wb->Worksheets;
			 Worksheet^ wsdat  = static_cast<Worksheet^>(wss[1]);
			 Worksheet^ wsconclusion  = static_cast<Worksheet^>(wss[2]);

			 wsdat->Name = "dat";
			 wsconclusion->Name = "conclusion";

			 // delete worksheet[3]
			 static_cast<Worksheet^>(xlapp->ActiveWorkbook->Sheets->Item[3])->Delete();
			 
			 //保存時の確認ダイアログを表示しない
			 xlapp->DisplayAlerts = false;
			 
			 // writing data labels
			 for(unsigned int i=0;i<NUM_DUMPDATA;i++){
				 String^ labelstr = gcnew String(label[i]);
				 wsdat->Cells[1,i+1] = labelstr;
			 }

			 // writing datas
			 double pro;
			 SetProgressDelegate^ deleSetProgress = gcnew SetProgressDelegate(this, &Form1::SetProgress);
			 for(unsigned int j=0;j<max_step;j++){
				 for(unsigned int i=0;i<NUM_DUMPDATA;i++)
					 wsdat->Cells[j+2,i+1] = datalog[i,j];
				 pro=(double)(j)/max_step*100;
				 this->Invoke(deleSetProgress, static_cast<int>(pro));
			 }
			 pro = 0;
			 this->Invoke(deleSetProgress, static_cast<int>(pro));

			 double *ec,*en_ec,*en_ecmod,*esy,*esa;
			 ec = new double[max_step];
			 en_ec = new double[max_step];
			 en_ecmod = new double[max_step];
			 esy = new double[max_step];
			 esa = new double[max_step];
			 for(unsigned int i=0;i<max_step;i++){
				 ec[i] = datalog[1,i];
				 en_ec[i] = datalog[2,i];
				 en_ecmod[i] = datalog[3,i];
				 esy[i] = datalog[8,i];
				 esa[i] = datalog[9,i];
			 }
			 wsconclusion->Cells[1,1] = "average contouring error";
			 wsconclusion->Cells[2,1] = "average normal error - contouring error";
			 wsconclusion->Cells[3,1] = "average normal error - contouring error(mod)";
			 wsconclusion->Cells[4,1] = "average synchronous error(y)";
			 wsconclusion->Cells[5,1] = "average synchronous error(a)";
			 wsconclusion->Cells[1,2] = AverageAbs(ec,max_step);
			 wsconclusion->Cells[2,2] = AverageAbs(en_ec,max_step);
			 wsconclusion->Cells[3,2] = AverageAbs(en_ecmod,max_step);
			 wsconclusion->Cells[4,2] = AverageAbs(esy,max_step);
			 wsconclusion->Cells[5,2] = AverageAbs(esa,max_step);
			 delete ec,en_ec,en_ecmod,esy,esa;

			 xlapp->Visible=true;

			 //wb->SaveAs("aaa");
			 wb->SaveAs(Type::Missing,//"C:\Documents and Settings\nakamura\デスクトップ\five-axis_machine_tool\five-axis_machine_tool\Book1.xlsx",
						//Microsoft.Office.Interop.Excel.XlFileFormat.xlXMLSpreadsheet,
						Type::Missing,
						Type::Missing,
						Type::Missing,
						Type::Missing,
						Type::Missing,
						Microsoft::Office::Interop::Excel::XlSaveAsAccessMode::xlNoChange,
						Type::Missing,
						Type::Missing,
						Type::Missing,
						Type::Missing,
						Type::Missing);

			 // COM オブジェクトの参照カウントを解放する (正しくは COM オブジェクトの参照カウントを解放する を参照)
			 System::Runtime::InteropServices::Marshal::ReleaseComObject(wsdat);
			 System::Runtime::InteropServices::Marshal::ReleaseComObject(wsconclusion);
			 System::Runtime::InteropServices::Marshal::ReleaseComObject(wss);
			 wb->Close(true,Type::Missing,false);
			 System::Runtime::InteropServices::Marshal::ReleaseComObject(wb);
			 System::Runtime::InteropServices::Marshal::ReleaseComObject(wbs);
			 // Excel を終了する
			 xlapp->Quit();
			 System::Runtime::InteropServices::Marshal::ReleaseComObject(xlapp);
			 
			 textBoxProfile->AppendText("Excel file output!\r\n");
		 }
private: System::Void Form1_Load(System::Object^  sender, System::EventArgs^  e) {
			 
			 using namespace define;

			enable_sim = true;
			enable_noise = false;
			enable_resolution = false;
			enable_sync = true;
			control_type = control2d::INDEPENDENT;
			is_clicked_start = false;
			is_be_controling = false;
			select_3d = false;

			max_step = 0;
		
			graph1 = gcnew array<double,2>(2,MAX_COUNT);
			graph2 = gcnew array<double,2>(2,MAX_COUNT);
			graph3 = gcnew array<double,2>(2,MAX_COUNT);
			graph4 = gcnew array<double,2>(2,MAX_COUNT);
			graph5 = gcnew array<double,2>(2,MAX_COUNT);
			graph6 = gcnew array<double,2>(2,MAX_COUNT);
			graph7 = gcnew array<double,2>(2,MAX_COUNT);
			graph8 = gcnew array<double,2>(2,MAX_COUNT);
			graph9 = gcnew array<double,2>(2,MAX_COUNT);
			graph10 = gcnew array<double,2>(2,MAX_COUNT);
			graph11 = gcnew array<double,2>(2,MAX_COUNT);
			graph12 = gcnew array<double,2>(2,MAX_COUNT);
			graph13 = gcnew array<double,2>(2,MAX_COUNT);
			for(unsigned int i=0;i<MAX_COUNT;i++){
				graph1[0,i] = i;
				graph2[0,i] = i;
				graph3[0,i] = i;
				graph4[0,i] = i;
				graph5[0,i] = i;
				graph6[0,i] = i;
				graph7[0,i] = i;
				graph8[0,i] = i;
				graph9[0,i] = i;
				graph10[0,i] = i;
				graph11[0,i] = i;
				graph12[0,i] = i;
				graph13[0,i] = i;
			}

			datalog = gcnew array<double,2>(NUM_DUMPDATA,MAX_COUNT);

			// reading config data
			std::ifstream ifs("config.ini");
			double pole,mass,fric;
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleX->Text = pole.ToString();
			textBoxMassX->Text = mass.ToString();
			textBoxFrictionX->Text = fric.ToString();
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleY1->Text = pole.ToString();
			textBoxMassY1->Text = mass.ToString();
			textBoxFrictionY1->Text = fric.ToString();
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleY2->Text = pole.ToString();
			textBoxMassY2->Text = mass.ToString();
			textBoxFrictionY2->Text = fric.ToString();
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleZ->Text = pole.ToString();
			textBoxMassZ->Text = mass.ToString();
			textBoxFrictionZ->Text = fric.ToString();
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleC->Text = pole.ToString();
			textBoxInertiaC->Text = mass.ToString();
			textBoxFrictionC->Text = fric.ToString();
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleA1->Text = pole.ToString();
			textBoxInertiaA1->Text = mass.ToString();
			textBoxFrictionA1->Text = fric.ToString();
			ifs >> pole; ifs >> mass; ifs >> fric;
			textBoxPoleA2->Text = pole.ToString();
			textBoxInertiaA2->Text = mass.ToString();
			textBoxFrictionA2->Text = fric.ToString();
			ifs >> pole;
			textBoxPoleT->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleN->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleB->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleI->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleJ->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleK->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleSyncY->Text = pole.ToString();
			ifs >> pole;
			textBoxPoleSyncA->Text = pole.ToString();
			double count,rad;
			ifs >> count;
			textBoxMaxCount->Text = count.ToString();
			ifs >> rad;
			textBoxRad->Text = rad.ToString();
			
			std::string devicename_str;
			ifs >> devicename_str;
			String^ aiodevicename = gcnew String(devicename_str.c_str());
			AIOText->Text = aiodevicename;
			ifs >> devicename_str;
			String^ cntdevicename = gcnew String(devicename_str.c_str());
			CNTText->Text = cntdevicename;

			 //-----------------------------------
			 // ウィンドウハンドルのコピー
			 //-----------------------------------
			 hMainWnd	= this->Handle.ToInt32();
			 //-----------------------------------
			 // デリゲート初期化
			 //-----------------------------------
			 dele_Timer	= gcnew CCntCLI::PTIMERCALLBACK(this, &Form1::TimerCallBackProc);

			 //-----------------------------------
			 // ガベージコレクションにより破棄されないようにデリゲートへの参照を追加
			 //-----------------------------------
			 hGC_Timer	= GCHandle::Alloc(dele_Timer);
		 }
private: System::Void Form1_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) {
			// writing config data
			std::ofstream ofs("config.ini");
			double pole,mass,fric;
			pole = double::Parse(textBoxPoleX->Text);
			mass = double::Parse(textBoxMassX->Text);
			fric = double::Parse(textBoxFrictionX->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleY1->Text);
			mass = double::Parse(textBoxMassY1->Text);
			fric = double::Parse(textBoxFrictionY1->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleY2->Text);
			mass = double::Parse(textBoxMassY2->Text);
			fric = double::Parse(textBoxFrictionY2->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleZ->Text);
			mass = double::Parse(textBoxMassZ->Text);
			fric = double::Parse(textBoxFrictionZ->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleC->Text);
			mass = double::Parse(textBoxInertiaC->Text);
			fric = double::Parse(textBoxFrictionC->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleA1->Text);
			mass = double::Parse(textBoxInertiaA1->Text);
			fric = double::Parse(textBoxFrictionA1->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleA2->Text);
			mass = double::Parse(textBoxInertiaA2->Text);
			fric = double::Parse(textBoxFrictionA2->Text);
			ofs <<pole<<std::endl<<mass<<std::endl<<fric<<std::endl;
			pole = double::Parse(textBoxPoleT->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleN->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleB->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleI->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleJ->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleK->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleSyncY->Text);
			ofs << pole << std::endl;
			pole = double::Parse(textBoxPoleSyncA->Text);
			ofs << pole << std::endl;
			ofs << double::Parse(textBoxMaxCount->Text) <<std::endl;
			ofs << double::Parse(textBoxRad->Text) <<std::endl;

			String^ devicename_str;
			char *ch;
			size_t convertedChars = 0,sizeInBytes;
			pin_ptr<const wchar_t> wch;
			errno_t err;

			devicename_str = AIOText->Text;
			// Pin memory so GC can't move it while native function is called
			wch = PtrToStringChars(devicename_str);
			sizeInBytes = ((devicename_str->Length + 1) * 2);
			ch = (char *)malloc(sizeInBytes);
			err = wcstombs_s(&convertedChars, 
							ch, sizeInBytes,
							wch, sizeInBytes);
			ofs << ch << std::endl;
			devicename_str = CNTText->Text;
			// Pin memory so GC can't move it while native function is called
			wch = PtrToStringChars(devicename_str);
			sizeInBytes = ((devicename_str->Length + 1) * 2);
			ch = (char *)malloc(sizeInBytes);
			err = wcstombs_s(&convertedChars, 
							ch, sizeInBytes,
							wch, sizeInBytes);
			ofs << ch << std::endl;

			hGC_Timer.Free();
		 }
void TimerCallBackProc(short m_Id, int wParam, int lParam, void * Param){
	using namespace define;

	time = step * SAMPLING_TIME;

	if(select_3d){
		if(control_type == control3d::INDEPENDENT)
			control3d::IndependentCon(step,AioId,CntId,enable_sync,datalog);
		else if(control_type == control3d::CONTOURING)
			control3d::ContouringCon(step,AioId,CntId,enable_sync,datalog);
		else if(control_type == control3d::MOD_CONTOURING)
			control3d::modContouringCon(step,AioId,CntId,enable_sync,datalog);
	}else{
		if(control_type == control2d::INDEPENDENT)
			control2d::IndependentCon(step,AioId,CntId,enable_sync,datalog);
			//control2d::Identification(step,AioId,CntId,enable_sync,datalog);
		else if(control_type == control2d::CONTOURING)
			control2d::ContouringCon(step,AioId,CntId,enable_sync,datalog);
		else if(control_type == control2d::MOD_CONTOURING)
			control2d::modContouringCon(step,AioId,CntId,enable_sync,datalog);
	}

	// show progress bar
	double pro = static_cast<double>(step*100)/max_step;
	SetProgressDelegate^ deleSetProgress = gcnew SetProgressDelegate(this, &Form1::SetProgress);
	this->Invoke(deleSetProgress, static_cast<int>(pro));

	// 設定時間が経過したら制御停止
	// データログの変数アレイの数はmax_stepによって決まるので、設定時間経過後は終了条件に関わらず制御停止
	if(++step > max_step-1){
		if(is_stop == false){
			is_stop = true;
			ControlStop();
		}
	}
	else if(is_finish_condition_time == false){
		if(control2d::IsReachPoint(time,max_angle,datalog) && is_stop == false){
			is_stop = true;
			ControlStop();
		}
	}

	return;
}
private: void ControlStop(void){
			 using namespace define;
			 using namespace CAioCLI;
			 using namespace CCntCLI;
			 
			 SetNotifyDelegate^ deleSetNotifyText	= gcnew SetNotifyDelegate(this, &Form1::SetNotifyText);
			 StringBuilder	^ErrorString	= gcnew StringBuilder(256);	//エラーコード文字列
			 short ChNo[NUM_COUNTER] = {0,1,2,3,4,5,6};

			 if(!enable_sim){
				 CntRet = CntOutputDOBit ( CntId , 0 , 0 , 0 );
				 CntRet = CntStopNotifyTimer ( CntId );
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntStopNotifyTimer= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }
				 // Stoping counter board and DA board
				 CntRet = CntStopCount ( CntId , ChNo , NUM_COUNTER );
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntStopCount= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }
				 for(unsigned int i=0;i<NUM_ACTUATOR;i++)
				 {
					 AioRet = AioSingleAoEx(AioId,i,0);
					 if (AioRet != 0){
						 AioRet2 = AioGetErrorString(AioRet, ErrorString);
						 this->Invoke(deleSetNotifyText, String::Format("AioSingleAoEx= {0:d}: {1}",AioRet,ErrorString));
						 return;
					 }
				 }
				 // ＤＡボードのクローズ
				 AioRet = AioExit(AioId);
				 if (AioRet != 0){
					 AioRet2 = AioGetErrorString(AioRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("AioExit= {0:d}: {1}",AioRet,ErrorString));
					 return;
				 }
				 // カウンタボードのクローズ
				 CntRet = CntExit(CntId);
				 if (CntRet != 0){
					 CntRet2 = CntGetErrorString(CntRet, ErrorString);
					 this->Invoke(deleSetNotifyText, String::Format("CntExit= {0:d}: {1}",CntRet,ErrorString));
					 return;
				 }
			 }

			 // 最適化問題を解き、輪郭誤差の真値を求める
			 this->Invoke(deleSetNotifyText, "輪郭誤差計算中...\r\n");
			 CalcOptContouringError(datalog,select_3d);

			 // グラフ用テーブルへのデータ記録
			 for(unsigned int i=0;i<MAX_COUNT;i++)
			 {
				 graph1[1,i] = datalog[1,i];	// ec
				 graph2[1,i] = datalog[2,i];	// ec-en
				 graph3[1,i] = datalog[3,i];	// ec-en(mod)
				 graph4[1,i] = datalog[16,i];	// force(x-axis)
				 graph5[1,i] = datalog[24,i];	// force(y1-axis)
				 graph6[1,i] = datalog[32,i];	// force(y2-axis)
				 graph7[1,i] = datalog[40,i];	// force(z-axis)
				 graph8[1,i] = datalog[48,i];	// torque(c-axis)
				 graph9[1,i] = datalog[56,i];	// torque(a1-axis)
				 graph10[1,i] = datalog[64,i];	// torque(a2-axis)
				 graph11[1,i] = datalog[5,i];	// et
				 graph12[1,i] = datalog[6,i];	// en
				 graph13[1,i] = datalog[7,i];	// eb
			 }

			 const int num = 3;
			 double *temp[num];
			 for(unsigned int i=0;i<num;i++)
				 temp[i] = new double[datalog->GetLength(1)];
			 for(unsigned int i=0;i<max_step;i++){
				 temp[0][i] = datalog[1,i];		// ec
				 temp[1][i] = datalog[66,i];	// phi_t
				 temp[2][i] = datalog[67,i];	// phi_td
			 }

			 double ave[num],ave2[num],peakp[num],peakn[num];
			 unsigned int samp1 = static_cast<unsigned int>(2*PI / OMG / SAMPLING_TIME);
			 for(unsigned int i=0;i<num;i++){
				 ave[i] = AverageAbs(temp[i],max_step)*1000000;
				 peakp[i] = PeakPos(temp[i],max_step)*1000000;
				 peakn[i] = PeakNeg(temp[i],max_step)*1000000;
				 if(max_step > samp1)
					 ave2[i] = AverageAbs(&temp[i][samp1],max_step-samp1)*1000000;
				 else
					 ave2[i] = 0;
			 }

			 for(unsigned int i=0;i<3;i++)
				 delete temp[i];

			 String^ str;
			 if(enable_sim)
				 str = "Simulation has finished.\r\n";
			 else
				 str = "Experiment has finished.\r\n";

			 str += "Average Contouring Error :\r\n";
			 str += ave[0].ToString("f4")+"μm\r\n";
			 str += "Average Contouring Error(1-) :\r\n";
			 str += ave2[0].ToString("f4")+"μm\r\n";
			 str += "Peak Contouring Error :\r\n";
			 str += "Max "+peakp[0].ToString("f4")+"μm\r\n";
			 str += "Min "+peakn[0].ToString("f4")+"μm\r\n";

			 str += "Average Orientation(t) Error :\r\n";
			 str += ave[1].ToString("f4")+"μrad\r\n";
			 str += "Average Orientation(t) Error(1-) :\r\n";
			 str += ave2[1].ToString("f4")+"μrad\r\n";
			 str += "Peak Orientation(t) Error :\r\n";
			 str += "Max "+peakp[1].ToString("f4")+"μrad\r\n";
			 str += "Min "+peakn[1].ToString("f4")+"μrad\r\n";

			 str += "Average Orientation(td) Error :\r\n";
			 str += ave[2].ToString("f4")+"μrad\r\n";
			 str += "Average Orientation(td) Error(1-) :\r\n";
			 str += ave2[2].ToString("f4")+"μrad\r\n";
			 str += "Peak Orientation(td) Error :\r\n";
			 str += "Max "+peakp[2].ToString("f4")+"μrad\r\n";
			 str += "Min "+peakn[2].ToString("f4")+"μrad\r\n";

			 this->Invoke(deleSetNotifyText, String::Format(str));
			 
			 is_be_controling = false;
		 }

private: System::Void ControlStopBottun_Click(System::Object^  sender, System::EventArgs^  e) {
			 if(!is_be_controling)
				 return;
			 ControlStop();
		 }

//---------------------------------------------------------
// コントロール操作用同期関数
//---------------------------------------------------------
delegate void SetNotifyDelegate(String^ str);
delegate void SetNotifyClearDelegate();
delegate void SetProgressDelegate(int pro);
void SetNotifyText(String^ str){textBoxProfile->AppendText(str);}
void SetNotifyClear(){textBoxProfile->Clear();}
void SetProgress(int pro)
{
	progressBar1->Value = pro;
}

private: System::Void radioButtonAngle_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 if(radioAngle->Checked){
				 this->textBoxRad->Enabled = true;
				 this->textBoxMaxCount->Enabled = false;
			 }else{
				 this->textBoxRad->Enabled = false;
				 this->textBoxMaxCount->Enabled = true;
			 }
		 }
private: System::Void textBoxMaxCount_TextChanged(System::Object^  sender, System::EventArgs^  e) {
			 using namespace define;

			 double totaltime = double::Parse(textBoxMaxCount->Text);
			 unsigned int act_count = static_cast<unsigned int>(totaltime / SAMPLING_TIME);
			 totaltime = act_count * SAMPLING_TIME;
			 textBoxMaxCount->Text = totaltime.ToString();
		 }
private: double AverageAbs(double input[],size_t n){
			 double sum = 0;

			 for(unsigned int i=0;i<n;i++)
				 sum += abs(input[i]);

			 return sum / (double)n;
		 }
private: double PeakPos(double input[],size_t n){
			double temp;

			temp = input[0];
			for(unsigned int i=1;i<n;i++){
				if(temp < input[i])
					temp = input[i];
			}

			return temp;
		}

private: double PeakNeg(double input[],size_t n){
			double temp;

			temp = input[0];
			for(unsigned int i=1;i<n;i++){
				if(temp > input[i])
					temp = input[i];
			}

			return temp;
		}
private: System::Void radioButtonSelectInd_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 if(radioButtonSelectInd->Checked){
				 textBoxPoleT->Enabled = false;
				 textBoxPoleN->Enabled = false;
				 textBoxPoleB->Enabled = false;
				 textBoxPoleI->Enabled = false;
				 textBoxPoleJ->Enabled = false;
				 textBoxPoleK->Enabled = false;
				 textBoxPoleX->Enabled = true;
				 textBoxPoleY1->Enabled = true;
				 textBoxPoleZ->Enabled = true;
				 textBoxPoleC->Enabled = true;
				 textBoxPoleA1->Enabled = true;
				 if(!checkBoxSync->Checked){
					 textBoxPoleY2->Enabled = true;
					 textBoxPoleA2->Enabled = true;
				 }
			 }else{
				 textBoxPoleT->Enabled = true;
				 textBoxPoleN->Enabled = true;
				 textBoxPoleB->Enabled = true;
				 textBoxPoleI->Enabled = true;
				 textBoxPoleJ->Enabled = true;
				 textBoxPoleK->Enabled = true;
				 textBoxPoleX->Enabled = false;
				 textBoxPoleY1->Enabled = false;
				 textBoxPoleZ->Enabled = false;
				 textBoxPoleC->Enabled = true;
				 textBoxPoleA1->Enabled = true;
				 if(!checkBoxSync->Checked){
					 textBoxPoleY2->Enabled = false;
					 textBoxPoleA2->Enabled = true;
				 }
			 }
		 }
private: System::Void checkBoxSync_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 if(checkBoxSync->Checked){
				 textBoxPoleY2->Enabled = false;
				 textBoxPoleA2->Enabled = false;
				 textBoxPoleSyncY->Enabled = true;
				 textBoxPoleSyncA->Enabled = true;
			 }else{
				 if(radioButtonSelectInd->Checked){
					 textBoxPoleY2->Enabled = true;
					 textBoxPoleA2->Enabled = true;
				 }else{
					 textBoxPoleY2->Enabled = false;
					 textBoxPoleA2->Enabled = true;
				 }
				 textBoxPoleSyncY->Enabled = false;
				 textBoxPoleSyncA->Enabled = false;
			 }
		 }
private: void CalcMandC(double M[],double Fri[],array<double,2>^ datalog){
			 using namespace define;
			 vector<double> Ans(2),F(max_step),Ff(F);
			 matrix<double> Phi(max_step,2),tPhi,invPhi,Phif(Phi);
			 int force_index[] = {16,24,32,40,48,56,64},
				 acc_index[] = {15,23,31,39,47,55,63},
				 vel_index[] = {14,22,30,38,46,54,62};
			 double cut=2,
				 a = (SAMPLING_TIME*cut)/(SAMPLING_TIME*cut+2),
				 b = (SAMPLING_TIME*cut-2)/(SAMPLING_TIME*cut+2);
			

			 for(unsigned int i=0;i<define::NUM_ACTUATOR;i++){
				 for(unsigned int j=0;j<max_step;j++){
					 F(j) = datalog[force_index[i],j];		// force
					 Phi(j,0) = datalog[acc_index[i],j];	// acc
					 Phi(j,1) = datalog[vel_index[i],j];	// vel
				 }
				 Phif.clear();
				 Ff.clear();
				 Ff(0) = a*F(0);
				 row(Phif,0) = a*(row(Phi,0));
				 for(unsigned int k=1;k<max_step;k++){
					 Ff(k) = a*( F(k)+F(k-1) ) - b*Ff(k-1);
					 row(Phif,k) = a*(row(Phi,k)+row(Phi,k-1)) - b*row(Phif,k-1);
				 }
				 Phi = Phif;
				 F = Ff;
				 tPhi = trans(Phi);
				 math::invert(prod(tPhi,Phi),invPhi);
			 
				 Ans = prod(prod(invPhi,tPhi),F);

				 M[i] = Ans(0);
				 Fri[i] = Ans(1);
			 }
		 }
private: void CalcOptContouringError(array<double,2>^ datalog,bool select_3d){
			 using namespace define;

			 double ax,bx,cx,t,tol=1.0e-7,xmin,fmin,td;
			 vector<double> co(3),val(2);

			 for(unsigned int i=0;i<max_step;i++){
				 t = datalog[0,i];
				 td = 0.01;
				 ax = t + td;
				 cx = t - td;
				 bx = ( ax+cx ) / 2;
				 co(0) = datalog[13,i];	// x
				 co(1) = datalog[21,i];	// y
				 co(2) = datalog[37,i];	// z
				 double fa,fb,fc;
				 if(select_3d){
					 optimization::golden::mnbrak(ax,bx,cx,fa,fb,fc,co,val,optimization::function3d);
					 fmin = optimization::golden::neogolden(ax,bx,cx,co,val,optimization::function3d,tol,xmin);
				 }else{
					 optimization::golden::mnbrak(ax,bx,cx,fa,fb,fc,co,val,optimization::function2d);
					 fmin = optimization::golden::neogolden(ax,bx,cx,co,val,optimization::function2d,tol,xmin);
				 }

				 datalog[1,i] = fmin;	// ec

				 if(select_3d){
					 double tmin=val(0);
					 vector<double>	wa(NUM_DOF),wadot(wa),waddot(wa),
									wrt(wa),wrtdot(wa),wrtddot(wa),
									wropt(wa),wroptdot(wa),wroptddot(wa),
									Oa(3),Ort(Oa),Oropt(Oa);
					 wa(3) = datalog[45,i];	// c
					 wa(4) = datalog[53,i];	// a1
					 Oa = control3d::CalcOrientation(wa);
					 //control3d::GenerateReference(t,wrt,wrtdot,wrtddot);
					 (*control3d::GenR)(t,wrt,wrtdot,wrtddot);
					 Ort = control3d::CalcOrientation(wrt);
					 //control3d::GenerateReference(tmin,wropt,wroptdot,wroptddot);
					 (*control3d::GenR)(tmin,wropt,wroptdot,wroptddot);
					 Oropt = control3d::CalcOrientation(wropt);
					 
					 double phi_t,phi_td;
					 phi_t = acos( inner_prod(Ort,Oa) );
					 phi_td = acos( inner_prod(Oropt,Oa) );
				 			 
					 datalog[66,i] = phi_t;		// phi_t
					 datalog[67,i] = phi_td;	// phi_td
				 }
			 }
		 }

private: System::Void radioButtonSelect3D_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 if(radioButtonSelect3D->Checked)
				 select_3d = true;
			 else
				 select_3d = false;
			 
		 }
private: System::Void ButtonIdentification_Click(System::Object^  sender, System::EventArgs^  e) {
			 using namespace define;

			 if(is_be_controling)
				 return;

			 FormIdentification^ FormObj = gcnew FormIdentification(); // 子ﾌｫｰﾑ生成
			 FormObj->AioId = AioId;
			 FormObj->CntId = CntId;
			 FormObj->AioDeviceName = AIOText->Text;
			 FormObj->CntDeviceName = CNTText->Text;

			 FormObj->ShowDialog(); // 子ﾌｫｰﾑを表示（モーダル）
		 }
};
}

