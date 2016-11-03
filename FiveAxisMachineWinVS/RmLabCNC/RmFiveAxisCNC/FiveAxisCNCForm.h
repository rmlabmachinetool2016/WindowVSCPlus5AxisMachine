#pragma once
namespace RmFiveAxisCNC{

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::Windows::Forms::DataVisualization::Charting;
	using namespace System::Diagnostics;
	using namespace RmLabCNC;
	/// <summary>
	/// Summary for FiveAxisCNCForm
	/// </summary>
	public ref class FiveAxisCNCForm : public System::Windows::Forms::Form
	{
#pragma region Windows Form Designer generated variable
	public:
		FiveAxisCNCForm(void)
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
		~FiveAxisCNCForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::TabControl^  FiveAxisCNCTabControl;
	private: System::Windows::Forms::TabPage^  tabPageMachining;
	protected: 

	protected: 

	protected: 

	protected: 

	protected: 

	private: System::Windows::Forms::RadioButton^  radioButton1;
	private: System::Windows::Forms::TabPage^  tabPageExperiment;
	private: System::Windows::Forms::Button^  buttonEditGcodeFile;

	private: System::Windows::Forms::Button^  buttonRecentGcode;

	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::TextBox^  textBoxGcodeFilename;

	private: System::Windows::Forms::Button^  buttonLoadGcode;
	private: System::Windows::Forms::TextBox^  textBoxGcodeContent;
	private: System::Windows::Forms::Label^  labelConnectStatus;



	private: System::Windows::Forms::Button^  buttonStop;
	private: System::Windows::Forms::Button^  buttonExperimentStartPause;

	private: System::Windows::Forms::Button^  buttonIOConnection;
	private: System::Windows::Forms::Button^  buttonEmergencyStop;






	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::TextBox^  textBoxConfigFilename;

	private: System::Windows::Forms::Button^  buttonRecentConfig;

	private: System::Windows::Forms::Button^  buttonLoadConfig;

	private: System::Windows::Forms::PictureBox^  pictureBoxFiveAxisCNC;

	private: System::Windows::Forms::TextBox^  textBoxA2Encoder;

	private: System::Windows::Forms::TextBox^  textBoxA1Encoder;

	private: System::Windows::Forms::TextBox^  textBoxCEncoder;

	private: System::Windows::Forms::TextBox^  textBoxZEncoder;

	private: System::Windows::Forms::TextBox^  textBoxY2Encoder;

	private: System::Windows::Forms::TextBox^  textBoxY1Encoder;

	private: System::Windows::Forms::TextBox^  textBoxXEncoder;


	private: System::Windows::Forms::TextBox^  textBoxA2Position;

	private: System::Windows::Forms::TextBox^  textBoxA1Position;

	private: System::Windows::Forms::TextBox^  textBoxCPosition;

	private: System::Windows::Forms::TextBox^  textBoxZPosition;

	private: System::Windows::Forms::TextBox^  textBoxY2Position;

	private: System::Windows::Forms::TextBox^  textBoxY1Position;

	private: System::Windows::Forms::TextBox^  textBoxXPosition;

	private: System::Windows::Forms::TabPage^  tabPageResultGraph;
	private: System::Windows::Forms::TextBox^  textBoxControlParameters;

	private: System::Windows::Forms::Button^  buttonSetOrigin;
private: System::Windows::Forms::Button^  buttonSimulationStart;
private: System::Windows::Forms::DataVisualization::Charting::Chart^  chartRealReferenceContour;
private: System::Windows::Forms::Button^  buttonShowGraph;
private: System::Windows::Forms::Button^  buttonClearGraph;
private: System::Windows::Forms::Button^  buttonZoomOut;
private: System::Windows::Forms::Button^  buttonUpdatePosition;
private: System::Windows::Forms::Label^  labelTimeDisplay;
private: System::Windows::Forms::Button^  buttonTestCounter;
private: System::Windows::Forms::TextBox^  textBoxTimeRun;
private: System::Windows::Forms::Button^  buttonOpenResultFile;
private: System::Windows::Forms::OpenFileDialog^  openFileDialogResultFile;
private: System::Windows::Forms::Label^  label13;
private: System::Windows::Forms::TextBox^  textBoxSampleTime;
private: System::Windows::Forms::Label^  label2;
private: System::Windows::Forms::ContextMenuStrip^  contextMenuStripRecentGcode;
private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem1;
private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem2;
private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem3;
private: System::Windows::Forms::Button^  buttonAnotherGraph;
private: System::Windows::Forms::ContextMenuStrip^  contextMenuStripAnotherGraph;
private: System::Windows::Forms::ToolStripTextBox^  RelationU_Vel;
private: System::Windows::Forms::ToolStripTextBox^  ContourError;
private: System::Windows::Forms::ToolStripTextBox^  ApsoluteError;
private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem4;
private: System::Windows::Forms::ToolStripTextBox^  toolStripTextBox1;
private: System::Windows::Forms::ToolStripTextBox^  toolStripTextBox2;
private: System::Windows::Forms::ToolStripTextBox^  toolStripTextBox3;
private: System::Windows::Forms::ToolStripComboBox^  toolStripComboBox1;




private: System::Windows::Forms::ToolStripTextBox^  toolStripTextBoxDefaultGraph;
private: System::Windows::Forms::Button^  buttonZMinus;
private: System::Windows::Forms::Button^  buttonZPlus;


private: System::Windows::Forms::Button^  buttonY1Minus;
private: System::Windows::Forms::Button^  buttonY1Plus;


private: System::Windows::Forms::Button^  buttonXMinus;

private: System::Windows::Forms::Button^  buttonXPlus;



private: System::Windows::Forms::Label^  labelManualSpeed;
private: System::Windows::Forms::RadioButton^  radioButtonManualSpeedX1;
private: System::Windows::Forms::RadioButton^  radioButtonManualSpeedX1000;



private: System::Windows::Forms::RadioButton^  radioButtonManualSpeedX100;

private: System::Windows::Forms::RadioButton^  radioButtonManualSpeedX10;
private: System::Windows::Forms::Timer^  timerFiveAxisForm;
private: System::Windows::Forms::CheckBox^  checkBoxPositionRegulation;
private: System::Windows::Forms::CheckBox^  checkBoxSaveData;

private: System::Windows::Forms::TextBox^  textBoxDataFileName;
private: System::Windows::Forms::Label^  label15;
private: System::Windows::Forms::Button^  buttonA1Plus;

private: System::Windows::Forms::Button^  buttonA1Minus;

private: System::Windows::Forms::Button^  buttonCPlus;
private: System::Windows::Forms::Button^  buttonCMinus;
private: System::Windows::Forms::TabPage^  Setting;
private: System::Windows::Forms::Button^  buttonLoadSelectedSetting;
private: System::Windows::Forms::ComboBox^  comboBoxLoadSetting;
private: System::Windows::Forms::Label^  label17;
private: System::Windows::Forms::Label^  label16;
private: System::Windows::Forms::ComboBox^  comboBoxProgramStartSetting;
private: System::Windows::Forms::CheckBox^  checkBoxLimitTime;
private: System::Windows::Forms::TextBox^  textBoxSpinSpeed;
private: System::Windows::Forms::CheckBox^  checkBoxSpinSetting;
private: System::Windows::Forms::CheckBox^  checkBoxPositionUpdate;
private: System::Windows::Forms::CheckBox^  checkBoxEncoderUpdate;
private: System::Windows::Forms::ComboBox^  comboBoxControllerType;
private: System::Windows::Forms::ComboBox^  comboBoxDisturbanceObserver;
private: System::Windows::Forms::ComboBox^  comboBoxFrictionModel;
private: System::Windows::Forms::Label^  label14;
private: System::Windows::Forms::Label^  label11;
private: System::Windows::Forms::Label^  label10;
private: System::Windows::Forms::Button^  buttonWorkpieceAMinus;
private: System::Windows::Forms::Button^  buttonWorkpieceAPlus;


private: System::Windows::Forms::TextBox^  textBoxWorkpieceA;

private: System::Windows::Forms::Label^  label23;
private: System::Windows::Forms::Button^  buttonWorkpieceCMinus;
private: System::Windows::Forms::Button^  buttonWorkpieceCPlus;


private: System::Windows::Forms::TextBox^  textBoxWorkpieceC;

private: System::Windows::Forms::Label^  label22;
private: System::Windows::Forms::Button^  buttonWorkpieceZMinus;
private: System::Windows::Forms::Button^  buttonWorkpieceZPlus;


private: System::Windows::Forms::TextBox^  textBoxWorkpieceZ;

private: System::Windows::Forms::Label^  label21;
private: System::Windows::Forms::Button^  buttonWorkpieceYMinus;
private: System::Windows::Forms::Button^  buttonWorkpieceYPlus;


private: System::Windows::Forms::TextBox^  textBoxWorkpieceY;

private: System::Windows::Forms::Label^  label20;
private: System::Windows::Forms::Button^  buttonWorkpieceXMinus;
private: System::Windows::Forms::Button^  buttonWorkpieceXPlus;



private: System::Windows::Forms::TextBox^  textBoxWorkpieceX;

private: System::Windows::Forms::Label^  label19;
private: System::Windows::Forms::Label^  label18;
private: System::Windows::Forms::Label^  label24;






private: System::ComponentModel::IContainer^  components;





	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>

#pragma endregion
#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(FiveAxisCNCForm::typeid));
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea9 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea10 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea11 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea12 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend9 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend10 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend11 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend12 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series53 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series54 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series55 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series56 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series57 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series58 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series59 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series60 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series61 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series62 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series63 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series64 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series65 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series66 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series67 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series68 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series69 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series70 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series71 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series72 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series73 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series74 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series75 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series76 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series77 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series78 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Title^  title11 = (gcnew System::Windows::Forms::DataVisualization::Charting::Title());
			System::Windows::Forms::DataVisualization::Charting::Title^  title12 = (gcnew System::Windows::Forms::DataVisualization::Charting::Title());
			System::Windows::Forms::DataVisualization::Charting::Title^  title13 = (gcnew System::Windows::Forms::DataVisualization::Charting::Title());
			System::Windows::Forms::DataVisualization::Charting::Title^  title14 = (gcnew System::Windows::Forms::DataVisualization::Charting::Title());
			System::Windows::Forms::DataVisualization::Charting::Title^  title15 = (gcnew System::Windows::Forms::DataVisualization::Charting::Title());
			this->FiveAxisCNCTabControl = (gcnew System::Windows::Forms::TabControl());
			this->tabPageMachining = (gcnew System::Windows::Forms::TabPage());
			this->radioButton1 = (gcnew System::Windows::Forms::RadioButton());
			this->tabPageExperiment = (gcnew System::Windows::Forms::TabPage());
			this->textBoxSpinSpeed = (gcnew System::Windows::Forms::TextBox());
			this->checkBoxSpinSetting = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxLimitTime = (gcnew System::Windows::Forms::CheckBox());
			this->textBoxDataFileName = (gcnew System::Windows::Forms::TextBox());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->checkBoxSaveData = (gcnew System::Windows::Forms::CheckBox());
			this->textBoxTimeRun = (gcnew System::Windows::Forms::TextBox());
			this->labelTimeDisplay = (gcnew System::Windows::Forms::Label());
			this->buttonSimulationStart = (gcnew System::Windows::Forms::Button());
			this->textBoxControlParameters = (gcnew System::Windows::Forms::TextBox());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->textBoxConfigFilename = (gcnew System::Windows::Forms::TextBox());
			this->buttonRecentConfig = (gcnew System::Windows::Forms::Button());
			this->buttonLoadConfig = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->buttonWorkpieceAMinus = (gcnew System::Windows::Forms::Button());
			this->buttonWorkpieceAPlus = (gcnew System::Windows::Forms::Button());
			this->textBoxWorkpieceA = (gcnew System::Windows::Forms::TextBox());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->buttonWorkpieceCMinus = (gcnew System::Windows::Forms::Button());
			this->buttonWorkpieceCPlus = (gcnew System::Windows::Forms::Button());
			this->textBoxWorkpieceC = (gcnew System::Windows::Forms::TextBox());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->buttonWorkpieceZMinus = (gcnew System::Windows::Forms::Button());
			this->buttonWorkpieceZPlus = (gcnew System::Windows::Forms::Button());
			this->textBoxWorkpieceZ = (gcnew System::Windows::Forms::TextBox());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->buttonWorkpieceYMinus = (gcnew System::Windows::Forms::Button());
			this->buttonWorkpieceYPlus = (gcnew System::Windows::Forms::Button());
			this->textBoxWorkpieceY = (gcnew System::Windows::Forms::TextBox());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->buttonWorkpieceXMinus = (gcnew System::Windows::Forms::Button());
			this->buttonWorkpieceXPlus = (gcnew System::Windows::Forms::Button());
			this->textBoxWorkpieceX = (gcnew System::Windows::Forms::TextBox());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->checkBoxPositionUpdate = (gcnew System::Windows::Forms::CheckBox());
			this->checkBoxEncoderUpdate = (gcnew System::Windows::Forms::CheckBox());
			this->buttonA1Plus = (gcnew System::Windows::Forms::Button());
			this->buttonA1Minus = (gcnew System::Windows::Forms::Button());
			this->buttonCPlus = (gcnew System::Windows::Forms::Button());
			this->buttonCMinus = (gcnew System::Windows::Forms::Button());
			this->checkBoxPositionRegulation = (gcnew System::Windows::Forms::CheckBox());
			this->radioButtonManualSpeedX1000 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonManualSpeedX100 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonManualSpeedX10 = (gcnew System::Windows::Forms::RadioButton());
			this->radioButtonManualSpeedX1 = (gcnew System::Windows::Forms::RadioButton());
			this->labelManualSpeed = (gcnew System::Windows::Forms::Label());
			this->buttonZMinus = (gcnew System::Windows::Forms::Button());
			this->buttonZPlus = (gcnew System::Windows::Forms::Button());
			this->buttonY1Minus = (gcnew System::Windows::Forms::Button());
			this->buttonY1Plus = (gcnew System::Windows::Forms::Button());
			this->buttonXMinus = (gcnew System::Windows::Forms::Button());
			this->buttonXPlus = (gcnew System::Windows::Forms::Button());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->textBoxSampleTime = (gcnew System::Windows::Forms::TextBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->buttonTestCounter = (gcnew System::Windows::Forms::Button());
			this->buttonUpdatePosition = (gcnew System::Windows::Forms::Button());
			this->buttonSetOrigin = (gcnew System::Windows::Forms::Button());
			this->pictureBoxFiveAxisCNC = (gcnew System::Windows::Forms::PictureBox());
			this->textBoxA2Encoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxA1Encoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxCEncoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxZEncoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxY2Encoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxY1Encoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxXEncoder = (gcnew System::Windows::Forms::TextBox());
			this->textBoxA2Position = (gcnew System::Windows::Forms::TextBox());
			this->textBoxA1Position = (gcnew System::Windows::Forms::TextBox());
			this->textBoxCPosition = (gcnew System::Windows::Forms::TextBox());
			this->textBoxZPosition = (gcnew System::Windows::Forms::TextBox());
			this->textBoxY2Position = (gcnew System::Windows::Forms::TextBox());
			this->textBoxY1Position = (gcnew System::Windows::Forms::TextBox());
			this->textBoxXPosition = (gcnew System::Windows::Forms::TextBox());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->labelConnectStatus = (gcnew System::Windows::Forms::Label());
			this->buttonStop = (gcnew System::Windows::Forms::Button());
			this->buttonExperimentStartPause = (gcnew System::Windows::Forms::Button());
			this->buttonIOConnection = (gcnew System::Windows::Forms::Button());
			this->buttonEmergencyStop = (gcnew System::Windows::Forms::Button());
			this->buttonEditGcodeFile = (gcnew System::Windows::Forms::Button());
			this->buttonRecentGcode = (gcnew System::Windows::Forms::Button());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->textBoxGcodeFilename = (gcnew System::Windows::Forms::TextBox());
			this->buttonLoadGcode = (gcnew System::Windows::Forms::Button());
			this->textBoxGcodeContent = (gcnew System::Windows::Forms::TextBox());
			this->tabPageResultGraph = (gcnew System::Windows::Forms::TabPage());
			this->buttonAnotherGraph = (gcnew System::Windows::Forms::Button());
			this->buttonOpenResultFile = (gcnew System::Windows::Forms::Button());
			this->buttonZoomOut = (gcnew System::Windows::Forms::Button());
			this->buttonClearGraph = (gcnew System::Windows::Forms::Button());
			this->buttonShowGraph = (gcnew System::Windows::Forms::Button());
			this->chartRealReferenceContour = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->Setting = (gcnew System::Windows::Forms::TabPage());
			this->comboBoxDisturbanceObserver = (gcnew System::Windows::Forms::ComboBox());
			this->comboBoxFrictionModel = (gcnew System::Windows::Forms::ComboBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->comboBoxControllerType = (gcnew System::Windows::Forms::ComboBox());
			this->buttonLoadSelectedSetting = (gcnew System::Windows::Forms::Button());
			this->comboBoxLoadSetting = (gcnew System::Windows::Forms::ComboBox());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->comboBoxProgramStartSetting = (gcnew System::Windows::Forms::ComboBox());
			this->openFileDialogResultFile = (gcnew System::Windows::Forms::OpenFileDialog());
			this->contextMenuStripRecentGcode = (gcnew System::Windows::Forms::ContextMenuStrip(this->components));
			this->toolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripMenuItem2 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripMenuItem3 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->contextMenuStripAnotherGraph = (gcnew System::Windows::Forms::ContextMenuStrip(this->components));
			this->toolStripTextBoxDefaultGraph = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->RelationU_Vel = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->ContourError = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->ApsoluteError = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->toolStripMenuItem4 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripTextBox1 = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->toolStripTextBox2 = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->toolStripTextBox3 = (gcnew System::Windows::Forms::ToolStripTextBox());
			this->toolStripComboBox1 = (gcnew System::Windows::Forms::ToolStripComboBox());
			this->timerFiveAxisForm = (gcnew System::Windows::Forms::Timer(this->components));
			this->FiveAxisCNCTabControl->SuspendLayout();
			this->tabPageMachining->SuspendLayout();
			this->tabPageExperiment->SuspendLayout();
			this->groupBox1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBoxFiveAxisCNC))->BeginInit();
			this->tabPageResultGraph->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->chartRealReferenceContour))->BeginInit();
			this->Setting->SuspendLayout();
			this->contextMenuStripRecentGcode->SuspendLayout();
			this->contextMenuStripAnotherGraph->SuspendLayout();
			this->SuspendLayout();
			// 
			// FiveAxisCNCTabControl
			// 
			this->FiveAxisCNCTabControl->Controls->Add(this->tabPageMachining);
			this->FiveAxisCNCTabControl->Controls->Add(this->tabPageExperiment);
			this->FiveAxisCNCTabControl->Controls->Add(this->tabPageResultGraph);
			this->FiveAxisCNCTabControl->Controls->Add(this->Setting);
			this->FiveAxisCNCTabControl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Regular, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->FiveAxisCNCTabControl->Location = System::Drawing::Point(3, 4);
			this->FiveAxisCNCTabControl->Name = L"FiveAxisCNCTabControl";
			this->FiveAxisCNCTabControl->SelectedIndex = 1;
			this->FiveAxisCNCTabControl->Size = System::Drawing::Size(983, 647);
			this->FiveAxisCNCTabControl->TabIndex = 0;
			// 
			// tabPageMachining
			// 
			this->tabPageMachining->BackColor = System::Drawing::Color::Transparent;
			this->tabPageMachining->Controls->Add(this->radioButton1);
			this->tabPageMachining->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->tabPageMachining->Location = System::Drawing::Point(4, 25);
			this->tabPageMachining->Name = L"tabPageMachining";
			this->tabPageMachining->Padding = System::Windows::Forms::Padding(3);
			this->tabPageMachining->Size = System::Drawing::Size(975, 618);
			this->tabPageMachining->TabIndex = 0;
			this->tabPageMachining->Text = L"Machining Tab";
			// 
			// radioButton1
			// 
			this->radioButton1->AutoSize = true;
			this->radioButton1->Checked = true;
			this->radioButton1->Location = System::Drawing::Point(19, 40);
			this->radioButton1->Name = L"radioButton1";
			this->radioButton1->Size = System::Drawing::Size(119, 24);
			this->radioButton1->TabIndex = 0;
			this->radioButton1->TabStop = true;
			this->radioButton1->Text = L"radioButton1";
			this->radioButton1->UseVisualStyleBackColor = true;
			// 
			// tabPageExperiment
			// 
			this->tabPageExperiment->BackColor = System::Drawing::Color::LightSteelBlue;
			this->tabPageExperiment->Controls->Add(this->textBoxSpinSpeed);
			this->tabPageExperiment->Controls->Add(this->checkBoxSpinSetting);
			this->tabPageExperiment->Controls->Add(this->checkBoxLimitTime);
			this->tabPageExperiment->Controls->Add(this->textBoxDataFileName);
			this->tabPageExperiment->Controls->Add(this->label15);
			this->tabPageExperiment->Controls->Add(this->checkBoxSaveData);
			this->tabPageExperiment->Controls->Add(this->textBoxTimeRun);
			this->tabPageExperiment->Controls->Add(this->labelTimeDisplay);
			this->tabPageExperiment->Controls->Add(this->buttonSimulationStart);
			this->tabPageExperiment->Controls->Add(this->textBoxControlParameters);
			this->tabPageExperiment->Controls->Add(this->label12);
			this->tabPageExperiment->Controls->Add(this->textBoxConfigFilename);
			this->tabPageExperiment->Controls->Add(this->buttonRecentConfig);
			this->tabPageExperiment->Controls->Add(this->buttonLoadConfig);
			this->tabPageExperiment->Controls->Add(this->groupBox1);
			this->tabPageExperiment->Controls->Add(this->labelConnectStatus);
			this->tabPageExperiment->Controls->Add(this->buttonStop);
			this->tabPageExperiment->Controls->Add(this->buttonExperimentStartPause);
			this->tabPageExperiment->Controls->Add(this->buttonIOConnection);
			this->tabPageExperiment->Controls->Add(this->buttonEmergencyStop);
			this->tabPageExperiment->Controls->Add(this->buttonEditGcodeFile);
			this->tabPageExperiment->Controls->Add(this->buttonRecentGcode);
			this->tabPageExperiment->Controls->Add(this->label1);
			this->tabPageExperiment->Controls->Add(this->textBoxGcodeFilename);
			this->tabPageExperiment->Controls->Add(this->buttonLoadGcode);
			this->tabPageExperiment->Controls->Add(this->textBoxGcodeContent);
			this->tabPageExperiment->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->tabPageExperiment->Location = System::Drawing::Point(4, 25);
			this->tabPageExperiment->Name = L"tabPageExperiment";
			this->tabPageExperiment->Padding = System::Windows::Forms::Padding(3);
			this->tabPageExperiment->Size = System::Drawing::Size(975, 618);
			this->tabPageExperiment->TabIndex = 1;
			this->tabPageExperiment->Text = L"Experiment Tab";
			// 
			// textBoxSpinSpeed
			// 
			this->textBoxSpinSpeed->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxSpinSpeed->Location = System::Drawing::Point(125, 427);
			this->textBoxSpinSpeed->Name = L"textBoxSpinSpeed";
			this->textBoxSpinSpeed->Size = System::Drawing::Size(83, 26);
			this->textBoxSpinSpeed->TabIndex = 54;
			this->textBoxSpinSpeed->Text = L"20";
			// 
			// checkBoxSpinSetting
			// 
			this->checkBoxSpinSetting->AutoSize = true;
			this->checkBoxSpinSetting->BackColor = System::Drawing::Color::White;
			this->checkBoxSpinSetting->BackgroundImageLayout = System::Windows::Forms::ImageLayout::None;
			this->checkBoxSpinSetting->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->checkBoxSpinSetting->ForeColor = System::Drawing::Color::Red;
			this->checkBoxSpinSetting->Location = System::Drawing::Point(127, 399);
			this->checkBoxSpinSetting->Name = L"checkBoxSpinSetting";
			this->checkBoxSpinSetting->Size = System::Drawing::Size(81, 20);
			this->checkBoxSpinSetting->TabIndex = 53;
			this->checkBoxSpinSetting->Text = L"Spin On";
			this->checkBoxSpinSetting->UseVisualStyleBackColor = false;
			this->checkBoxSpinSetting->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::checkBoxSpinSetting_MouseClick);
			// 
			// checkBoxLimitTime
			// 
			this->checkBoxLimitTime->AutoSize = true;
			this->checkBoxLimitTime->BackColor = System::Drawing::Color::White;
			this->checkBoxLimitTime->BackgroundImageLayout = System::Windows::Forms::ImageLayout::None;
			this->checkBoxLimitTime->Checked = true;
			this->checkBoxLimitTime->CheckState = System::Windows::Forms::CheckState::Checked;
			this->checkBoxLimitTime->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->checkBoxLimitTime->ForeColor = System::Drawing::Color::Red;
			this->checkBoxLimitTime->Location = System::Drawing::Point(169, 557);
			this->checkBoxLimitTime->Name = L"checkBoxLimitTime";
			this->checkBoxLimitTime->Size = System::Drawing::Size(83, 17);
			this->checkBoxLimitTime->TabIndex = 52;
			this->checkBoxLimitTime->Text = L"Limit Time";
			this->checkBoxLimitTime->UseVisualStyleBackColor = false;
			// 
			// textBoxDataFileName
			// 
			this->textBoxDataFileName->Location = System::Drawing::Point(508, 381);
			this->textBoxDataFileName->Name = L"textBoxDataFileName";
			this->textBoxDataFileName->Size = System::Drawing::Size(229, 26);
			this->textBoxDataFileName->TabIndex = 51;
			this->textBoxDataFileName->Text = L"RmFiveAxisData.rme";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label15->Location = System::Drawing::Point(355, 385);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(139, 20);
			this->label15->TabIndex = 50;
			this->label15->Text = L"Default Data out ::";
			// 
			// checkBoxSaveData
			// 
			this->checkBoxSaveData->AutoSize = true;
			this->checkBoxSaveData->BackColor = System::Drawing::Color::White;
			this->checkBoxSaveData->BackgroundImageLayout = System::Windows::Forms::ImageLayout::None;
			this->checkBoxSaveData->Checked = true;
			this->checkBoxSaveData->CheckState = System::Windows::Forms::CheckState::Checked;
			this->checkBoxSaveData->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->checkBoxSaveData->ForeColor = System::Drawing::Color::Blue;
			this->checkBoxSaveData->Location = System::Drawing::Point(259, 557);
			this->checkBoxSaveData->Name = L"checkBoxSaveData";
			this->checkBoxSaveData->Size = System::Drawing::Size(100, 20);
			this->checkBoxSaveData->TabIndex = 49;
			this->checkBoxSaveData->Text = L"Save Data";
			this->checkBoxSaveData->UseVisualStyleBackColor = false;
			// 
			// textBoxTimeRun
			// 
			this->textBoxTimeRun->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxTimeRun->Location = System::Drawing::Point(169, 582);
			this->textBoxTimeRun->Name = L"textBoxTimeRun";
			this->textBoxTimeRun->Size = System::Drawing::Size(83, 26);
			this->textBoxTimeRun->TabIndex = 47;
			this->textBoxTimeRun->Text = L"0.5";
			// 
			// labelTimeDisplay
			// 
			this->labelTimeDisplay->AutoSize = true;
			this->labelTimeDisplay->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelTimeDisplay->Location = System::Drawing::Point(258, 585);
			this->labelTimeDisplay->Name = L"labelTimeDisplay";
			this->labelTimeDisplay->Size = System::Drawing::Size(43, 20);
			this->labelTimeDisplay->TabIndex = 28;
			this->labelTimeDisplay->Text = L"Time";
			// 
			// buttonSimulationStart
			// 
			this->buttonSimulationStart->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonSimulationStart->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->buttonSimulationStart->Location = System::Drawing::Point(383, 505);
			this->buttonSimulationStart->Name = L"buttonSimulationStart";
			this->buttonSimulationStart->Size = System::Drawing::Size(111, 46);
			this->buttonSimulationStart->TabIndex = 27;
			this->buttonSimulationStart->Text = L"Start Simulation";
			this->buttonSimulationStart->UseVisualStyleBackColor = true;
			this->buttonSimulationStart->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonSimulationStart_Click);
			// 
			// textBoxControlParameters
			// 
			this->textBoxControlParameters->BackColor = System::Drawing::SystemColors::Control;
			this->textBoxControlParameters->Font = (gcnew System::Drawing::Font(L"Times New Roman", 14.25F, System::Drawing::FontStyle::Regular, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->textBoxControlParameters->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->textBoxControlParameters->Location = System::Drawing::Point(508, 413);
			this->textBoxControlParameters->Multiline = true;
			this->textBoxControlParameters->Name = L"textBoxControlParameters";
			this->textBoxControlParameters->Size = System::Drawing::Size(461, 195);
			this->textBoxControlParameters->TabIndex = 26;
			this->textBoxControlParameters->Text = L"Control Equation and Value";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label12->Location = System::Drawing::Point(354, 358);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(144, 20);
			this->label12->TabIndex = 25;
			this->label12->Text = L"Default config file ::";
			// 
			// textBoxConfigFilename
			// 
			this->textBoxConfigFilename->Location = System::Drawing::Point(508, 352);
			this->textBoxConfigFilename->Name = L"textBoxConfigFilename";
			this->textBoxConfigFilename->Size = System::Drawing::Size(461, 26);
			this->textBoxConfigFilename->TabIndex = 24;
			// 
			// buttonRecentConfig
			// 
			this->buttonRecentConfig->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonRecentConfig->ForeColor = System::Drawing::Color::Navy;
			this->buttonRecentConfig->Location = System::Drawing::Point(359, 454);
			this->buttonRecentConfig->Name = L"buttonRecentConfig";
			this->buttonRecentConfig->Size = System::Drawing::Size(139, 29);
			this->buttonRecentConfig->TabIndex = 23;
			this->buttonRecentConfig->Text = L"Recent config file";
			this->buttonRecentConfig->UseVisualStyleBackColor = true;
			this->buttonRecentConfig->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonRecentConfig_Click);
			// 
			// buttonLoadConfig
			// 
			this->buttonLoadConfig->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonLoadConfig->ForeColor = System::Drawing::Color::Navy;
			this->buttonLoadConfig->Location = System::Drawing::Point(359, 416);
			this->buttonLoadConfig->Name = L"buttonLoadConfig";
			this->buttonLoadConfig->Size = System::Drawing::Size(139, 29);
			this->buttonLoadConfig->TabIndex = 22;
			this->buttonLoadConfig->Text = L"Load config file";
			this->buttonLoadConfig->UseVisualStyleBackColor = true;
			this->buttonLoadConfig->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonLoadConfig_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->label24);
			this->groupBox1->Controls->Add(this->buttonWorkpieceAMinus);
			this->groupBox1->Controls->Add(this->buttonWorkpieceAPlus);
			this->groupBox1->Controls->Add(this->textBoxWorkpieceA);
			this->groupBox1->Controls->Add(this->label23);
			this->groupBox1->Controls->Add(this->buttonWorkpieceCMinus);
			this->groupBox1->Controls->Add(this->buttonWorkpieceCPlus);
			this->groupBox1->Controls->Add(this->textBoxWorkpieceC);
			this->groupBox1->Controls->Add(this->label22);
			this->groupBox1->Controls->Add(this->buttonWorkpieceZMinus);
			this->groupBox1->Controls->Add(this->buttonWorkpieceZPlus);
			this->groupBox1->Controls->Add(this->textBoxWorkpieceZ);
			this->groupBox1->Controls->Add(this->label21);
			this->groupBox1->Controls->Add(this->buttonWorkpieceYMinus);
			this->groupBox1->Controls->Add(this->buttonWorkpieceYPlus);
			this->groupBox1->Controls->Add(this->textBoxWorkpieceY);
			this->groupBox1->Controls->Add(this->label20);
			this->groupBox1->Controls->Add(this->buttonWorkpieceXMinus);
			this->groupBox1->Controls->Add(this->buttonWorkpieceXPlus);
			this->groupBox1->Controls->Add(this->textBoxWorkpieceX);
			this->groupBox1->Controls->Add(this->label19);
			this->groupBox1->Controls->Add(this->label18);
			this->groupBox1->Controls->Add(this->checkBoxPositionUpdate);
			this->groupBox1->Controls->Add(this->checkBoxEncoderUpdate);
			this->groupBox1->Controls->Add(this->buttonA1Plus);
			this->groupBox1->Controls->Add(this->buttonA1Minus);
			this->groupBox1->Controls->Add(this->buttonCPlus);
			this->groupBox1->Controls->Add(this->buttonCMinus);
			this->groupBox1->Controls->Add(this->checkBoxPositionRegulation);
			this->groupBox1->Controls->Add(this->radioButtonManualSpeedX1000);
			this->groupBox1->Controls->Add(this->radioButtonManualSpeedX100);
			this->groupBox1->Controls->Add(this->radioButtonManualSpeedX10);
			this->groupBox1->Controls->Add(this->radioButtonManualSpeedX1);
			this->groupBox1->Controls->Add(this->labelManualSpeed);
			this->groupBox1->Controls->Add(this->buttonZMinus);
			this->groupBox1->Controls->Add(this->buttonZPlus);
			this->groupBox1->Controls->Add(this->buttonY1Minus);
			this->groupBox1->Controls->Add(this->buttonY1Plus);
			this->groupBox1->Controls->Add(this->buttonXMinus);
			this->groupBox1->Controls->Add(this->buttonXPlus);
			this->groupBox1->Controls->Add(this->label13);
			this->groupBox1->Controls->Add(this->textBoxSampleTime);
			this->groupBox1->Controls->Add(this->label2);
			this->groupBox1->Controls->Add(this->buttonTestCounter);
			this->groupBox1->Controls->Add(this->buttonUpdatePosition);
			this->groupBox1->Controls->Add(this->buttonSetOrigin);
			this->groupBox1->Controls->Add(this->pictureBoxFiveAxisCNC);
			this->groupBox1->Controls->Add(this->textBoxA2Encoder);
			this->groupBox1->Controls->Add(this->textBoxA1Encoder);
			this->groupBox1->Controls->Add(this->textBoxCEncoder);
			this->groupBox1->Controls->Add(this->textBoxZEncoder);
			this->groupBox1->Controls->Add(this->textBoxY2Encoder);
			this->groupBox1->Controls->Add(this->textBoxY1Encoder);
			this->groupBox1->Controls->Add(this->textBoxXEncoder);
			this->groupBox1->Controls->Add(this->textBoxA2Position);
			this->groupBox1->Controls->Add(this->textBoxA1Position);
			this->groupBox1->Controls->Add(this->textBoxCPosition);
			this->groupBox1->Controls->Add(this->textBoxZPosition);
			this->groupBox1->Controls->Add(this->textBoxY2Position);
			this->groupBox1->Controls->Add(this->textBoxY1Position);
			this->groupBox1->Controls->Add(this->textBoxXPosition);
			this->groupBox1->Controls->Add(this->label9);
			this->groupBox1->Controls->Add(this->label6);
			this->groupBox1->Controls->Add(this->label7);
			this->groupBox1->Controls->Add(this->label8);
			this->groupBox1->Controls->Add(this->label5);
			this->groupBox1->Controls->Add(this->label4);
			this->groupBox1->Controls->Add(this->label3);
			this->groupBox1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->groupBox1->Location = System::Drawing::Point(104, 7);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(868, 339);
			this->groupBox1->TabIndex = 21;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Five Axis CNC Information";
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label24->ForeColor = System::Drawing::Color::DarkGreen;
			this->label24->Location = System::Drawing::Point(301, 45);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(142, 20);
			this->label24->TabIndex = 89;
			this->label24->Text = L"Machine Cordinate";
			// 
			// buttonWorkpieceAMinus
			// 
			this->buttonWorkpieceAMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceAMinus.Image")));
			this->buttonWorkpieceAMinus->Location = System::Drawing::Point(151, 224);
			this->buttonWorkpieceAMinus->Name = L"buttonWorkpieceAMinus";
			this->buttonWorkpieceAMinus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceAMinus->TabIndex = 88;
			this->buttonWorkpieceAMinus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceAMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceAMinus_MouseDown);
			this->buttonWorkpieceAMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceAMinus_MouseUp);
			// 
			// buttonWorkpieceAPlus
			// 
			this->buttonWorkpieceAPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonWorkpieceAPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceAPlus.Image")));
			this->buttonWorkpieceAPlus->Location = System::Drawing::Point(188, 224);
			this->buttonWorkpieceAPlus->Name = L"buttonWorkpieceAPlus";
			this->buttonWorkpieceAPlus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceAPlus->TabIndex = 87;
			this->buttonWorkpieceAPlus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceAPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceAPlus_MouseDown);
			this->buttonWorkpieceAPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceAPlus_MouseUp);
			// 
			// textBoxWorkpieceA
			// 
			this->textBoxWorkpieceA->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxWorkpieceA->Location = System::Drawing::Point(37, 224);
			this->textBoxWorkpieceA->Name = L"textBoxWorkpieceA";
			this->textBoxWorkpieceA->Size = System::Drawing::Size(111, 26);
			this->textBoxWorkpieceA->TabIndex = 86;
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label23->ForeColor = System::Drawing::Color::Red;
			this->label23->Location = System::Drawing::Point(10, 224);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(21, 20);
			this->label23->TabIndex = 85;
			this->label23->Text = L"A";
			// 
			// buttonWorkpieceCMinus
			// 
			this->buttonWorkpieceCMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceCMinus.Image")));
			this->buttonWorkpieceCMinus->Location = System::Drawing::Point(151, 193);
			this->buttonWorkpieceCMinus->Name = L"buttonWorkpieceCMinus";
			this->buttonWorkpieceCMinus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceCMinus->TabIndex = 84;
			this->buttonWorkpieceCMinus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceCMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceCMinus_MouseDown);
			this->buttonWorkpieceCMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceCMinus_MouseUp);
			// 
			// buttonWorkpieceCPlus
			// 
			this->buttonWorkpieceCPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonWorkpieceCPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceCPlus.Image")));
			this->buttonWorkpieceCPlus->Location = System::Drawing::Point(188, 193);
			this->buttonWorkpieceCPlus->Name = L"buttonWorkpieceCPlus";
			this->buttonWorkpieceCPlus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceCPlus->TabIndex = 83;
			this->buttonWorkpieceCPlus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceCPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceCPlus_MouseDown);
			this->buttonWorkpieceCPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceCPlus_MouseUp);
			// 
			// textBoxWorkpieceC
			// 
			this->textBoxWorkpieceC->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxWorkpieceC->Location = System::Drawing::Point(37, 193);
			this->textBoxWorkpieceC->Name = L"textBoxWorkpieceC";
			this->textBoxWorkpieceC->Size = System::Drawing::Size(111, 26);
			this->textBoxWorkpieceC->TabIndex = 82;
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label22->ForeColor = System::Drawing::Color::Red;
			this->label22->Location = System::Drawing::Point(10, 193);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(21, 20);
			this->label22->TabIndex = 81;
			this->label22->Text = L"C";
			// 
			// buttonWorkpieceZMinus
			// 
			this->buttonWorkpieceZMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceZMinus.Image")));
			this->buttonWorkpieceZMinus->Location = System::Drawing::Point(151, 144);
			this->buttonWorkpieceZMinus->Name = L"buttonWorkpieceZMinus";
			this->buttonWorkpieceZMinus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceZMinus->TabIndex = 80;
			this->buttonWorkpieceZMinus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceZMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceZMinus_MouseDown);
			this->buttonWorkpieceZMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceZMinus_MouseUp);
			// 
			// buttonWorkpieceZPlus
			// 
			this->buttonWorkpieceZPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonWorkpieceZPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceZPlus.Image")));
			this->buttonWorkpieceZPlus->Location = System::Drawing::Point(188, 144);
			this->buttonWorkpieceZPlus->Name = L"buttonWorkpieceZPlus";
			this->buttonWorkpieceZPlus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceZPlus->TabIndex = 79;
			this->buttonWorkpieceZPlus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceZPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceZPlus_MouseDown);
			this->buttonWorkpieceZPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceZPlus_MouseUp);
			// 
			// textBoxWorkpieceZ
			// 
			this->textBoxWorkpieceZ->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxWorkpieceZ->Location = System::Drawing::Point(37, 144);
			this->textBoxWorkpieceZ->Name = L"textBoxWorkpieceZ";
			this->textBoxWorkpieceZ->Size = System::Drawing::Size(111, 26);
			this->textBoxWorkpieceZ->TabIndex = 78;
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label21->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->label21->Location = System::Drawing::Point(10, 144);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(20, 20);
			this->label21->TabIndex = 77;
			this->label21->Text = L"Z";
			// 
			// buttonWorkpieceYMinus
			// 
			this->buttonWorkpieceYMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceYMinus.Image")));
			this->buttonWorkpieceYMinus->Location = System::Drawing::Point(151, 112);
			this->buttonWorkpieceYMinus->Name = L"buttonWorkpieceYMinus";
			this->buttonWorkpieceYMinus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceYMinus->TabIndex = 76;
			this->buttonWorkpieceYMinus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceYMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceYMinus_MouseDown);
			this->buttonWorkpieceYMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceYMinus_MouseUp);
			// 
			// buttonWorkpieceYPlus
			// 
			this->buttonWorkpieceYPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonWorkpieceYPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceYPlus.Image")));
			this->buttonWorkpieceYPlus->Location = System::Drawing::Point(188, 112);
			this->buttonWorkpieceYPlus->Name = L"buttonWorkpieceYPlus";
			this->buttonWorkpieceYPlus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceYPlus->TabIndex = 75;
			this->buttonWorkpieceYPlus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceYPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceYPlus_MouseDown);
			this->buttonWorkpieceYPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceYPlus_MouseUp);
			// 
			// textBoxWorkpieceY
			// 
			this->textBoxWorkpieceY->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxWorkpieceY->Location = System::Drawing::Point(37, 112);
			this->textBoxWorkpieceY->Name = L"textBoxWorkpieceY";
			this->textBoxWorkpieceY->Size = System::Drawing::Size(111, 26);
			this->textBoxWorkpieceY->TabIndex = 74;
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label20->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->label20->Location = System::Drawing::Point(10, 112);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(21, 20);
			this->label20->TabIndex = 73;
			this->label20->Text = L"Y";
			// 
			// buttonWorkpieceXMinus
			// 
			this->buttonWorkpieceXMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceXMinus.Image")));
			this->buttonWorkpieceXMinus->Location = System::Drawing::Point(151, 80);
			this->buttonWorkpieceXMinus->Name = L"buttonWorkpieceXMinus";
			this->buttonWorkpieceXMinus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceXMinus->TabIndex = 72;
			this->buttonWorkpieceXMinus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceXMinus->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonWorkpieceXMinus_Click);
			this->buttonWorkpieceXMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceXMinus_MouseDown);
			this->buttonWorkpieceXMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceXMinus_MouseUp);
			// 
			// buttonWorkpieceXPlus
			// 
			this->buttonWorkpieceXPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonWorkpieceXPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonWorkpieceXPlus.Image")));
			this->buttonWorkpieceXPlus->Location = System::Drawing::Point(188, 80);
			this->buttonWorkpieceXPlus->Name = L"buttonWorkpieceXPlus";
			this->buttonWorkpieceXPlus->Size = System::Drawing::Size(31, 26);
			this->buttonWorkpieceXPlus->TabIndex = 71;
			this->buttonWorkpieceXPlus->UseVisualStyleBackColor = true;
			this->buttonWorkpieceXPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceXPlus_MouseDown);
			this->buttonWorkpieceXPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonWorkpieceXPlus_MouseUp);
			// 
			// textBoxWorkpieceX
			// 
			this->textBoxWorkpieceX->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxWorkpieceX->Location = System::Drawing::Point(37, 80);
			this->textBoxWorkpieceX->Name = L"textBoxWorkpieceX";
			this->textBoxWorkpieceX->Size = System::Drawing::Size(111, 26);
			this->textBoxWorkpieceX->TabIndex = 70;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label19->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->label19->Location = System::Drawing::Point(10, 80);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(21, 20);
			this->label19->TabIndex = 69;
			this->label19->Text = L"X";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label18->ForeColor = System::Drawing::Color::Blue;
			this->label18->Location = System::Drawing::Point(0, 43);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(157, 20);
			this->label18->TabIndex = 68;
			this->label18->Text = L"Workpiece Cordinate";
			// 
			// checkBoxPositionUpdate
			// 
			this->checkBoxPositionUpdate->AutoSize = true;
			this->checkBoxPositionUpdate->BackColor = System::Drawing::Color::White;
			this->checkBoxPositionUpdate->BackgroundImageLayout = System::Windows::Forms::ImageLayout::None;
			this->checkBoxPositionUpdate->Checked = true;
			this->checkBoxPositionUpdate->CheckState = System::Windows::Forms::CheckState::Checked;
			this->checkBoxPositionUpdate->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->checkBoxPositionUpdate->ForeColor = System::Drawing::Color::Navy;
			this->checkBoxPositionUpdate->Location = System::Drawing::Point(163, 45);
			this->checkBoxPositionUpdate->Name = L"checkBoxPositionUpdate";
			this->checkBoxPositionUpdate->Size = System::Drawing::Size(132, 20);
			this->checkBoxPositionUpdate->TabIndex = 67;
			this->checkBoxPositionUpdate->Text = L"Show pos [mm]";
			this->checkBoxPositionUpdate->UseVisualStyleBackColor = false;
			this->checkBoxPositionUpdate->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::checkBoxPositionUpdate_MouseClick);
			// 
			// checkBoxEncoderUpdate
			// 
			this->checkBoxEncoderUpdate->AutoSize = true;
			this->checkBoxEncoderUpdate->BackColor = System::Drawing::Color::White;
			this->checkBoxEncoderUpdate->BackgroundImageLayout = System::Windows::Forms::ImageLayout::None;
			this->checkBoxEncoderUpdate->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->checkBoxEncoderUpdate->ForeColor = System::Drawing::Color::Blue;
			this->checkBoxEncoderUpdate->Location = System::Drawing::Point(478, 54);
			this->checkBoxEncoderUpdate->Name = L"checkBoxEncoderUpdate";
			this->checkBoxEncoderUpdate->Size = System::Drawing::Size(129, 20);
			this->checkBoxEncoderUpdate->TabIndex = 66;
			this->checkBoxEncoderUpdate->Text = L"Encoder Value";
			this->checkBoxEncoderUpdate->UseVisualStyleBackColor = false;
			// 
			// buttonA1Plus
			// 
			this->buttonA1Plus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonA1Plus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonA1Plus.Image")));
			this->buttonA1Plus->Location = System::Drawing::Point(429, 230);
			this->buttonA1Plus->Name = L"buttonA1Plus";
			this->buttonA1Plus->Size = System::Drawing::Size(31, 26);
			this->buttonA1Plus->TabIndex = 65;
			this->buttonA1Plus->UseVisualStyleBackColor = true;
			this->buttonA1Plus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonA1Plus_MouseDown);
			this->buttonA1Plus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonA1Plus_MouseUp);
			// 
			// buttonA1Minus
			// 
			this->buttonA1Minus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonA1Minus.Image")));
			this->buttonA1Minus->Location = System::Drawing::Point(392, 230);
			this->buttonA1Minus->Name = L"buttonA1Minus";
			this->buttonA1Minus->Size = System::Drawing::Size(31, 26);
			this->buttonA1Minus->TabIndex = 64;
			this->buttonA1Minus->UseVisualStyleBackColor = true;
			this->buttonA1Minus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonA1Minus_MouseDown);
			this->buttonA1Minus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonA1Minus_MouseUp);
			// 
			// buttonCPlus
			// 
			this->buttonCPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonCPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonCPlus.Image")));
			this->buttonCPlus->Location = System::Drawing::Point(429, 199);
			this->buttonCPlus->Name = L"buttonCPlus";
			this->buttonCPlus->Size = System::Drawing::Size(31, 26);
			this->buttonCPlus->TabIndex = 63;
			this->buttonCPlus->UseVisualStyleBackColor = true;
			this->buttonCPlus->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonCPlus_Click);
			this->buttonCPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonCPlus_MouseDown);
			this->buttonCPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonCPlus_MouseUp);
			// 
			// buttonCMinus
			// 
			this->buttonCMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonCMinus.Image")));
			this->buttonCMinus->Location = System::Drawing::Point(392, 199);
			this->buttonCMinus->Name = L"buttonCMinus";
			this->buttonCMinus->Size = System::Drawing::Size(31, 26);
			this->buttonCMinus->TabIndex = 62;
			this->buttonCMinus->UseVisualStyleBackColor = true;
			this->buttonCMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonCMinus_MouseDown);
			this->buttonCMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonCMinus_MouseUp);
			// 
			// checkBoxPositionRegulation
			// 
			this->checkBoxPositionRegulation->AutoSize = true;
			this->checkBoxPositionRegulation->Location = System::Drawing::Point(450, 297);
			this->checkBoxPositionRegulation->Name = L"checkBoxPositionRegulation";
			this->checkBoxPositionRegulation->Size = System::Drawing::Size(105, 24);
			this->checkBoxPositionRegulation->TabIndex = 61;
			this->checkBoxPositionRegulation->Text = L"Regulation";
			this->checkBoxPositionRegulation->UseVisualStyleBackColor = true;
			this->checkBoxPositionRegulation->CheckStateChanged += gcnew System::EventHandler(this, &FiveAxisCNCForm::checkBoxPositionRegulation_CheckStateChanged);
			// 
			// radioButtonManualSpeedX1000
			// 
			this->radioButtonManualSpeedX1000->Appearance = System::Windows::Forms::Appearance::Button;
			this->radioButtonManualSpeedX1000->AutoSize = true;
			this->radioButtonManualSpeedX1000->Location = System::Drawing::Point(745, 17);
			this->radioButtonManualSpeedX1000->Name = L"radioButtonManualSpeedX1000";
			this->radioButtonManualSpeedX1000->Size = System::Drawing::Size(57, 30);
			this->radioButtonManualSpeedX1000->TabIndex = 60;
			this->radioButtonManualSpeedX1000->Text = L"X500";
			this->radioButtonManualSpeedX1000->UseVisualStyleBackColor = true;
			// 
			// radioButtonManualSpeedX100
			// 
			this->radioButtonManualSpeedX100->Appearance = System::Windows::Forms::Appearance::Button;
			this->radioButtonManualSpeedX100->AutoSize = true;
			this->radioButtonManualSpeedX100->Location = System::Drawing::Point(660, 17);
			this->radioButtonManualSpeedX100->Name = L"radioButtonManualSpeedX100";
			this->radioButtonManualSpeedX100->Size = System::Drawing::Size(65, 30);
			this->radioButtonManualSpeedX100->TabIndex = 59;
			this->radioButtonManualSpeedX100->Text = L" X100 ";
			this->radioButtonManualSpeedX100->UseVisualStyleBackColor = true;
			// 
			// radioButtonManualSpeedX10
			// 
			this->radioButtonManualSpeedX10->Appearance = System::Windows::Forms::Appearance::Button;
			this->radioButtonManualSpeedX10->AutoSize = true;
			this->radioButtonManualSpeedX10->Location = System::Drawing::Point(584, 17);
			this->radioButtonManualSpeedX10->Name = L"radioButtonManualSpeedX10";
			this->radioButtonManualSpeedX10->Size = System::Drawing::Size(56, 30);
			this->radioButtonManualSpeedX10->TabIndex = 58;
			this->radioButtonManualSpeedX10->Text = L" X10 ";
			this->radioButtonManualSpeedX10->UseVisualStyleBackColor = true;
			// 
			// radioButtonManualSpeedX1
			// 
			this->radioButtonManualSpeedX1->Appearance = System::Windows::Forms::Appearance::Button;
			this->radioButtonManualSpeedX1->AutoSize = true;
			this->radioButtonManualSpeedX1->Checked = true;
			this->radioButtonManualSpeedX1->Location = System::Drawing::Point(517, 17);
			this->radioButtonManualSpeedX1->Name = L"radioButtonManualSpeedX1";
			this->radioButtonManualSpeedX1->Size = System::Drawing::Size(47, 30);
			this->radioButtonManualSpeedX1->TabIndex = 48;
			this->radioButtonManualSpeedX1->TabStop = true;
			this->radioButtonManualSpeedX1->Text = L" X1 ";
			this->radioButtonManualSpeedX1->UseVisualStyleBackColor = true;
			// 
			// labelManualSpeed
			// 
			this->labelManualSpeed->AutoSize = true;
			this->labelManualSpeed->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelManualSpeed->ForeColor = System::Drawing::Color::Red;
			this->labelManualSpeed->Location = System::Drawing::Point(271, 22);
			this->labelManualSpeed->Name = L"labelManualSpeed";
			this->labelManualSpeed->Size = System::Drawing::Size(240, 20);
			this->labelManualSpeed->TabIndex = 57;
			this->labelManualSpeed->Text = L"Manual Speed: 0.01 mm/s, deg/s";
			// 
			// buttonZMinus
			// 
			this->buttonZMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonZMinus.Image")));
			this->buttonZMinus->Location = System::Drawing::Point(392, 166);
			this->buttonZMinus->Name = L"buttonZMinus";
			this->buttonZMinus->Size = System::Drawing::Size(31, 26);
			this->buttonZMinus->TabIndex = 56;
			this->buttonZMinus->UseVisualStyleBackColor = true;
			this->buttonZMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonZMinus_MouseDown);
			this->buttonZMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonZMinus_MouseUp);
			// 
			// buttonZPlus
			// 
			this->buttonZPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonZPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonZPlus.Image")));
			this->buttonZPlus->Location = System::Drawing::Point(429, 166);
			this->buttonZPlus->Name = L"buttonZPlus";
			this->buttonZPlus->Size = System::Drawing::Size(31, 26);
			this->buttonZPlus->TabIndex = 55;
			this->buttonZPlus->UseVisualStyleBackColor = true;
			this->buttonZPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonZPlus_MouseDown);
			this->buttonZPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonZPlus_MouseUp);
			// 
			// buttonY1Minus
			// 
			this->buttonY1Minus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonY1Minus.Image")));
			this->buttonY1Minus->Location = System::Drawing::Point(392, 110);
			this->buttonY1Minus->Name = L"buttonY1Minus";
			this->buttonY1Minus->Size = System::Drawing::Size(31, 26);
			this->buttonY1Minus->TabIndex = 54;
			this->buttonY1Minus->UseVisualStyleBackColor = true;
			this->buttonY1Minus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonY1Minus_MouseDown);
			this->buttonY1Minus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonY1Minus_MouseUp);
			// 
			// buttonY1Plus
			// 
			this->buttonY1Plus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonY1Plus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonY1Plus.Image")));
			this->buttonY1Plus->Location = System::Drawing::Point(429, 110);
			this->buttonY1Plus->Name = L"buttonY1Plus";
			this->buttonY1Plus->Size = System::Drawing::Size(31, 26);
			this->buttonY1Plus->TabIndex = 53;
			this->buttonY1Plus->UseVisualStyleBackColor = true;
			this->buttonY1Plus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonY1Plus_MouseDown);
			this->buttonY1Plus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonY1Plus_MouseUp);
			// 
			// buttonXMinus
			// 
			this->buttonXMinus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonXMinus.Image")));
			this->buttonXMinus->Location = System::Drawing::Point(392, 78);
			this->buttonXMinus->Name = L"buttonXMinus";
			this->buttonXMinus->Size = System::Drawing::Size(31, 26);
			this->buttonXMinus->TabIndex = 52;
			this->buttonXMinus->UseVisualStyleBackColor = true;
			this->buttonXMinus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonXMinus_MouseDown);
			this->buttonXMinus->MouseLeave += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonXMinus_MouseLeave);
			this->buttonXMinus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonXMinus_MouseUp);
			// 
			// buttonXPlus
			// 
			this->buttonXPlus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonXPlus->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"buttonXPlus.Image")));
			this->buttonXPlus->Location = System::Drawing::Point(429, 78);
			this->buttonXPlus->Name = L"buttonXPlus";
			this->buttonXPlus->Size = System::Drawing::Size(31, 26);
			this->buttonXPlus->TabIndex = 51;
			this->buttonXPlus->UseVisualStyleBackColor = true;
			this->buttonXPlus->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonXPlus_MouseDown);
			this->buttonXPlus->MouseEnter += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonXPlus_MouseEnter);
			this->buttonXPlus->MouseLeave += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonXPlus_MouseLeave);
			this->buttonXPlus->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &FiveAxisCNCForm::buttonXPlus_MouseUp);
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label13->Location = System::Drawing::Point(809, 266);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(40, 20);
			this->label13->TabIndex = 50;
			this->label13->Text = L"(ms)";
			// 
			// textBoxSampleTime
			// 
			this->textBoxSampleTime->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxSampleTime->Location = System::Drawing::Point(732, 264);
			this->textBoxSampleTime->Name = L"textBoxSampleTime";
			this->textBoxSampleTime->Size = System::Drawing::Size(70, 26);
			this->textBoxSampleTime->TabIndex = 49;
			this->textBoxSampleTime->Text = L"5";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label2->Location = System::Drawing::Point(625, 267);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(101, 20);
			this->label2->TabIndex = 48;
			this->label2->Text = L"Sample Time";
			// 
			// buttonTestCounter
			// 
			this->buttonTestCounter->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonTestCounter->ForeColor = System::Drawing::Color::Maroon;
			this->buttonTestCounter->Location = System::Drawing::Point(710, 297);
			this->buttonTestCounter->Name = L"buttonTestCounter";
			this->buttonTestCounter->Size = System::Drawing::Size(139, 29);
			this->buttonTestCounter->TabIndex = 46;
			this->buttonTestCounter->Text = L"Test Counter";
			this->buttonTestCounter->UseVisualStyleBackColor = true;
			this->buttonTestCounter->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonTestCounter_Click);
			// 
			// buttonUpdatePosition
			// 
			this->buttonUpdatePosition->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonUpdatePosition->ForeColor = System::Drawing::Color::Maroon;
			this->buttonUpdatePosition->Location = System::Drawing::Point(565, 297);
			this->buttonUpdatePosition->Name = L"buttonUpdatePosition";
			this->buttonUpdatePosition->Size = System::Drawing::Size(139, 29);
			this->buttonUpdatePosition->TabIndex = 45;
			this->buttonUpdatePosition->Text = L"Update Position";
			this->buttonUpdatePosition->UseVisualStyleBackColor = true;
			this->buttonUpdatePosition->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonUpdatePosition_Click);
			// 
			// buttonSetOrigin
			// 
			this->buttonSetOrigin->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonSetOrigin->ForeColor = System::Drawing::Color::Maroon;
			this->buttonSetOrigin->Location = System::Drawing::Point(278, 297);
			this->buttonSetOrigin->Name = L"buttonSetOrigin";
			this->buttonSetOrigin->Size = System::Drawing::Size(139, 29);
			this->buttonSetOrigin->TabIndex = 27;
			this->buttonSetOrigin->Text = L"Set Origin";
			this->buttonSetOrigin->UseVisualStyleBackColor = true;
			this->buttonSetOrigin->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonSetOrigin_Click);
			// 
			// pictureBoxFiveAxisCNC
			// 
			this->pictureBoxFiveAxisCNC->ImageLocation = L"5AxisMachineMiniLocked.jpg";
			this->pictureBoxFiveAxisCNC->InitialImage = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"pictureBoxFiveAxisCNC.InitialImage")));
			this->pictureBoxFiveAxisCNC->Location = System::Drawing::Point(629, 59);
			this->pictureBoxFiveAxisCNC->Name = L"pictureBoxFiveAxisCNC";
			this->pictureBoxFiveAxisCNC->Size = System::Drawing::Size(209, 199);
			this->pictureBoxFiveAxisCNC->TabIndex = 26;
			this->pictureBoxFiveAxisCNC->TabStop = false;
			// 
			// textBoxA2Encoder
			// 
			this->textBoxA2Encoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxA2Encoder->Location = System::Drawing::Point(478, 257);
			this->textBoxA2Encoder->Name = L"textBoxA2Encoder";
			this->textBoxA2Encoder->Size = System::Drawing::Size(123, 26);
			this->textBoxA2Encoder->TabIndex = 43;
			// 
			// textBoxA1Encoder
			// 
			this->textBoxA1Encoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxA1Encoder->Location = System::Drawing::Point(478, 227);
			this->textBoxA1Encoder->Name = L"textBoxA1Encoder";
			this->textBoxA1Encoder->Size = System::Drawing::Size(123, 26);
			this->textBoxA1Encoder->TabIndex = 42;
			// 
			// textBoxCEncoder
			// 
			this->textBoxCEncoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxCEncoder->Location = System::Drawing::Point(478, 197);
			this->textBoxCEncoder->Name = L"textBoxCEncoder";
			this->textBoxCEncoder->Size = System::Drawing::Size(123, 26);
			this->textBoxCEncoder->TabIndex = 41;
			// 
			// textBoxZEncoder
			// 
			this->textBoxZEncoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxZEncoder->Location = System::Drawing::Point(478, 167);
			this->textBoxZEncoder->Name = L"textBoxZEncoder";
			this->textBoxZEncoder->Size = System::Drawing::Size(123, 26);
			this->textBoxZEncoder->TabIndex = 40;
			// 
			// textBoxY2Encoder
			// 
			this->textBoxY2Encoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxY2Encoder->Location = System::Drawing::Point(478, 137);
			this->textBoxY2Encoder->Name = L"textBoxY2Encoder";
			this->textBoxY2Encoder->Size = System::Drawing::Size(123, 26);
			this->textBoxY2Encoder->TabIndex = 39;
			// 
			// textBoxY1Encoder
			// 
			this->textBoxY1Encoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxY1Encoder->Location = System::Drawing::Point(478, 107);
			this->textBoxY1Encoder->Name = L"textBoxY1Encoder";
			this->textBoxY1Encoder->Size = System::Drawing::Size(123, 26);
			this->textBoxY1Encoder->TabIndex = 38;
			// 
			// textBoxXEncoder
			// 
			this->textBoxXEncoder->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxXEncoder->Location = System::Drawing::Point(478, 78);
			this->textBoxXEncoder->Name = L"textBoxXEncoder";
			this->textBoxXEncoder->Size = System::Drawing::Size(123, 26);
			this->textBoxXEncoder->TabIndex = 37;
			// 
			// textBoxA2Position
			// 
			this->textBoxA2Position->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxA2Position->Location = System::Drawing::Point(278, 257);
			this->textBoxA2Position->Name = L"textBoxA2Position";
			this->textBoxA2Position->Size = System::Drawing::Size(111, 26);
			this->textBoxA2Position->TabIndex = 35;
			// 
			// textBoxA1Position
			// 
			this->textBoxA1Position->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxA1Position->Location = System::Drawing::Point(278, 227);
			this->textBoxA1Position->Name = L"textBoxA1Position";
			this->textBoxA1Position->Size = System::Drawing::Size(111, 26);
			this->textBoxA1Position->TabIndex = 34;
			// 
			// textBoxCPosition
			// 
			this->textBoxCPosition->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxCPosition->Location = System::Drawing::Point(278, 197);
			this->textBoxCPosition->Name = L"textBoxCPosition";
			this->textBoxCPosition->Size = System::Drawing::Size(111, 26);
			this->textBoxCPosition->TabIndex = 33;
			// 
			// textBoxZPosition
			// 
			this->textBoxZPosition->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxZPosition->Location = System::Drawing::Point(278, 167);
			this->textBoxZPosition->Name = L"textBoxZPosition";
			this->textBoxZPosition->Size = System::Drawing::Size(111, 26);
			this->textBoxZPosition->TabIndex = 32;
			// 
			// textBoxY2Position
			// 
			this->textBoxY2Position->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxY2Position->Location = System::Drawing::Point(278, 137);
			this->textBoxY2Position->Name = L"textBoxY2Position";
			this->textBoxY2Position->Size = System::Drawing::Size(111, 26);
			this->textBoxY2Position->TabIndex = 31;
			// 
			// textBoxY1Position
			// 
			this->textBoxY1Position->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxY1Position->Location = System::Drawing::Point(278, 107);
			this->textBoxY1Position->Name = L"textBoxY1Position";
			this->textBoxY1Position->Size = System::Drawing::Size(111, 26);
			this->textBoxY1Position->TabIndex = 30;
			// 
			// textBoxXPosition
			// 
			this->textBoxXPosition->BackColor = System::Drawing::SystemColors::InactiveBorder;
			this->textBoxXPosition->Location = System::Drawing::Point(278, 78);
			this->textBoxXPosition->Name = L"textBoxXPosition";
			this->textBoxXPosition->Size = System::Drawing::Size(111, 26);
			this->textBoxXPosition->TabIndex = 29;
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label9->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label9->Location = System::Drawing::Point(251, 261);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(31, 20);
			this->label9->TabIndex = 28;
			this->label9->Text = L"A2";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label6->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label6->Location = System::Drawing::Point(251, 230);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(31, 20);
			this->label6->TabIndex = 27;
			this->label6->Text = L"A1";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label7->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->label7->Location = System::Drawing::Point(251, 199);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(21, 20);
			this->label7->TabIndex = 26;
			this->label7->Text = L"C";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label8->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->label8->Location = System::Drawing::Point(251, 169);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(20, 20);
			this->label8->TabIndex = 25;
			this->label8->Text = L"Z";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label5->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label5->Location = System::Drawing::Point(251, 139);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(31, 20);
			this->label5->TabIndex = 24;
			this->label5->Text = L"Y2";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label4->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(128)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->label4->Location = System::Drawing::Point(251, 108);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(31, 20);
			this->label4->TabIndex = 23;
			this->label4->Text = L"Y1";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label3->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->label3->Location = System::Drawing::Point(251, 78);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(21, 20);
			this->label3->TabIndex = 22;
			this->label3->Text = L"X";
			// 
			// labelConnectStatus
			// 
			this->labelConnectStatus->AutoSize = true;
			this->labelConnectStatus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->labelConnectStatus->Location = System::Drawing::Point(3, 458);
			this->labelConnectStatus->Name = L"labelConnectStatus";
			this->labelConnectStatus->Size = System::Drawing::Size(120, 20);
			this->labelConnectStatus->TabIndex = 20;
			this->labelConnectStatus->Text = L"Connect Status";
			// 
			// buttonStop
			// 
			this->buttonStop->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 18, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonStop->ForeColor = System::Drawing::Color::Red;
			this->buttonStop->Location = System::Drawing::Point(383, 562);
			this->buttonStop->Name = L"buttonStop";
			this->buttonStop->Size = System::Drawing::Size(110, 46);
			this->buttonStop->TabIndex = 19;
			this->buttonStop->Text = L"Stop";
			this->buttonStop->UseVisualStyleBackColor = true;
			this->buttonStop->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonStop_Click);
			// 
			// buttonExperimentStartPause
			// 
			this->buttonExperimentStartPause->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonExperimentStartPause->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->buttonExperimentStartPause->Location = System::Drawing::Point(214, 505);
			this->buttonExperimentStartPause->Name = L"buttonExperimentStartPause";
			this->buttonExperimentStartPause->Size = System::Drawing::Size(109, 46);
			this->buttonExperimentStartPause->TabIndex = 18;
			this->buttonExperimentStartPause->Text = L"Start Experiment";
			this->buttonExperimentStartPause->UseVisualStyleBackColor = true;
			this->buttonExperimentStartPause->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonExperimentStartPause_Click);
			// 
			// buttonIOConnection
			// 
			this->buttonIOConnection->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 9.75F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonIOConnection->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(0)));
			this->buttonIOConnection->Location = System::Drawing::Point(5, 399);
			this->buttonIOConnection->Name = L"buttonIOConnection";
			this->buttonIOConnection->Size = System::Drawing::Size(110, 46);
			this->buttonIOConnection->TabIndex = 17;
			this->buttonIOConnection->Text = L"Conect to CNC";
			this->buttonIOConnection->UseVisualStyleBackColor = true;
			this->buttonIOConnection->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonIOConnection_Click);
			// 
			// buttonEmergencyStop
			// 
			this->buttonEmergencyStop->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Bold, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonEmergencyStop->ForeColor = System::Drawing::Color::Red;
			this->buttonEmergencyStop->Location = System::Drawing::Point(0, 553);
			this->buttonEmergencyStop->Name = L"buttonEmergencyStop";
			this->buttonEmergencyStop->Size = System::Drawing::Size(163, 56);
			this->buttonEmergencyStop->TabIndex = 16;
			this->buttonEmergencyStop->Text = L"Emergency Stop";
			this->buttonEmergencyStop->UseVisualStyleBackColor = true;
			this->buttonEmergencyStop->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonEmergencyStop_Click);
			// 
			// buttonEditGcodeFile
			// 
			this->buttonEditGcodeFile->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, 
				System::Drawing::GraphicsUnit::Point, static_cast<System::Byte>(0)));
			this->buttonEditGcodeFile->ForeColor = System::Drawing::Color::Navy;
			this->buttonEditGcodeFile->Location = System::Drawing::Point(214, 454);
			this->buttonEditGcodeFile->Name = L"buttonEditGcodeFile";
			this->buttonEditGcodeFile->Size = System::Drawing::Size(139, 29);
			this->buttonEditGcodeFile->TabIndex = 6;
			this->buttonEditGcodeFile->Text = L"Edit Gcode File";
			this->buttonEditGcodeFile->UseVisualStyleBackColor = true;
			this->buttonEditGcodeFile->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonEditGcodeFile_Click);
			// 
			// buttonRecentGcode
			// 
			this->buttonRecentGcode->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonRecentGcode->ForeColor = System::Drawing::Color::Navy;
			this->buttonRecentGcode->Location = System::Drawing::Point(214, 416);
			this->buttonRecentGcode->Name = L"buttonRecentGcode";
			this->buttonRecentGcode->Size = System::Drawing::Size(139, 29);
			this->buttonRecentGcode->TabIndex = 5;
			this->buttonRecentGcode->Text = L"Recent Gcode File";
			this->buttonRecentGcode->UseVisualStyleBackColor = true;
			this->buttonRecentGcode->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonRecentGcode_Click);
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label1->Location = System::Drawing::Point(6, 355);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(46, 20);
			this->label1->TabIndex = 4;
			this->label1->Text = L"File ::";
			// 
			// textBoxGcodeFilename
			// 
			this->textBoxGcodeFilename->Location = System::Drawing::Point(56, 349);
			this->textBoxGcodeFilename->Name = L"textBoxGcodeFilename";
			this->textBoxGcodeFilename->Size = System::Drawing::Size(297, 26);
			this->textBoxGcodeFilename->TabIndex = 3;
			// 
			// buttonLoadGcode
			// 
			this->buttonLoadGcode->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->buttonLoadGcode->ForeColor = System::Drawing::Color::Navy;
			this->buttonLoadGcode->Location = System::Drawing::Point(214, 381);
			this->buttonLoadGcode->Name = L"buttonLoadGcode";
			this->buttonLoadGcode->Size = System::Drawing::Size(139, 29);
			this->buttonLoadGcode->TabIndex = 2;
			this->buttonLoadGcode->Text = L"Load Gcode";
			this->buttonLoadGcode->UseVisualStyleBackColor = true;
			this->buttonLoadGcode->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonLoadGcode_Click);
			// 
			// textBoxGcodeContent
			// 
			this->textBoxGcodeContent->BackColor = System::Drawing::SystemColors::Control;
			this->textBoxGcodeContent->Font = (gcnew System::Drawing::Font(L"Times New Roman", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->textBoxGcodeContent->ForeColor = System::Drawing::SystemColors::HotTrack;
			this->textBoxGcodeContent->Location = System::Drawing::Point(3, 3);
			this->textBoxGcodeContent->Multiline = true;
			this->textBoxGcodeContent->Name = L"textBoxGcodeContent";
			this->textBoxGcodeContent->ScrollBars = System::Windows::Forms::ScrollBars::Vertical;
			this->textBoxGcodeContent->Size = System::Drawing::Size(95, 340);
			this->textBoxGcodeContent->TabIndex = 1;
			this->textBoxGcodeContent->Text = L"Show Gcode  content";
			// 
			// tabPageResultGraph
			// 
			this->tabPageResultGraph->BackColor = System::Drawing::Color::Transparent;
			this->tabPageResultGraph->Controls->Add(this->buttonAnotherGraph);
			this->tabPageResultGraph->Controls->Add(this->buttonOpenResultFile);
			this->tabPageResultGraph->Controls->Add(this->buttonZoomOut);
			this->tabPageResultGraph->Controls->Add(this->buttonClearGraph);
			this->tabPageResultGraph->Controls->Add(this->buttonShowGraph);
			this->tabPageResultGraph->Controls->Add(this->chartRealReferenceContour);
			this->tabPageResultGraph->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->tabPageResultGraph->Location = System::Drawing::Point(4, 25);
			this->tabPageResultGraph->Name = L"tabPageResultGraph";
			this->tabPageResultGraph->Size = System::Drawing::Size(975, 618);
			this->tabPageResultGraph->TabIndex = 2;
			this->tabPageResultGraph->Text = L"Result Graph";
			// 
			// buttonAnotherGraph
			// 
			this->buttonAnotherGraph->Location = System::Drawing::Point(542, 529);
			this->buttonAnotherGraph->Name = L"buttonAnotherGraph";
			this->buttonAnotherGraph->Size = System::Drawing::Size(123, 31);
			this->buttonAnotherGraph->TabIndex = 5;
			this->buttonAnotherGraph->Text = L"Another Graph";
			this->buttonAnotherGraph->UseVisualStyleBackColor = true;
			this->buttonAnotherGraph->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonAnotherGraph_Click);
			// 
			// buttonOpenResultFile
			// 
			this->buttonOpenResultFile->Location = System::Drawing::Point(399, 529);
			this->buttonOpenResultFile->Name = L"buttonOpenResultFile";
			this->buttonOpenResultFile->Size = System::Drawing::Size(123, 31);
			this->buttonOpenResultFile->TabIndex = 4;
			this->buttonOpenResultFile->Text = L"Result File";
			this->buttonOpenResultFile->UseVisualStyleBackColor = true;
			this->buttonOpenResultFile->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonOpenResultFile_Click);
			// 
			// buttonZoomOut
			// 
			this->buttonZoomOut->Location = System::Drawing::Point(134, 529);
			this->buttonZoomOut->Name = L"buttonZoomOut";
			this->buttonZoomOut->Size = System::Drawing::Size(120, 31);
			this->buttonZoomOut->TabIndex = 3;
			this->buttonZoomOut->Text = L"Zoom Out";
			this->buttonZoomOut->UseVisualStyleBackColor = true;
			this->buttonZoomOut->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonZoomOut_Click);
			// 
			// buttonClearGraph
			// 
			this->buttonClearGraph->Location = System::Drawing::Point(260, 529);
			this->buttonClearGraph->Name = L"buttonClearGraph";
			this->buttonClearGraph->Size = System::Drawing::Size(123, 31);
			this->buttonClearGraph->TabIndex = 2;
			this->buttonClearGraph->Text = L"Clear Graph";
			this->buttonClearGraph->UseVisualStyleBackColor = true;
			this->buttonClearGraph->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonClearGraph_Click);
			// 
			// buttonShowGraph
			// 
			this->buttonShowGraph->Location = System::Drawing::Point(5, 529);
			this->buttonShowGraph->Name = L"buttonShowGraph";
			this->buttonShowGraph->Size = System::Drawing::Size(123, 31);
			this->buttonShowGraph->TabIndex = 1;
			this->buttonShowGraph->Text = L"Show Graph";
			this->buttonShowGraph->UseVisualStyleBackColor = true;
			this->buttonShowGraph->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonShowGraph_Click);
			// 
			// chartRealReferenceContour
			// 
			this->chartRealReferenceContour->BackColor = System::Drawing::Color::AliceBlue;
			this->chartRealReferenceContour->BackImageTransparentColor = System::Drawing::Color::White;
			this->chartRealReferenceContour->BackSecondaryColor = System::Drawing::Color::DimGray;
			chartArea9->AlignmentOrientation = static_cast<System::Windows::Forms::DataVisualization::Charting::AreaAlignmentOrientations>((System::Windows::Forms::DataVisualization::Charting::AreaAlignmentOrientations::Vertical | System::Windows::Forms::DataVisualization::Charting::AreaAlignmentOrientations::Horizontal));
			chartArea9->Area3DStyle->WallWidth = 10;
			chartArea9->AxisX->LineColor = System::Drawing::Color::Lime;
			chartArea9->AxisX->LineWidth = 2;
			chartArea9->AxisX->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea9->AxisX->MajorTickMark->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea9->AxisX->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea9->AxisX->MaximumAutoSize = 100;
			chartArea9->AxisY->LineColor = System::Drawing::Color::Lime;
			chartArea9->AxisY->LineWidth = 2;
			chartArea9->AxisY->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea9->AxisY->MajorTickMark->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea9->AxisY->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea9->AxisY->MaximumAutoSize = 100;
			chartArea9->BackColor = System::Drawing::Color::Navy;
			chartArea9->CursorX->IsUserEnabled = true;
			chartArea9->CursorX->IsUserSelectionEnabled = true;
			chartArea9->CursorY->IsUserEnabled = true;
			chartArea9->CursorY->IsUserSelectionEnabled = true;
			chartArea9->InnerPlotPosition->Auto = false;
			chartArea9->InnerPlotPosition->Height = 86;
			chartArea9->InnerPlotPosition->Width = 80;
			chartArea9->InnerPlotPosition->X = 10;
			chartArea9->InnerPlotPosition->Y = 2;
			chartArea9->Name = L"ChartArea1";
			chartArea9->Position->Auto = false;
			chartArea9->Position->Height = 50;
			chartArea9->Position->Width = 50;
			chartArea10->AxisX->LabelStyle->Format = L"F2";
			chartArea10->AxisX->LabelStyle->Interval = 0;
			chartArea10->AxisX->LabelStyle->IntervalOffset = 0;
			chartArea10->AxisX->LabelStyle->IntervalOffsetType = System::Windows::Forms::DataVisualization::Charting::DateTimeIntervalType::Auto;
			chartArea10->AxisX->LineColor = System::Drawing::Color::Lime;
			chartArea10->AxisX->LineWidth = 2;
			chartArea10->AxisX->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::Dash;
			chartArea10->AxisX->MajorTickMark->LineColor = System::Drawing::Color::DodgerBlue;
			chartArea10->AxisX->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea10->AxisX->ScaleView->SmallScrollMinSize = 0.5;
			chartArea10->AxisY->IsStartedFromZero = false;
			chartArea10->AxisY->LabelStyle->Format = L"F2";
			chartArea10->AxisY->LineColor = System::Drawing::Color::Lime;
			chartArea10->AxisY->LineWidth = 2;
			chartArea10->AxisY->MajorGrid->IntervalOffset = 0;
			chartArea10->AxisY->MajorGrid->IntervalOffsetType = System::Windows::Forms::DataVisualization::Charting::DateTimeIntervalType::Number;
			chartArea10->AxisY->MajorGrid->IntervalType = System::Windows::Forms::DataVisualization::Charting::DateTimeIntervalType::Number;
			chartArea10->AxisY->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea10->AxisY->MajorTickMark->IntervalOffset = 0;
			chartArea10->AxisY->MajorTickMark->IntervalOffsetType = System::Windows::Forms::DataVisualization::Charting::DateTimeIntervalType::Number;
			chartArea10->AxisY->MajorTickMark->IntervalType = System::Windows::Forms::DataVisualization::Charting::DateTimeIntervalType::Number;
			chartArea10->AxisY->MajorTickMark->LineColor = System::Drawing::Color::DodgerBlue;
			chartArea10->AxisY->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea10->AxisY->ScaleView->SmallScrollMinSize = 0.1;
			chartArea10->BackColor = System::Drawing::Color::White;
			chartArea10->CursorX->IsUserEnabled = true;
			chartArea10->CursorX->IsUserSelectionEnabled = true;
			chartArea10->CursorX->SelectionColor = System::Drawing::Color::Yellow;
			chartArea10->CursorY->IsUserEnabled = true;
			chartArea10->CursorY->IsUserSelectionEnabled = true;
			chartArea10->CursorY->SelectionColor = System::Drawing::Color::Yellow;
			chartArea10->InnerPlotPosition->Auto = false;
			chartArea10->InnerPlotPosition->Height = 90;
			chartArea10->InnerPlotPosition->Width = 90;
			chartArea10->InnerPlotPosition->X = 10;
			chartArea10->Name = L"ChartArea2";
			chartArea10->Position->Auto = false;
			chartArea10->Position->Height = 50;
			chartArea10->Position->Width = 50;
			chartArea10->Position->Y = 50;
			chartArea11->AxisX->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			chartArea11->AxisX->LineWidth = 2;
			chartArea11->AxisX->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea11->AxisX->MajorTickMark->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea11->AxisX->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea11->AxisY->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			chartArea11->AxisY->LineWidth = 2;
			chartArea11->AxisY->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea11->AxisY->MajorTickMark->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea11->AxisY->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea11->BackColor = System::Drawing::Color::White;
			chartArea11->CursorX->IsUserEnabled = true;
			chartArea11->CursorX->IsUserSelectionEnabled = true;
			chartArea11->CursorY->IsUserEnabled = true;
			chartArea11->CursorY->IsUserSelectionEnabled = true;
			chartArea11->InnerPlotPosition->Auto = false;
			chartArea11->InnerPlotPosition->Height = 88;
			chartArea11->InnerPlotPosition->Width = 90;
			chartArea11->InnerPlotPosition->X = 8;
			chartArea11->InnerPlotPosition->Y = 2;
			chartArea11->Name = L"ChartArea3";
			chartArea11->Position->Auto = false;
			chartArea11->Position->Height = 50;
			chartArea11->Position->Width = 50;
			chartArea11->Position->X = 50;
			chartArea12->AxisX->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			chartArea12->AxisX->LineWidth = 2;
			chartArea12->AxisX->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea12->AxisX->MajorTickMark->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea12->AxisX->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea12->AxisY->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			chartArea12->AxisY->LineWidth = 2;
			chartArea12->AxisY->MajorGrid->LineDashStyle = System::Windows::Forms::DataVisualization::Charting::ChartDashStyle::DashDot;
			chartArea12->AxisY->MajorTickMark->LineColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea12->AxisY->MajorTickMark->TickMarkStyle = System::Windows::Forms::DataVisualization::Charting::TickMarkStyle::InsideArea;
			chartArea12->BackColor = System::Drawing::Color::White;
			chartArea12->CursorX->IsUserEnabled = true;
			chartArea12->CursorX->IsUserSelectionEnabled = true;
			chartArea12->CursorY->IsUserEnabled = true;
			chartArea12->CursorY->IsUserSelectionEnabled = true;
			chartArea12->InnerPlotPosition->Auto = false;
			chartArea12->InnerPlotPosition->Height = 92;
			chartArea12->InnerPlotPosition->Width = 90;
			chartArea12->InnerPlotPosition->X = 8;
			chartArea12->Name = L"ChartArea4";
			chartArea12->Position->Auto = false;
			chartArea12->Position->Height = 50;
			chartArea12->Position->Width = 50;
			chartArea12->Position->X = 50;
			chartArea12->Position->Y = 50;
			this->chartRealReferenceContour->ChartAreas->Add(chartArea9);
			this->chartRealReferenceContour->ChartAreas->Add(chartArea10);
			this->chartRealReferenceContour->ChartAreas->Add(chartArea11);
			this->chartRealReferenceContour->ChartAreas->Add(chartArea12);
			legend9->DockedToChartArea = L"ChartArea1";
			legend9->LegendStyle = System::Windows::Forms::DataVisualization::Charting::LegendStyle::Column;
			legend9->Name = L"LegendChart1";
			legend10->DockedToChartArea = L"ChartArea2";
			legend10->Name = L"LegendChart2";
			legend11->DockedToChartArea = L"ChartArea3";
			legend11->Name = L"LegendChart3";
			legend12->DockedToChartArea = L"ChartArea4";
			legend12->Name = L"LegendChart4";
			this->chartRealReferenceContour->Legends->Add(legend9);
			this->chartRealReferenceContour->Legends->Add(legend10);
			this->chartRealReferenceContour->Legends->Add(legend11);
			this->chartRealReferenceContour->Legends->Add(legend12);
			this->chartRealReferenceContour->Location = System::Drawing::Point(5, 3);
			this->chartRealReferenceContour->Margin = System::Windows::Forms::Padding(2);
			this->chartRealReferenceContour->Name = L"chartRealReferenceContour";
			this->chartRealReferenceContour->Palette = System::Windows::Forms::DataVisualization::Charting::ChartColorPalette::Bright;
			series53->ChartArea = L"ChartArea1";
			series53->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series53->Color = System::Drawing::Color::Red;
			series53->Legend = L"LegendChart1";
			series53->MarkerColor = System::Drawing::Color::Red;
			series53->MarkerSize = 2;
			series53->Name = L"RealContour";
			series53->YValuesPerPoint = 2;
			series54->ChartArea = L"ChartArea1";
			series54->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series54->Color = System::Drawing::Color::Lime;
			series54->Legend = L"LegendChart1";
			series54->MarkerColor = System::Drawing::Color::Lime;
			series54->MarkerSize = 2;
			series54->Name = L"ReferenceContour";
			series55->BorderColor = System::Drawing::Color::White;
			series55->ChartArea = L"ChartArea2";
			series55->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series55->Color = System::Drawing::Color::Navy;
			series55->Legend = L"LegendChart2";
			series55->MarkerColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			series55->MarkerSize = 2;
			series55->Name = L"XControlVoltage";
			series56->ChartArea = L"ChartArea2";
			series56->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series56->Color = System::Drawing::Color::Green;
			series56->Legend = L"LegendChart2";
			series56->MarkerColor = System::Drawing::Color::Green;
			series56->MarkerSize = 2;
			series56->Name = L"YControlVoltage";
			series57->ChartArea = L"ChartArea2";
			series57->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series57->Color = System::Drawing::Color::Red;
			series57->Legend = L"LegendChart2";
			series57->Name = L"ZControlVoltage";
			series58->ChartArea = L"ChartArea4";
			series58->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series58->Color = System::Drawing::Color::Navy;
			series58->Legend = L"LegendChart4";
			series58->MarkerColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			series58->MarkerSize = 2;
			series58->Name = L"XPredictedVoltage";
			series59->ChartArea = L"ChartArea4";
			series59->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series59->Color = System::Drawing::Color::Green;
			series59->Legend = L"LegendChart4";
			series59->MarkerColor = System::Drawing::Color::Green;
			series59->MarkerSize = 2;
			series59->Name = L"YPredictedVoltage";
			series60->ChartArea = L"ChartArea3";
			series60->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series60->Color = System::Drawing::Color::Navy;
			series60->Legend = L"LegendChart3";
			series60->MarkerColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			series60->MarkerSize = 2;
			series60->Name = L"ew0Error";
			series61->ChartArea = L"ChartArea3";
			series61->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series61->Color = System::Drawing::Color::Green;
			series61->Legend = L"LegendChart3";
			series61->MarkerColor = System::Drawing::Color::Green;
			series61->MarkerSize = 2;
			series61->Name = L"ew1Error";
			series62->ChartArea = L"ChartArea3";
			series62->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series62->Legend = L"LegendChart3";
			series62->Name = L"ec0";
			series63->ChartArea = L"ChartArea3";
			series63->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series63->Legend = L"LegendChart3";
			series63->Name = L"ec1";
			series64->ChartArea = L"ChartArea2";
			series64->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series64->Enabled = false;
			series64->Legend = L"LegendChart2";
			series64->Name = L"RefX_1";
			series65->ChartArea = L"ChartArea1";
			series65->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series65->Legend = L"LegendChart2";
			series65->Name = L"RefX_2";
			series66->ChartArea = L"ChartArea4";
			series66->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series66->Legend = L"LegendChart4";
			series66->Name = L"RefY_1";
			series67->ChartArea = L"ChartArea4";
			series67->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series67->Legend = L"LegendChart4";
			series67->Name = L"RefY_2";
			series68->ChartArea = L"ChartArea2";
			series68->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series68->Legend = L"LegendChart2";
			series68->Name = L"RealX_1";
			series69->ChartArea = L"ChartArea2";
			series69->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series69->Legend = L"LegendChart2";
			series69->Name = L"RealX_2";
			series70->ChartArea = L"ChartArea4";
			series70->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series70->Legend = L"LegendChart4";
			series70->Name = L"RealY_1";
			series71->ChartArea = L"ChartArea4";
			series71->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series71->Legend = L"LegendChart4";
			series71->Name = L"RealY_2";
			series72->ChartArea = L"ChartArea2";
			series72->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series72->Legend = L"LegendChart2";
			series72->Name = L"RefX_1_UX";
			series72->YValuesPerPoint = 2;
			series73->ChartArea = L"ChartArea4";
			series73->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series73->Legend = L"LegendChart4";
			series73->Name = L"RefY_1_UY";
			series74->ChartArea = L"ChartArea3";
			series74->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series74->Legend = L"LegendChart3";
			series74->Name = L"RefX_Time";
			series75->ChartArea = L"ChartArea3";
			series75->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series75->Legend = L"LegendChart3";
			series75->Name = L"RefY_Time";
			series76->ChartArea = L"ChartArea3";
			series76->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series76->Legend = L"LegendChart3";
			series76->Name = L"RefZ_Time";
			series77->ChartArea = L"ChartArea4";
			series77->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series77->Color = System::Drawing::Color::Red;
			series77->Legend = L"LegendChart4";
			series77->Name = L"ZPredictedVoltage";
			series78->ChartArea = L"ChartArea3";
			series78->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series78->Color = System::Drawing::Color::Red;
			series78->Legend = L"LegendChart3";
			series78->MarkerColor = System::Drawing::Color::Red;
			series78->Name = L"ew2Error";
			this->chartRealReferenceContour->Series->Add(series53);
			this->chartRealReferenceContour->Series->Add(series54);
			this->chartRealReferenceContour->Series->Add(series55);
			this->chartRealReferenceContour->Series->Add(series56);
			this->chartRealReferenceContour->Series->Add(series57);
			this->chartRealReferenceContour->Series->Add(series58);
			this->chartRealReferenceContour->Series->Add(series59);
			this->chartRealReferenceContour->Series->Add(series60);
			this->chartRealReferenceContour->Series->Add(series61);
			this->chartRealReferenceContour->Series->Add(series62);
			this->chartRealReferenceContour->Series->Add(series63);
			this->chartRealReferenceContour->Series->Add(series64);
			this->chartRealReferenceContour->Series->Add(series65);
			this->chartRealReferenceContour->Series->Add(series66);
			this->chartRealReferenceContour->Series->Add(series67);
			this->chartRealReferenceContour->Series->Add(series68);
			this->chartRealReferenceContour->Series->Add(series69);
			this->chartRealReferenceContour->Series->Add(series70);
			this->chartRealReferenceContour->Series->Add(series71);
			this->chartRealReferenceContour->Series->Add(series72);
			this->chartRealReferenceContour->Series->Add(series73);
			this->chartRealReferenceContour->Series->Add(series74);
			this->chartRealReferenceContour->Series->Add(series75);
			this->chartRealReferenceContour->Series->Add(series76);
			this->chartRealReferenceContour->Series->Add(series77);
			this->chartRealReferenceContour->Series->Add(series78);
			this->chartRealReferenceContour->Size = System::Drawing::Size(967, 522);
			this->chartRealReferenceContour->TabIndex = 0;
			this->chartRealReferenceContour->Text = L"Hello ";
			title11->Alignment = System::Drawing::ContentAlignment::TopRight;
			title11->BackColor = System::Drawing::Color::Transparent;
			title11->BorderColor = System::Drawing::Color::Transparent;
			title11->DockedToChartArea = L"ChartArea2";
			title11->ForeColor = System::Drawing::Color::Blue;
			title11->Name = L"X, Y Control Voltage Out (V)";
			title11->Text = L"X, Y Control Voltage Out (V)";
			title11->TextOrientation = System::Windows::Forms::DataVisualization::Charting::TextOrientation::Horizontal;
			title11->Visible = false;
			title12->Alignment = System::Drawing::ContentAlignment::TopRight;
			title12->DockedToChartArea = L"ChartArea1";
			title12->ForeColor = System::Drawing::Color::AliceBlue;
			title12->Name = L"Real Contour";
			title12->Text = L"Real Contour";
			title12->Visible = false;
			title13->Alignment = System::Drawing::ContentAlignment::TopRight;
			title13->DockedToChartArea = L"ChartArea3";
			title13->ForeColor = System::Drawing::Color::Blue;
			title13->Name = L"Title3";
			title13->Text = L"ew Error micro m";
			title13->Visible = false;
			title14->Alignment = System::Drawing::ContentAlignment::TopRight;
			title14->DockedToChartArea = L"ChartArea4";
			title14->ForeColor = System::Drawing::Color::Blue;
			title14->Name = L"Title4";
			title14->Text = L"X, Y Predicted Voltage out (V)";
			title14->Visible = false;
			title15->Alignment = System::Drawing::ContentAlignment::TopRight;
			title15->BackColor = System::Drawing::Color::Transparent;
			title15->DockedToChartArea = L"ChartArea1";
			title15->ForeColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(64)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)));
			title15->Name = L"Ref Contour";
			title15->Text = L"Ref Contour";
			title15->Visible = false;
			this->chartRealReferenceContour->Titles->Add(title11);
			this->chartRealReferenceContour->Titles->Add(title12);
			this->chartRealReferenceContour->Titles->Add(title13);
			this->chartRealReferenceContour->Titles->Add(title14);
			this->chartRealReferenceContour->Titles->Add(title15);
			// 
			// Setting
			// 
			this->Setting->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->Setting->Controls->Add(this->comboBoxDisturbanceObserver);
			this->Setting->Controls->Add(this->comboBoxFrictionModel);
			this->Setting->Controls->Add(this->label14);
			this->Setting->Controls->Add(this->label11);
			this->Setting->Controls->Add(this->label10);
			this->Setting->Controls->Add(this->comboBoxControllerType);
			this->Setting->Controls->Add(this->buttonLoadSelectedSetting);
			this->Setting->Controls->Add(this->comboBoxLoadSetting);
			this->Setting->Controls->Add(this->label17);
			this->Setting->Controls->Add(this->label16);
			this->Setting->Controls->Add(this->comboBoxProgramStartSetting);
			this->Setting->Location = System::Drawing::Point(4, 25);
			this->Setting->Name = L"Setting";
			this->Setting->Padding = System::Windows::Forms::Padding(3);
			this->Setting->Size = System::Drawing::Size(975, 618);
			this->Setting->TabIndex = 3;
			this->Setting->Text = L"Setting";
			// 
			// comboBoxDisturbanceObserver
			// 
			this->comboBoxDisturbanceObserver->FormattingEnabled = true;
			this->comboBoxDisturbanceObserver->Items->AddRange(gcnew cli::array< System::Object^  >(4) {L"No disturbance observer", L"Velocity estimation based  Ba", 
				L"Acceleration estimation based Khalick", L"AdaptiveFrictionCompensation Proposed Ba"});
			this->comboBoxDisturbanceObserver->Location = System::Drawing::Point(674, 161);
			this->comboBoxDisturbanceObserver->Name = L"comboBoxDisturbanceObserver";
			this->comboBoxDisturbanceObserver->Size = System::Drawing::Size(291, 24);
			this->comboBoxDisturbanceObserver->TabIndex = 10;
			// 
			// comboBoxFrictionModel
			// 
			this->comboBoxFrictionModel->FormattingEnabled = true;
			this->comboBoxFrictionModel->Items->AddRange(gcnew cli::array< System::Object^  >(7) {L"No friction compensation", L"Coulomb viscous friction model", 
				L"Sinusoidal friction model", L"The Stribeck-Coulomb-viscous  friction model", L"The Eccentric-Coulomb-viscous  friction model", 
				L"Lugre dynamic friction", L"Lugre model using nonlinear static friction"});
			this->comboBoxFrictionModel->Location = System::Drawing::Point(341, 161);
			this->comboBoxFrictionModel->Name = L"comboBoxFrictionModel";
			this->comboBoxFrictionModel->Size = System::Drawing::Size(291, 24);
			this->comboBoxFrictionModel->TabIndex = 9;
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(697, 131);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(176, 16);
			this->label14->TabIndex = 8;
			this->label14->Text = L"Select disturbance observer";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(385, 131);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(128, 16);
			this->label11->TabIndex = 7;
			this->label11->Text = L"Select friction model";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(18, 131);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(133, 16);
			this->label10->TabIndex = 6;
			this->label10->Text = L"Select controller type";
			// 
			// comboBoxControllerType
			// 
			this->comboBoxControllerType->FormattingEnabled = true;
			this->comboBoxControllerType->Items->AddRange(gcnew cli::array< System::Object^  >(3) {L"PD Tracking controller", L"PD Contouring controller", 
				L"Slidingmode contouring controller"});
			this->comboBoxControllerType->Location = System::Drawing::Point(21, 161);
			this->comboBoxControllerType->Name = L"comboBoxControllerType";
			this->comboBoxControllerType->Size = System::Drawing::Size(291, 24);
			this->comboBoxControllerType->TabIndex = 5;
			// 
			// buttonLoadSelectedSetting
			// 
			this->buttonLoadSelectedSetting->Location = System::Drawing::Point(475, 59);
			this->buttonLoadSelectedSetting->Name = L"buttonLoadSelectedSetting";
			this->buttonLoadSelectedSetting->Size = System::Drawing::Size(59, 24);
			this->buttonLoadSelectedSetting->TabIndex = 4;
			this->buttonLoadSelectedSetting->Text = L"Load";
			this->buttonLoadSelectedSetting->UseVisualStyleBackColor = true;
			this->buttonLoadSelectedSetting->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::buttonLoadSelectedSetting_Click);
			// 
			// comboBoxLoadSetting
			// 
			this->comboBoxLoadSetting->FormattingEnabled = true;
			this->comboBoxLoadSetting->Items->AddRange(gcnew cli::array< System::Object^  >(5) {L"My last settings", L"Default settings", 
				L"Settings for experiment", L"Settings for X,Y,Z,C,A tuning", L"Settings for machining"});
			this->comboBoxLoadSetting->Location = System::Drawing::Point(164, 59);
			this->comboBoxLoadSetting->Name = L"comboBoxLoadSetting";
			this->comboBoxLoadSetting->Size = System::Drawing::Size(291, 24);
			this->comboBoxLoadSetting->TabIndex = 3;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(18, 66);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(81, 16);
			this->label17->TabIndex = 2;
			this->label17->Text = L"Load setting";
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(18, 23);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(125, 16);
			this->label16->TabIndex = 1;
			this->label16->Text = L"When program start";
			// 
			// comboBoxProgramStartSetting
			// 
			this->comboBoxProgramStartSetting->FormattingEnabled = true;
			this->comboBoxProgramStartSetting->Items->AddRange(gcnew cli::array< System::Object^  >(5) {L"Show my last settings", L"Show default settings", 
				L"Show settings for experiment", L"Show settings for X,Y,Z,C,A tuning", L"Show settings for machining"});
			this->comboBoxProgramStartSetting->Location = System::Drawing::Point(164, 15);
			this->comboBoxProgramStartSetting->Name = L"comboBoxProgramStartSetting";
			this->comboBoxProgramStartSetting->Size = System::Drawing::Size(291, 24);
			this->comboBoxProgramStartSetting->TabIndex = 1;
			this->comboBoxProgramStartSetting->SelectedIndexChanged += gcnew System::EventHandler(this, &FiveAxisCNCForm::comboBoxProgramStartSetting_SelectedIndexChanged);
			// 
			// openFileDialogResultFile
			// 
			this->openFileDialogResultFile->FileName = L"openFileDialogResultFile";
			// 
			// contextMenuStripRecentGcode
			// 
			this->contextMenuStripRecentGcode->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->toolStripMenuItem1, 
				this->toolStripMenuItem2, this->toolStripMenuItem3});
			this->contextMenuStripRecentGcode->Name = L"contextMenuStripRecentGcode";
			this->contextMenuStripRecentGcode->Size = System::Drawing::Size(168, 70);
			// 
			// toolStripMenuItem1
			// 
			this->toolStripMenuItem1->Name = L"toolStripMenuItem1";
			this->toolStripMenuItem1->Size = System::Drawing::Size(167, 22);
			this->toolStripMenuItem1->Text = L"Gcode3Axis.tap";
			this->toolStripMenuItem1->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::toolStripMenuItem1_Click);
			// 
			// toolStripMenuItem2
			// 
			this->toolStripMenuItem2->Name = L"toolStripMenuItem2";
			this->toolStripMenuItem2->Size = System::Drawing::Size(167, 22);
			this->toolStripMenuItem2->Text = L"Parallelogram.tap";
			this->toolStripMenuItem2->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::toolStripMenuItem2_Click);
			// 
			// toolStripMenuItem3
			// 
			this->toolStripMenuItem3->Name = L"toolStripMenuItem3";
			this->toolStripMenuItem3->Size = System::Drawing::Size(167, 22);
			this->toolStripMenuItem3->Text = L"Rectangular.tap";
			this->toolStripMenuItem3->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::toolStripMenuItem3_Click);
			// 
			// contextMenuStripAnotherGraph
			// 
			this->contextMenuStripAnotherGraph->BackColor = System::Drawing::SystemColors::Control;
			this->contextMenuStripAnotherGraph->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {this->toolStripTextBoxDefaultGraph, 
				this->RelationU_Vel, this->ContourError, this->ApsoluteError, this->toolStripMenuItem4, this->toolStripComboBox1});
			this->contextMenuStripAnotherGraph->Name = L"contextMenuStripAnotherGraph";
			this->contextMenuStripAnotherGraph->ShowCheckMargin = true;
			this->contextMenuStripAnotherGraph->Size = System::Drawing::Size(204, 153);
			// 
			// toolStripTextBoxDefaultGraph
			// 
			this->toolStripTextBoxDefaultGraph->BackColor = System::Drawing::Color::Lime;
			this->toolStripTextBoxDefaultGraph->Name = L"toolStripTextBoxDefaultGraph";
			this->toolStripTextBoxDefaultGraph->ReadOnly = true;
			this->toolStripTextBoxDefaultGraph->Size = System::Drawing::Size(100, 23);
			this->toolStripTextBoxDefaultGraph->Text = L"Default Graph";
			this->toolStripTextBoxDefaultGraph->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::toolStripTextBoxDefaultGraph_Click);
			// 
			// RelationU_Vel
			// 
			this->RelationU_Vel->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(255)), 
				static_cast<System::Int32>(static_cast<System::Byte>(255)));
			this->RelationU_Vel->HideSelection = false;
			this->RelationU_Vel->Name = L"RelationU_Vel";
			this->RelationU_Vel->ReadOnly = true;
			this->RelationU_Vel->Size = System::Drawing::Size(100, 23);
			this->RelationU_Vel->Text = L"Voltage - Velocity: UAxis - RefX_1";
			this->RelationU_Vel->Click += gcnew System::EventHandler(this, &FiveAxisCNCForm::RelationU_Vel_Click);
			// 
			// ContourError
			// 
			this->ContourError->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(255)), 
				static_cast<System::Int32>(static_cast<System::Byte>(255)));
			this->ContourError->Name = L"ContourError";
			this->ContourError->ReadOnly = true;
			this->ContourError->Size = System::Drawing::Size(100, 23);
			this->ContourError->Text = L"Contour Error Elx, ElY1";
			// 
			// ApsoluteError
			// 
			this->ApsoluteError->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(255)), 
				static_cast<System::Int32>(static_cast<System::Byte>(255)));
			this->ApsoluteError->Name = L"ApsoluteError";
			this->ApsoluteError->ReadOnly = true;
			this->ApsoluteError->Size = System::Drawing::Size(100, 23);
			this->ApsoluteError->Text = L"Apsolute contour Erro Ec";
			// 
			// toolStripMenuItem4
			// 
			this->toolStripMenuItem4->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->toolStripTextBox1, 
				this->toolStripTextBox2, this->toolStripTextBox3});
			this->toolStripMenuItem4->Name = L"toolStripMenuItem4";
			this->toolStripMenuItem4->Size = System::Drawing::Size(203, 22);
			this->toolStripMenuItem4->Text = L"toolStripMenuItem4";
			// 
			// toolStripTextBox1
			// 
			this->toolStripTextBox1->Name = L"toolStripTextBox1";
			this->toolStripTextBox1->Size = System::Drawing::Size(100, 23);
			// 
			// toolStripTextBox2
			// 
			this->toolStripTextBox2->Name = L"toolStripTextBox2";
			this->toolStripTextBox2->Size = System::Drawing::Size(100, 23);
			// 
			// toolStripTextBox3
			// 
			this->toolStripTextBox3->Name = L"toolStripTextBox3";
			this->toolStripTextBox3->Size = System::Drawing::Size(100, 23);
			// 
			// toolStripComboBox1
			// 
			this->toolStripComboBox1->Items->AddRange(gcnew cli::array< System::Object^  >(3) {L"A", L"B", L"C"});
			this->toolStripComboBox1->Name = L"toolStripComboBox1";
			this->toolStripComboBox1->Size = System::Drawing::Size(121, 23);
			// 
			// timerFiveAxisForm
			// 
			this->timerFiveAxisForm->Enabled = true;
			this->timerFiveAxisForm->Tick += gcnew System::EventHandler(this, &FiveAxisCNCForm::timerFiveAxisForm_Tick);
			// 
			// FiveAxisCNCForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(984, 649);
			this->Controls->Add(this->FiveAxisCNCTabControl);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^  >(resources->GetObject(L"$this.Icon")));
			this->Name = L"FiveAxisCNCForm";
			this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
			this->Text = L"Robotmech Five axis CNC";
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &FiveAxisCNCForm::FiveAxisCNCForm_FormClosed);
			this->Load += gcnew System::EventHandler(this, &FiveAxisCNCForm::FiveAxisCNCForm_Load);
			this->FiveAxisCNCTabControl->ResumeLayout(false);
			this->tabPageMachining->ResumeLayout(false);
			this->tabPageMachining->PerformLayout();
			this->tabPageExperiment->ResumeLayout(false);
			this->tabPageExperiment->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBoxFiveAxisCNC))->EndInit();
			this->tabPageResultGraph->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->chartRealReferenceContour))->EndInit();
			this->Setting->ResumeLayout(false);
			this->Setting->PerformLayout();
			this->contextMenuStripRecentGcode->ResumeLayout(false);
			this->contextMenuStripAnotherGraph->ResumeLayout(false);
			this->contextMenuStripAnotherGraph->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion
#pragma region Windows Form function 
private: System::Void buttonSimulationStart_Click(System::Object^  sender, System::EventArgs^  e) {
			 if (NewGUIProcessing.bConnectStatus )
			 {
				 labelConnectStatus->Text = "Experimental mode running\n Cant run simulation \n Stop Connecting to CNC  \n and Try a again";
			 } 
			 else
			 {


			 // Five axis orientation
			 UpdateSettingParameters(); // Update sample time, Controller type

			 NewGUIProcessing.step = 0;
	//		 RmLabFiveAxisCNC.m_iSelectedFrictionModel = 1;
			 RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = NewGUIProcessing.SAMPLING_TIME;
			 RmLabFiveAxisCNC.m_fSampTimeRef =  NewGUIProcessing.SAMPLING_TIME;
			 RmLabFiveAxisCNC.OpenGcodeFile(GcodeProgramFolder+textBoxGcodeFilename->Text); 

			 RmLabFiveAxisCNC.OpenBinaryFile(textBoxDataFileName->Text);//textBoxDataFileName

			 //		  RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_X,3.0);
//			 RmLabFiveAxisCNC.InitGlobalVariable("GlobalVariable.txt");
			  
			 
			 // send Debug text 
//			 textBoxControlParameters->Text = "MX"
//				 + System::Convert::ToString(RmLabFiveAxisCNC.GetStaticVariable("A2BWa2"));//RmLabCNC::mt_M(0,0));//
			
//			 RmLabFiveAxisCNC.InitGlobalVariable(InitialConfigFolder+textBoxConfigFilename->Text);
			 RmLabFiveAxisCNC.InitControllerParameters(InitialConfigFolder+textBoxConfigFilename->Text);
			 // Init Current Gcode cmd and next Gcode cmd
			 RmLabFiveAxisCNC.GetNextGCodeLine(); 
			 RmLabFiveAxisCNC.GetNextGCodeLine(); 
			 // send Debug text 
			 
			 textBoxControlParameters->Text = "Real M X , Real Coulomb X, Real viscous X"
			       + RmLabFiveAxisCNC.DebugDataString();//RmLabCNC::mt_M(0,0));//


//			 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.GetCurrentReference());


// 			 NewGUIProcessing.step = NewGUIProcessing.step +1;
// 			 // 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
// 
// 			 RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
// 			 RmLabFiveAxisCNC.GetNextPointRefInGCodePath();// No meaning but it make out put voltage not is NaN
// 
// 			 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
// 			 textBoxControlParameters->Text = RmLabFiveAxisCNC.m_strDebugString;
// //			 RmLabFiveAxisCNC.SendOutputControl();
			 if (checkBoxSaveData->Checked){RmLabFiveAxisCNC.SaveDataToBinaryFile();};	 // Save start data 0  00  00 all

			 //		m_cTrajectoryControl.GetNextRef();
			 RealTimeWatch.Start();
			 m_iTimeStartTick = RealTimeWatch.GetTimestamp();
			  labelConnectStatus->Text = "Starting Simulation...\n";
		 while (!RmLabFiveAxisCNC.Finish())
			 {
//				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.GetCurrentReference());
				 NewGUIProcessing.step = NewGUIProcessing.step +1;
				 // 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;

				 RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
				 RmLabFiveAxisCNC.m_fexpTnowCounter = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
				 RmLabFiveAxisCNC.GetNextPointRefInGCodePath();  //*********************************Real Time********************//
	//			 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
				 RmLabFiveAxisCNC.FiveAxisMachineController();


				 if (checkBoxSaveData->Checked){RmLabFiveAxisCNC.SaveDataToBinaryFile();};	
				 //			 RmLabFiveAxisCNC.SendOutputControl();
				 RmLabFiveAxisCNC.SendOutputToVirtualSystem();
			 }
			 RmLabFiveAxisCNC.CloseBinaryFile();
			 RmLabFiveAxisCNC.CloseGcodeFile();
// 			 RmLabFiveAxisCNC.StopFiveAxisCNC();
//			 labelConnectStatus->Text = "Starting Simulation...\n";
			 m_fSecondRealTime = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;

			 labelConnectStatus->Text = labelConnectStatus->Text +"Finished Simulation\n"+
			 RmLabFiveAxisCNC.m_fexpTnowReal.ToString("f4")+"  "+ RmLabFiveAxisCNC.m_fexpRunT.ToString("f4");

			 labelTimeDisplay->Text = m_fSecondRealTime.ToString("F6");
			 }
		 }
private: System::Void buttonExperimentStartPause_Click(System::Object^  sender, System::EventArgs^  e) {
			 System::String^ ErrorRp;
			 // Experiment with out regulation
//			 RmLabFiveAxisCNC.m_bOriginSetup = true;
//			 if ((NewGUIProcessing.bConnectStatus )&&(RmLabFiveAxisCNC.m_bOriginSetup))
//			 {
//
////				 buttonExperimentStartPause->Text = "Pause Experiment";
//				 m_fTimeRun = System::Convert::ToDouble(textBoxTimeRun->Text);
//
//				 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
//				 NewGUIProcessing.maxTime = System::Convert::ToDouble(textBoxTimeRun->Text);
//				// NewGUIProcessing.SAMPLING_TIME = 0.05;// 100ms;
//				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxSampleTime->Text)/1000.0;// 100ms;
////				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxY1Encoder->Text);// 100ms;
//				 NewGUIProcessing.step = 0;
//
//				 RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = NewGUIProcessing.SAMPLING_TIME;
//				 RmLabFiveAxisCNC.m_fSampTimeRef =  NewGUIProcessing.SAMPLING_TIME;
//
//
//				 //		  RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_X,3.0);
//// 				 RmLabFiveAxisCNC.InitGlobalVariable("GlobalVariable.txt");
//// 				 RmLabFiveAxisCNC.OpenGcodeFile("GcodeSapmle1.tap");
//				 RmLabFiveAxisCNC.InitGlobalVariable( textBoxConfigFilename->Text);
//				 RmLabFiveAxisCNC.OpenGcodeFile(textBoxGcodeFilename->Text);  
//				 RmLabFiveAxisCNC.OpenBinaryFile("RmFiveAxisData.rme");
//				 RmLabFiveAxisCNC.SetMachineOrigin();
//				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());// Set real position to (0,0,0)
////				 RmLabFiveAxisCNC.GetNextPointRefInGCodePath(); 
////				 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
////				 RmLabFiveAxisCNC.SendOutputControl();
////				 RmLabFiveAxisCNC.SaveDataToBinaryFile();
//				 //		m_cTrajectoryControl.GetNextRef();
//
//
//				 RealTimeWatch.Start();
//				 m_iTimeStartTick = RealTimeWatch.GetTimestamp();
//
//				 NewGUIProcessing.step = NewGUIProcessing.step +1;
//				 // 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
//
//				// RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
//				 RmLabFiveAxisCNC.m_fexpTnowReal = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
//				 RmLabFiveAxisCNC.m_fexpTnowCounter = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
//
//
//				 RmLabFiveAxisCNC.GetNextPointRefInGCodePath();  //*********************************Real Time********************//
//
//				 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
//				 labelConnectStatus->Text = "Starting Experiment...\n";
//				 RmLabFiveAxisCNC.IOModule.StartCounter(pTimer,ErrorRp,this->hMainWnd);
//
//				 RmLabFiveAxisCNC.SendOutputControl();
//				 RmLabFiveAxisCNC.SaveDataToBinaryFile();
//
//			 }
			 // Experiment from regulation status
		//	 System::Object^  xsender;
			// System::EventArgs^  xe;

			 if ((NewGUIProcessing.bConnectStatus )&&(RmLabFiveAxisCNC.m_bOriginSetup))
			 {
				// buttonSetOrigin_Click(xsender,xe);
//				 RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);
				 RmLabFiveAxisCNC.OpenGcodeFile(GcodeProgramFolder+textBoxGcodeFilename->Text);  
				 RmLabFiveAxisCNC.OpenBinaryFile(textBoxDataFileName->Text);
				 // Init Current Gcode cmd and next Gcode cmd
				 


				 m_bCloseDataFile = false;
				 m_bExperimentFinish = false;
				 RmLabFiveAxisCNC.m_bGcodeFINISH = false;
				 RmLabFiveAxisCNC.m_bNextGcodeFINISH = false;
//				 buttonExperimentStartPause->Text = "Pause Experiment";
				 m_fTimeRun = System::Convert::ToDouble(textBoxTimeRun->Text);

//				 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
				 NewGUIProcessing.maxTime = m_fTimeRun;//System::Convert::ToDouble(textBoxTimeRun->Text);
				// NewGUIProcessing.SAMPLING_TIME = 0.05;// 100ms;
				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxSampleTime->Text)/1000.0;// 100ms;
//				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxY1Encoder->Text);// 100ms;
//				 NewGUIProcessing.step = 0;

				 RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = NewGUIProcessing.SAMPLING_TIME;
				 RmLabFiveAxisCNC.m_fSampTimeRef =  NewGUIProcessing.SAMPLING_TIME;


				 //		  RmLabFiveAxisCNC.IOModule.OutputOneMotor(RmLabFiveAxisCNC.LINEAR_MOTOR_X,3.0);
// 				 RmLabFiveAxisCNC.InitGlobalVariable("GlobalVariable.txt");
// 				 RmLabFiveAxisCNC.OpenGcodeFile("GcodeSapmle1.tap");
//				 RmLabFiveAxisCNC.InitGlobalVariable( textBoxConfigFilename->Text);
//				 RmLabFiveAxisCNC.InitControlVariable();
			//	 RmLabFiveAxisCNC.SetMachineOrigin();
//				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());// Set real position to (0,0,0)
//				 RmLabFiveAxisCNC.GetNextPointRefInGCodePath(); 
//				 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
//				 RmLabFiveAxisCNC.SendOutputControl();
//				 RmLabFiveAxisCNC.SaveDataToBinaryFile();
				 //		m_cTrajectoryControl.GetNextRef();


//				 RealTimeWatch.Start();
				 m_iTimeStartTick = RealTimeWatch.GetTimestamp();

//				 NewGUIProcessing.step = NewGUIProcessing.step +1;
				 // 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;

				// RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
				 

				 RmLabFiveAxisCNC.m_fexpTnowReal = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
	//			 RmLabFiveAxisCNC.m_fNextAccFirstTime = RmLabFiveAxisCNC.m_fexpTnowReal;
	//			 RmLabFiveAxisCNC.m_fexpRunT = RmLabFiveAxisCNC.m_fexpTnowReal;
	//			 RmLabFiveAxisCNC.m_fexpRunTPre = RmLabFiveAxisCNC.m_fexpTnowReal;
				 RmLabFiveAxisCNC.m_fexpTnow = 0.0;
//				 RmLabFiveAxisCNC.m_fexpTnowCounter = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
				 

				 


				 RmLabFiveAxisCNC.SetMachineOrigin();
				 RmLabFiveAxisCNC.IOModule.SetPartOrigin();
				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());
				 RmLabFiveAxisCNC.SetRefPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());
				 RmLabFiveAxisCNC.InitControllerParameters(InitialConfigFolder+textBoxConfigFilename->Text);
				 RmLabFiveAxisCNC.GetNextGCodeLine(); 
				 RmLabFiveAxisCNC.GetNextGCodeLine(); 

			//	 RmLabFiveAxisCNC.ResetReferenceData();
			//	 RmLabFiveAxisCNC.ResetRealData();

//				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());// Set real position to (0,0,0)
//				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());// Set real position to (0,0,0)
//				 RmLabFiveAxisCNC.SetRefPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());// Set real position to (0,0,0)
//				 RmLabFiveAxisCNC.GetNextPointRefInGCodePath();  //*********************************Real Time********************//

//				 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
				 labelConnectStatus->Text = "Starting Experiment...\n";
//				 RmLabFiveAxisCNC.IOModule.StartCounter(pTimer,ErrorRp,this->hMainWnd);

//				 RmLabFiveAxisCNC.SendOutputControl();
//				 RmLabFiveAxisCNC.SaveDataToBinaryFile();

			 }
			 else
			 {
				 labelConnectStatus->Text = "Not connect to CNC \n Or not Setup Origin \n Try a again \n and start experiment";
			 }
		 }
private: System::Void buttonStop_Click(System::Object^  sender, System::EventArgs^  e) {
			 System::String^ ErrorRp;
		RmLabFiveAxisCNC.IOModule.StopAllMotor();
		RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);

		RmLabFiveAxisCNC.CloseBinaryFile();
		RmLabFiveAxisCNC.CloseGcodeFile();
		labelConnectStatus->Text = " Stop Machine";
		labelTimeDisplay->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_fexpTnowReal);
		 }
private: System::Void buttonIOConnection_Click(System::Object^  sender, System::EventArgs^  e) {
			 System::String^ ErrorRp;

			 if (NewGUIProcessing.bConnectStatus )
			 {
				 //				 NewIOModule.DisconnectToCNC(ErrorRp) ;
				 RmLabFiveAxisCNC.IOModule.DisconnectToCNC(ErrorRp);
				 if (ErrorRp == "OK")
				 {
					 labelConnectStatus->Text = "Disconnected to CNC";
					 labelConnectStatus->ForeColor =  SystemColors::ControlText.Red;
					 this->pictureBoxFiveAxisCNC->ImageLocation = L"5AxisMachineMiniLocked.jpg";
					 buttonIOConnection->Text = "Connect to CNC";
					 buttonIOConnection->ForeColor = System::Drawing::Color::FromArgb(0, 192, 0);
					 NewGUIProcessing.bConnectStatus = !NewGUIProcessing.bConnectStatus;
				 }
				 else
				 {
					 labelConnectStatus->Text = "Error"+ErrorRp;
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
					 labelConnectStatus->Text = "Connecting to CNC";
					 this->pictureBoxFiveAxisCNC->ImageLocation = L"5AxisMachineMini.jpg";
					 buttonIOConnection->Text = "Disconnect to CNC";
					 buttonIOConnection->ForeColor = SystemColors::ControlText.Red;
					 NewGUIProcessing.bConnectStatus = !NewGUIProcessing.bConnectStatus;
				 } 
				 else
				 {
					 labelConnectStatus->Text = "Error"+ErrorRp;
				 }

			 } 		
		 }
private: System::Void buttonEmergencyStop_Click(System::Object^  sender, System::EventArgs^  e) {
			 System::String^ ErrorRp;
			 RmLabFiveAxisCNC.IOModule.StopAllMotor();
			 RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);

		RmLabFiveAxisCNC.CloseBinaryFile();
		RmLabFiveAxisCNC.CloseGcodeFile();
		labelConnectStatus->Text = labelConnectStatus->Text +"Emergency Stop";
		labelTimeDisplay->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_fexpTnowReal);


		 }
private: System::Void buttonLoadGcode_Click(System::Object^  sender, System::EventArgs^  e) {
			 openFileDialogResultFile->InitialDirectory = GcodeProgramFolder;
			 // Call the ShowDialog method to show the dialog box.
			 openFileDialogResultFile->ShowDialog();
			 // Process input if the user clicked OK.
			 if (openFileDialogResultFile->CheckFileExists)
			 {
				 // 				 FileTextToWrite = "RmFiveAxisData.txt";
				 // 				 FileBinaryData = openFileDialogResultFile->FileName;
				 textBoxGcodeFilename->Text = openFileDialogResultFile->SafeFileName;
			 }
			 textBoxControlParameters->Text = Application::StartupPath;
		 }
private: System::Void buttonRecentGcode_Click(System::Object^  sender, System::EventArgs^  e) {

//			 contextMenuStripRecentGcode->add
//			 this->Cursor = gcnew System::Windows::Forms::Cursor( ::Cursor::Current->Handle );
			 contextMenuStripRecentGcode->Show(this->Cursor->Position);
		 }
private: System::Void buttonEditGcodeFile_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void buttonLoadConfig_Click(System::Object^  sender, System::EventArgs^  e) {

			 openFileDialogResultFile->InitialDirectory = InitialConfigFolder;
			 // Call the ShowDialog method to show the dialog box.
			 openFileDialogResultFile->ShowDialog();
			 	 // Process input if the user clicked OK.
			 if (openFileDialogResultFile->CheckFileExists)
			 {
// 				 FileTextToWrite = "RmFiveAxisData.txt";
// 				 FileBinaryData = openFileDialogResultFile->FileName;
				 textBoxConfigFilename->Text = openFileDialogResultFile->SafeFileName;
				 // Open the selected file to read.
// 				 System::IO::Stream fileStream = openFileDialogResultFile->re  .File.OpenRead();
// 
// 				 using (System::IO::StreamReader reader = new System::IO::StreamReader(fileStream))
// 				 {
// 					 // Read the first line from the file and write it the textbox.
// 					 tbResults.Text = reader.ReadLine();
// 				 }
// 				 fileStream.Close();
			 }

		 }
private: System::Void buttonRecentConfig_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void buttonSetOrigin_Click(System::Object^  sender, System::EventArgs^  e) {
 System::String^ ErrorRp;
             UpdateSettingParameters(); // Update sample time, Controller type
			 if (NewGUIProcessing.bConnectStatus )
			 {
				 // Check when machine is regulating, need to stop counter
// 				 if (checkBoxPositionRegulation->Checked)
// 				 {
// 					 RmLabFiveAxisCNC.CloseBinaryFile();
// 					 RmLabFiveAxisCNC.CloseGcodeFile();
// 					 RmLabFiveAxisCNC.StopFiveAxisCNC();
// 					 RmLabFiveAxisCNC.IOModule.StopAllMotor();
// 					 RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);
// 				 }


				 RmLabFiveAxisCNC.IOModule.StopAllMotor();
				 RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);// Stop regulation 
				 // Close data file if it is opening
				 if (!m_bCloseDataFile)
				 {
					 m_bCloseDataFile = true;
					 m_bExperimentFinish = true;
					 RmLabFiveAxisCNC.CloseBinaryFile();
					 RmLabFiveAxisCNC.CloseGcodeFile();
				 };


				 //				 buttonExperimentStartPause->Text = "Pause Experiment";
				 m_fTimeRun = System::Convert::ToDouble(textBoxTimeRun->Text);

				 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);
				 NewGUIProcessing.maxTime = m_fTimeRun;
				 // NewGUIProcessing.SAMPLING_TIME = 0.05;// 100ms;
				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxSampleTime->Text)/1000.0;// 100ms;
				 //				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxY1Encoder->Text);// 100ms;
				 NewGUIProcessing.step = 0;

				 RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = NewGUIProcessing.SAMPLING_TIME;
				 RmLabFiveAxisCNC.m_fSampTimeRef =  NewGUIProcessing.SAMPLING_TIME;

//				 RmLabFiveAxisCNC.InitGlobalVariable(InitialConfigFolder+textBoxConfigFilename->Text);
				 


//				 RmLabFiveAxisCNC.OpenGcodeFile(textBoxGcodeFilename->Text);  

//				 RmLabFiveAxisCNC.OpenBinaryFile(textBoxDataFileName->Text);

				 RmLabFiveAxisCNC.SetMachineOrigin();
				 RmLabFiveAxisCNC.IOModule.SetPartOrigin();
				 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());  // Set real and reference position to (0,0,0,0,0)
				 RmLabFiveAxisCNC.SetRefPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());
				 RmLabFiveAxisCNC.InitControllerParameters(InitialConfigFolder+textBoxConfigFilename->Text);


				 RealTimeWatch.Start();
				 m_iTimeStartTick = RealTimeWatch.GetTimestamp();

				 NewGUIProcessing.step = NewGUIProcessing.step +1;
				 // 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;

				 // RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
				 RmLabFiveAxisCNC.m_fexpTnowReal = 0.0;//;(double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
				 RmLabFiveAxisCNC.m_fexpTnowCounter = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
//				 RmLabFiveAxisCNC.InitControlVariable();
		//		 RmLabFiveAxisCNC.m_fNextAccFirstTime = RmLabFiveAxisCNC.m_fexpTnowReal;
				 RmLabFiveAxisCNC.GetNextPointRefInRegulation(); 
//				 RmLabFiveAxisCNC.ThreeAxisMachineController();
				 RmLabFiveAxisCNC.FiveAxisMachineControllerInRegulation();
//				 RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
				 labelConnectStatus->Text = "Position regulating...\n";
				 checkBoxPositionRegulation->Checked = true; // Auto regulating
				 RmLabFiveAxisCNC.m_bGcodeFINISH = true;
				 RmLabFiveAxisCNC.IOModule.StartCounter(pTimer,ErrorRp,this->hMainWnd);
				 RmLabFiveAxisCNC.SendOutputControl();
//				 if (checkBoxSaveData->Checked){RmLabFiveAxisCNC.SaveDataToBinaryFile();};


			 }
			 else
			 {
				 labelConnectStatus->Text = "Not connect to CNC\n Simulation moving";
				 if (!m_bCloseDataFile)
				 {
					 m_bCloseDataFile = true;
					 m_bExperimentFinish = true;
					 RmLabFiveAxisCNC.CloseBinaryFile();
					 RmLabFiveAxisCNC.CloseGcodeFile();
				 };
				 NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxSampleTime->Text)/1000.0;// 100ms;
				 NewGUIProcessing.step = 0;
				 RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = NewGUIProcessing.SAMPLING_TIME;
				 RmLabFiveAxisCNC.m_fSampTimeRef =  NewGUIProcessing.SAMPLING_TIME;

				 RmLabFiveAxisCNC.InitControllerParameters(InitialConfigFolder+textBoxConfigFilename->Text);

// 				 RmLabFiveAxisCNC.SetRealPosition(vector_machine_pos_init);  // Set real and reference position to (0,0,0,0,0)
// 				 RmLabFiveAxisCNC.SetRefPosition(vector_machine_pos_init);
				
			 }
			 
		 }
private: System::Void buttonTestCounter_Click(System::Object^  sender, System::EventArgs^  e) {
		 System::String^ ErrorRp;
			RmLabFiveAxisCNC.IOModule.SAMPLING_TIME = System::Convert::ToDouble(textBoxSampleTime->Text);// 100ms;
			 pTimer	= Marshal::GetFunctionPointerForDelegate(dele_Timer);

			 RmLabFiveAxisCNC.IOModule.StartCounter(pTimer,ErrorRp,this->hMainWnd);
			 RmLabFiveAxisCNC.SetMachineOrigin();
		 }
private: System::Void buttonUpdatePosition_Click(System::Object^  sender, System::EventArgs^  e) {
	//		 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());
			 UpdateRealPosition(); 			 
		 }
private: System::Void FiveAxisCNCForm_Load(System::Object^  sender, System::EventArgs^  e) {
			 //-----------------------------------
			 // No need
			 //-----------------------------------
			 hMainWnd	= this->Handle.ToInt32();
			 //-----------------------------------
			 // Connect TimerCallBackProc to PTIMERCALLBACK of CCntCLI
			 //-----------------------------------

			 dele_Timer	= gcnew CCntCLI::PTIMERCALLBACK(this, &FiveAxisCNCForm::TimerCallBackProc);
			 //-----------------------------------
			 // ガベージコレクションにより破棄されないようにデリゲートへの参照を追加
			 //-----------------------------------
			 hGC_Timer	= GCHandle::Alloc(dele_Timer);
			 NewGUIProcessing.bConnectStatus = 0;
			 NewGUIProcessing.time = 0.0;
			 NewGUIProcessing.maxTime = 3.0;
			 NewGUIProcessing.step = 0;

			 m_iFrequency = Stopwatch::Frequency;
			 textBoxSampleTime->Text = "2";
			 InitialConfigFolder = Application::StartupPath + "\\InitialConfig\\";
			 GcodeProgramFolder  = Application::StartupPath + "\\GcodeProgram\\";
			 m_sConfigFilename = "GlobalVariable.txt";
			 m_sGcodeFilename = "TUTGCode.tap";
			 textBoxConfigFilename->Text =   m_sConfigFilename;  
			 //      TUTGCode.tap    Gcode3Axis.tap
			 textBoxGcodeFilename->Text=   m_sGcodeFilename;//"Gcode3Axis.tap \\GcodeSapmle1.tap"; //TUTGCodeEx.tap
			 textBoxDataFileName->Text =  "RmFiveAxisData.rme";
			 checkBoxSaveData->Checked = false;
			 checkBoxLimitTime->Checked = true;
			 radioButtonManualSpeedX1->Checked = true;
			 comboBoxProgramStartSetting->SelectedIndex = 0;
			 comboBoxLoadSetting->SelectedIndex = 0;
			 InitStaticVariable();
			 UpdateRealPosition();
			 FiveAxisCNCFormSettingInit();
		 }
private: System::Void FiveAxisCNCForm_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) {
			 System::String^ ErrorRp;
			 RmLabFiveAxisCNC.IOModule.DisconnectToCNC(ErrorRp);
			 //-----------------------------------
			 // ガベージコレクションにより破棄されないようにデリゲートへの参照を追加
			 //-----------------------------------

			 hGC_Timer	= GCHandle::Alloc(dele_Timer);
			 FiveAxisCNCFormSettingSave();

		 }
#pragma endregion

#pragma region User define variable
public:
	GCHandle					hGC_Timer;
	CCntCLI::PTIMERCALLBACK^	dele_Timer;
	IntPtr						pTimer;
	int						hMainWnd;
	double           m_fTimeRun, m_fSecondRealTime;
	Stopwatch RealTimeWatch;
	Int64 m_iTimeStartTick, m_iTimeNowTick;
	Int64 m_iFrequency; 
	System::String^ FileBinaryData;
	System::String^ FileBinaryDataMoveToMatlabDir;
	System::String^ FileTextToWrite;
	System::String^ InitialConfigFolder;
	System::String^ GcodeProgramFolder;
	System::String^ FileConfig;
	System::String^ m_sConfigFilename;
	System::String^ m_sGcodeFilename;
	bool bbuttonXPlusMouseDown, bbuttonXMinusMouseDown,
		 bbuttonY1PlusMouseDown, bbuttonY1MinusMouseDown, 
		 bbuttonZPlusMouseDown, bbuttonZMinusMouseDown,
		 bbuttonCPlusMouseDown, bbuttonCMinusMouseDown,
		 bbuttonA1PlusMouseDown, bbuttonA1MinusMouseDown,

		 bbuttonWorkpieceXPlusMouseDown, bbuttonWorkpieceXMinusMouseDown,
		 bbuttonWorkpieceYPlusMouseDown, bbuttonWorkpieceYMinusMouseDown, 
		 bbuttonWorkpieceZPlusMouseDown, bbuttonWorkpieceZMinusMouseDown,
		 bbuttonWorkpieceCPlusMouseDown, bbuttonWorkpieceCMinusMouseDown,
		 bbuttonWorkpieceAPlusMouseDown, bbuttonWorkpieceAMinusMouseDown,

		 m_bCloseDataFile, m_bExperimentFinish;
	double           m_fManualSpeed;
private:
	/// <summary>
	///User define variable.
	GUIMainProcess::GUIProcessing NewGUIProcessing;
	RmLabCNC::FiveAxisCNC  RmLabFiveAxisCNC;
	/// </summary>

#pragma endregion

#pragma region User define fucntion
void InitStaticVariable();
void UpdateRealPosition(); 
void FiveAxisCNCFormSettingSave();
void FiveAxisCNCFormSettingInit();
void UpdateSettingParameters(); 
void FiveAxisCNCFormSettingLoad(short m_iIndex); 
void InitSettingStringScan(System::String^ scanString,System::String^ &strName,System::String^ &strValue);
void SettingFormVariable(System::String^ strName,System::String^ strValue);
//void TimerCallBackProc(short m_Id, int wParam, int lParam, void * Param);

void TimerCallBackProc(short m_Id, int wParam, int lParam, void * Param){
	System::String^ ErrorRp;

// 	NewGUIProcessing.time = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
// 	labelConnectStatus->Text = String::Format("Time = {0:C2}",NewGUIProcessing.time);
// 	if (NewGUIProcessing.time >NewGUIProcessing.maxTime)
// 	{
// 		RmLabFiveAxisCNC.IOModule.StopAllMotor();
// 		RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);
// 		NewGUIProcessing.step = 0;
// 	}
// 	else
// 	{
// 		NewGUIProcessing.step = NewGUIProcessing.step +1;
// 	}



	// Emergency stop check
	if ((RmLabFiveAxisCNC.m_fexpTnowReal> m_fTimeRun)&&(m_fTimeRun >DOUBLE_TOLERANCE)&&(checkBoxLimitTime->Checked))
	{
		RmLabFiveAxisCNC.IOModule.StopAllMotor();
		RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);

		RmLabFiveAxisCNC.CloseBinaryFile();
		RmLabFiveAxisCNC.CloseGcodeFile();
//		labelConnectStatus->Text = labelConnectStatus->Text +" Out of Fixed time run";
//		labelTimeDisplay->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_fexpTnowReal);
	}else
	{
			 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());
	if (!RmLabFiveAxisCNC.Finish())
	{
 		NewGUIProcessing.step = NewGUIProcessing.step +1;
// 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
		RmLabFiveAxisCNC.m_fexpTnowReal = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
		RmLabFiveAxisCNC.m_fexpTnowCounter = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
//		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
//		RmLabFiveAxisCNC.m_fexpTnowCounter = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
		 RmLabFiveAxisCNC.GetNextPointRefInGCodePath(); 
		//m_cTrajectoryControl.GetNextRealRef(); 
//		RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
		RmLabFiveAxisCNC.FiveAxisMachineController();
		RmLabFiveAxisCNC.SendOutputControl();
	//	RmLabFiveAxisCNC.SendOutputToVirtualSystem();
		if (checkBoxSaveData->Checked){RmLabFiveAxisCNC.SaveDataToBinaryFile();};	

	} 
	else
	{
		if (!m_bCloseDataFile)
		{
			m_bCloseDataFile = true;
			m_bExperimentFinish = true;
			RmLabFiveAxisCNC.CloseBinaryFile();
			RmLabFiveAxisCNC.CloseGcodeFile();
		};
		if (checkBoxPositionRegulation->Checked)  // Position Regulation controller
		{
		
		RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());
//		RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());
		NewGUIProcessing.step = NewGUIProcessing.step +1;
		RmLabFiveAxisCNC.m_fexpTnowReal = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
		RmLabFiveAxisCNC.m_fexpTnowCounter = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;

		RmLabFiveAxisCNC.GetNextPointRefInRegulation(); 
//		RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
//		RmLabFiveAxisCNC.ThreeAxisMachineController();
		RmLabFiveAxisCNC.FiveAxisMachineControllerInRegulation();
//		vec_OutputControl(0) = vec_OutputControl(0);
//		textBoxXPosition->Text = System::Convert::ToString(vec_OutputControl(0));
//		textBoxY1Position->Text = System::Convert::ToString(vec_OutputControl(1));
		RmLabFiveAxisCNC.SendOutputControl();
//		RmLabFiveAxisCNC.SaveDataToBinaryFile();
//		 if (checkBoxSaveData->Checked) {RmLabFiveAxisCNC.SaveDataToBinaryFile();};

		} 
		else
		{
			/*
			// Fix CNC position not change
			//		NewGUIProcessing.step = NewGUIProcessing.step +1;
			// 		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
			RmLabFiveAxisCNC.m_fexpTnowReal = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
			//		RmLabFiveAxisCNC.m_fexpTnowCounter = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
			//		RmLabFiveAxisCNC.m_fexpTnowReal = NewGUIProcessing.step * NewGUIProcessing.SAMPLING_TIME;
			//		RmLabFiveAxisCNC.m_fexpTnowCounter = (double)(RealTimeWatch.GetTimestamp()-m_iTimeStartTick)/(double) m_iFrequency;
			//		RmLabFiveAxisCNC.GetNextPointRefInGCodePath(); 
			//m_cTrajectoryControl.GetNextRealRef(); 
			RmLabFiveAxisCNC.IndependentControl3DFiveAxis();
			RmLabFiveAxisCNC.SendOutputControl();
			//		RmLabFiveAxisCNC.SaveDataToBinaryFile();
			*/
			RmLabFiveAxisCNC.CloseBinaryFile();
			RmLabFiveAxisCNC.CloseGcodeFile();
			RmLabFiveAxisCNC.StopFiveAxisCNC();
			RmLabFiveAxisCNC.IOModule.StopAllMotor();
			RmLabFiveAxisCNC.IOModule.StopCounter(ErrorRp);
			// 		labelConnectStatus->Text = labelConnectStatus->Text +"Finished Experiment";
			//		labelTimeDisplay->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_fexpTnowReal);
		}

	}//end if (!RmLabFiveAxisCNC.Finish())
	}/// end else Emergency stop check

/*	RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetAbsolutePosition());
	labelTimeDisplay->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.X);
//			textBoxXPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.X);
			 textBoxY1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.Y1);
			 textBoxY2Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.Y2);
			 textBoxZPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.Z);
			 textBoxCPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.A);
			 textBoxA1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.C1);
			 textBoxA2Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.C2);

			 */
		return;
	}


void ShowGraph();
void ClearGraph();
void SaveCurrentConfig();
void InitConfigLastTime();
#pragma endregion

private: System::Void buttonShowGraph_Click(System::Object^  sender, System::EventArgs^  e) {
//			 System::String^ FileBinaryData;
//			 System::String^ FileTextToWrite;

			 FileTextToWrite = "RmFiveAxisData.txt";
			 FileBinaryData = textBoxDataFileName->Text;
			 FileBinaryDataMoveToMatlabDir ="E:\\BUIDINHBA\\Google Drive\\ToyohashiStudy\\MATLAB\\MachineTool\\3AxisIndividualTuningD2_305\\ParametersTuning\\"+FileBinaryData;
          //   FileBinaryDataMoveToMatlabDir ="SLMC_3D_R20F20T1.rme";
			 ClearGraph() ;
			 ShowGraph();

		 }
private: System::Void buttonClearGraph_Click(System::Object^  sender, System::EventArgs^  e) {
			 ClearGraph() ;
		 }
private: System::Void buttonZoomOut_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea3"]->AxisX->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea3"]->AxisY->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisX->ScaleView->ZoomReset(0);
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->ScaleView->ZoomReset(0);
		 }


private: System::Void buttonOpenResultFile_Click(System::Object^  sender, System::EventArgs^  e) {
			 openFileDialogResultFile->ShowDialog();

			 if (openFileDialogResultFile->CheckFileExists)
			 {
				 FileTextToWrite = "RmFiveAxisData.txt";
				 FileBinaryData = openFileDialogResultFile->FileName;
				 labelTimeDisplay->Text = openFileDialogResultFile->FileName;
				 ShowGraph();
			 }
		 }
private: System::Void toolStripMenuItem1_Click(System::Object^  sender, System::EventArgs^  e) {
		//	 File
			textBoxGcodeFilename->Text = GcodeProgramFolder+ toolStripMenuItem1->Text;

		 }
private: System::Void toolStripMenuItem2_Click(System::Object^  sender, System::EventArgs^  e) {
			 textBoxGcodeFilename->Text = GcodeProgramFolder + toolStripMenuItem2->Text;
		 }
private: System::Void toolStripMenuItem3_Click(System::Object^  sender, System::EventArgs^  e) {
			 textBoxGcodeFilename->Text = GcodeProgramFolder + toolStripMenuItem3->Text;
		 }
private: System::Void buttonAnotherGraph_Click(System::Object^  sender, System::EventArgs^  e) {

          contextMenuStripAnotherGraph->Show(this->Cursor->Position);
		 }
private: System::Void RelationU_Vel_Click(System::Object^  sender, System::EventArgs^  e) {
			 
			 this->chartRealReferenceContour->Series["XControlVoltage"]->Enabled = false;
			 this->chartRealReferenceContour->Series["YControlVoltage"]->Enabled = false;
			 this->chartRealReferenceContour->Series["XPredictedVoltage"]->Enabled = false;
			 this->chartRealReferenceContour->Series["YPredictedVoltage"]->Enabled = false;

			 this->chartRealReferenceContour->Series["RefX_1_UX"]->Enabled = true;
			 this->chartRealReferenceContour->Series["RefY_1_UY"]->Enabled = true;

			 //Reset range Axis X and Y
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->Maximum = 6;
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->Minimum = -6;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->Maximum = 6;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->Minimum = -6;

			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Maximum = 7;
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Minimum = -7;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisX->Maximum = 7;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisX->Minimum = -7;
		 }
private: System::Void toolStripTextBoxDefaultGraph_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->chartRealReferenceContour->Series["XControlVoltage"]->Enabled = true;
			 this->chartRealReferenceContour->Series["YControlVoltage"]->Enabled = true;
			 this->chartRealReferenceContour->Series["XPredictedVoltage"]->Enabled = true;
			 this->chartRealReferenceContour->Series["YPredictedVoltage"]->Enabled = true;

			 this->chartRealReferenceContour->Series["RefX_1_UX"]->Enabled = false;
			 this->chartRealReferenceContour->Series["RefY_1_UY"]->Enabled = false;

			 //Reset range Axis X and Y
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Maximum = 20;
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Minimum = 0;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisX->Maximum = 20;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisX->Minimum = 0;

			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->Maximum = 36;
			 //	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Minimum = -36;
			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->Minimum = -36;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->Maximum = 36;
			 this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->Minimum = -36;
		 }
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void timerFiveAxisForm_Tick(System::Object^  sender, System::EventArgs^  e) {
			 
			 if ((RmLabFiveAxisCNC.m_fexpTnowReal> m_fTimeRun)&&(m_fTimeRun >DOUBLE_TOLERANCE)&&(checkBoxLimitTime->Checked))
			 {		
				labelConnectStatus->Text = "Out of Fixed time run";
				labelTimeDisplay->Text = RmLabFiveAxisCNC.m_fexpTnowReal.ToString("f4"); //System::Convert::ToString(RmLabFiveAxisCNC.m_fexpTnowReal);
			 }
			  RmLabFiveAxisCNC.m_CNCPosManualStep.X = 0.0;
			  RmLabFiveAxisCNC.m_CNCPosManualStep.Y = 0.0;
			  RmLabFiveAxisCNC.m_CNCPosManualStep.Z = 0.0;
			  RmLabFiveAxisCNC.m_CNCPosManualStep.C = 0.0;
			  RmLabFiveAxisCNC.m_CNCPosManualStep.A = 0.0;
			if (m_bExperimentFinish)
			 {
				 if (!NewGUIProcessing.bConnectStatus) {labelConnectStatus->Text = "Regulating in Simulation";}
				 else
					 {labelConnectStatus->Text = "Experiment Finished...\nRegulating";}
				 m_bExperimentFinish = false;
			}
			 if (bbuttonXPlusMouseDown){RmLabFiveAxisCNC.m_CNCPosManualStep.X = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME; }
			 if (bbuttonXMinusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.X = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
//				 bbuttonXMinusMouseDown = false;
			 }
			 if (bbuttonY1PlusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.Y = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
				 
			 }
			 if (bbuttonY1MinusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.Y = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
//				 textBoxY1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Y);
			 }
			 if (bbuttonZPlusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.Z = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
//				 textBoxZPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Z);
			 }
			 if (bbuttonZMinusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.Z = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
//				 textBoxZPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Z);
			 }
			 if (bbuttonCPlusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.C = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;
//				 textBoxCPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.C);
			 }
			 if (bbuttonCMinusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.C = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;
//				 textBoxCPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.C);
			 }
			 if (bbuttonA1PlusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.A = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;
//				 textBoxA1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.A);
			 }
			 if (bbuttonA1MinusMouseDown)
			 {
				 RmLabFiveAxisCNC.m_CNCPosManualStep.A = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;
//				 textBoxA1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.A);
			 }

			 RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.X = 0.0;
			 RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.Y = 0.0;
			 RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.Z = 0.0;
			 RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.C = 0.0;
			 RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.A = 0.0;
			 RmLabFiveAxisCNC.m_iCurrentCoordinate = MACHINECOORDINATE;

			 if (bbuttonWorkpieceXPlusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.X = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME; RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE; }
			 if (bbuttonWorkpieceYPlusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.Y = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME; RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE; }
			 if (bbuttonWorkpieceZPlusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.Z = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}
			 if (bbuttonWorkpieceCPlusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.C = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}
			 if (bbuttonWorkpieceAPlusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.A = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}

			 if (bbuttonWorkpieceXMinusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.X = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}
			 if (bbuttonWorkpieceYMinusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.Y = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}
			 if (bbuttonWorkpieceZMinusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.Z = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}
			 if (bbuttonWorkpieceCMinusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.C = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}
			 if (bbuttonWorkpieceAMinusMouseDown){ RmLabFiveAxisCNC.m_CNCWorkpiecePosStep.A = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME/PITODEGREE;  RmLabFiveAxisCNC.m_iCurrentCoordinate = WORKPIECECOORDINATE;}

			 if (radioButtonManualSpeedX1->Checked)
			 {
				 labelManualSpeed->Text = "Manual Speed: 0.01 mm/s, deg/s";
				 m_fManualSpeed = 0.01;
			 }
			 if (radioButtonManualSpeedX10->Checked)
			 {
				 labelManualSpeed->Text = "Manual Speed: 0.1 mm/s, deg/s";
				 m_fManualSpeed = 0.1;
			 }
			 if (radioButtonManualSpeedX100->Checked)
			 {
				 labelManualSpeed->Text = "Manual Speed: 1 mm/s, deg/s";
				 m_fManualSpeed = 1.0;
			 }
			 if (radioButtonManualSpeedX1000->Checked)
			 {
				 labelManualSpeed->Text = "Manual Speed: 5 mm/s, deg/s";
				 m_fManualSpeed = 3.0;
			 }
			 if (checkBoxPositionUpdate->Checked)
			 {
				 if (!NewGUIProcessing.bConnectStatus )
				 {
					 RmLabFiveAxisCNC.GetNextPointRefInRegulation();
					 textBoxXPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.X);
					 textBoxY1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.Y);
					 textBoxZPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.Z);
					 textBoxCPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.C*PITODEGREE);
					 textBoxA1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.A*PITODEGREE);
					 textBoxA2Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRefPos.A2*PITODEGREE );

					 textBoxWorkpieceX->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRefPos.X);
					 textBoxWorkpieceY->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRefPos.Y);
					 textBoxWorkpieceZ->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRefPos.Z);
					 textBoxWorkpieceC->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRefPos.C*PITODEGREE);
					 textBoxWorkpieceA->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRefPos.A*PITODEGREE);
				 } else
				 {

					 textBoxXPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.X);
					 textBoxY1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Y);
					 textBoxZPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Z);
					 textBoxCPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.C*PITODEGREE);
					 textBoxA1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.A*PITODEGREE);
					 textBoxA2Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.A2*PITODEGREE );

					 textBoxWorkpieceX->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRealPos.X);
					 textBoxWorkpieceY->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRealPos.Y);
					 textBoxWorkpieceZ->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRealPos.Z);
					 textBoxWorkpieceC->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRealPos.C*PITODEGREE);
					 textBoxWorkpieceA->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCWorkpieceRealPos.A*PITODEGREE);
				 }
	//			 RmLabFiveAxisCNC.SetRealPosition(RmLabFiveAxisCNC.IOModule.GetMachiningPosition());

			 }
		 }
private: System::Void buttonXPlus_MouseEnter(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void buttonXPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonXPlusMouseDown = true;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.X = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
		 }
private: System::Void buttonXPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 
			// bbuttonXPlusMouseDown = false;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.X = 0.0;
		 }
private: System::Void buttonXMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonXMinusMouseDown = true;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.X = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
		 }
private: System::Void buttonXMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
//			 bbuttonXMinusMouseDown = false;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.X = 0.0;
		 }
private: System::Void buttonY1Plus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonY1PlusMouseDown = true;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.Y1 = m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
		 }
private: System::Void buttonY1Plus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonY1PlusMouseDown = false;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.Y1 = 0.0;
		 }
private: System::Void buttonY1Minus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonY1MinusMouseDown = true;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.Y1 = -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
		 }
private: System::Void buttonY1Minus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonY1MinusMouseDown = false;
			  //RmLabFiveAxisCNC.m_CNCPosManualStep.Y1 = 0.0;
		 }
private: System::Void buttonZPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonZPlusMouseDown = true;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.Z =  m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
		 }
private: System::Void buttonZPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonZPlusMouseDown = false;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.Z = 0.0;
		 }
private: System::Void buttonZMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonZMinusMouseDown = true;
			 //RmLabFiveAxisCNC.m_CNCPosManualStep.Z =  -m_fManualSpeed*NewGUIProcessing.SAMPLING_TIME;
		 }
private: System::Void buttonZMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonZMinusMouseDown = false;
			  //RmLabFiveAxisCNC.m_CNCPosManualStep.Z = 0.0;
		 }
private: System::Void checkBoxPositionRegulation_CheckStateChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void buttonCPlus_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void buttonCPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonCPlusMouseDown = true;
		 }
private: System::Void buttonCPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonCPlusMouseDown = false;
		 }
private: System::Void buttonCMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonCMinusMouseDown = true;
		 }
private: System::Void buttonCMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonCMinusMouseDown = false;
		 }
private: System::Void buttonA1Plus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonA1PlusMouseDown = true;
		 }
private: System::Void buttonA1Plus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonA1PlusMouseDown = false;
		 }
private: System::Void buttonA1Minus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonA1MinusMouseDown = true;
		 }
private: System::Void buttonA1Minus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonA1MinusMouseDown = false;
		 }
private: System::Void comboBoxProgramStartSetting_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
//			 comboBoxProgramStartSetting->TabIndex;
//          label16->Text = comboBoxProgramStartSetting->SelectedItem->ToString();
		 }
private: System::Void buttonLoadSelectedSetting_Click(System::Object^  sender, System::EventArgs^  e) {

			 FiveAxisCNCFormSettingLoad(comboBoxLoadSetting->SelectedIndex); 
		 }
private: System::Void checkBoxSpinSetting_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 if (NewGUIProcessing.bConnectStatus)
			 {
				 if (checkBoxSpinSetting->Checked)
				 {
					 RmLabFiveAxisCNC.SpinStart(System::Convert::ToDouble(textBoxSpinSpeed->Text));
				 } 
				 else
				 {
					 RmLabFiveAxisCNC.SpinStop();
				 }
			 }
		 }
private: System::Void checkBoxPositionUpdate_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		 }
private: System::Void buttonXPlus_MouseLeave(System::Object^  sender, System::EventArgs^  e) {
			 bbuttonXPlusMouseDown = false;
		 }
private: System::Void buttonXMinus_MouseLeave(System::Object^  sender, System::EventArgs^  e) {
			 bbuttonXMinusMouseDown = false;
		 }
private: System::Void buttonWorkpieceXMinus_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void buttonWorkpieceXMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
          bbuttonWorkpieceXMinusMouseDown =  true;

		 }
private: System::Void buttonWorkpieceXMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonWorkpieceXMinusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceXPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceXPlusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceXPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceXPlusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceYMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceYMinusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceYMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceYMinusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceYPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceYPlusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceYPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonWorkpieceYPlusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceZMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceZMinusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceZMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceZMinusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceZPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceZPlusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceZPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceZPlusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceCMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceCMinusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceCMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceCMinusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceCPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceCPlusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceCPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceCPlusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceAMinus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceAMinusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceAMinus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			  bbuttonWorkpieceAMinusMouseDown =  false;
		 }
private: System::Void buttonWorkpieceAPlus_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceAPlusMouseDown =  true;
		 }
private: System::Void buttonWorkpieceAPlus_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 bbuttonWorkpieceAPlusMouseDown =  false;
			
		 }
};
}

