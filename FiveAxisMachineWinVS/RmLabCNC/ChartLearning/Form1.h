#pragma once

namespace ChartLearning {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;

	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
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
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart1;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::Button^  button2;
	protected: 

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->chart1))->BeginInit();
			this->SuspendLayout();
			// 
			// chart1
			// 
			chartArea1->AxisX->LabelStyle->Format = L"F2";
			chartArea1->AxisY->LabelStyle->Format = L"F2";
			chartArea1->AxisY->ScrollBar->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(0)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(192)));
			chartArea1->AxisY->ScrollBar->ButtonColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(0)), static_cast<System::Int32>(static_cast<System::Byte>(0)));
			chartArea1->AxisY->ScrollBar->LineColor = System::Drawing::Color::Navy;
			chartArea1->CursorX->IsUserEnabled = true;
			chartArea1->CursorX->IsUserSelectionEnabled = true;
			chartArea1->CursorY->IsUserEnabled = true;
			chartArea1->CursorY->IsUserSelectionEnabled = true;
			chartArea1->InnerPlotPosition->Auto = false;
			chartArea1->InnerPlotPosition->Height = 80;
			chartArea1->InnerPlotPosition->Width = 80;
			chartArea1->InnerPlotPosition->X = 10;
			chartArea1->InnerPlotPosition->Y = 10;
			chartArea1->Name = L"ChartArea1";
			this->chart1->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->chart1->Legends->Add(legend1);
			this->chart1->Location = System::Drawing::Point(12, 12);
			this->chart1->Name = L"chart1";
			series1->ChartArea = L"ChartArea1";
			series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series1->Legend = L"Legend1";
			series1->Name = L"Series1";
			this->chart1->Series->Add(series1);
			this->chart1->Size = System::Drawing::Size(616, 326);
			this->chart1->TabIndex = 0;
			this->chart1->Text = L"chart1";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(39, 371);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(246, 42);
			this->button1->TabIndex = 1;
			this->button1->Text = L"button1";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(361, 370);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(159, 42);
			this->button2->TabIndex = 2;
			this->button2->Text = L"Zoom Out";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(640, 435);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->chart1);
			this->Name = L"Form1";
			this->Text = L"Form1";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->chart1))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
				 System::String^ FileBinaryData;
				 System::String^ FileTextToWrite;

				 double m_fDataReadTime,m_fDataReadRefX,
					 m_fDataReadRefY,m_fDataReadRealX,m_fDataReadRealY;
				 double  realPositionX,realPositionY;
				 FileTextToWrite = "RmFiveAxisData.txt";
				 FileBinaryData = "RmFiveAxisData.rme";
				 try
				 {
					 FileStream^ BinaryDataFileStream = gcnew FileStream(FileBinaryData, FileMode::Open);
					 BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);
					 StreamWriter^ StreamTextWriter = gcnew StreamWriter(FileTextToWrite);
					 //		Console::WriteLine("contents of {0}:", fileName);
					 StreamTextWriter->WriteLine("Time       RefX       RefY       RealX     RealY");
					 while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
					 {
						 m_fDataReadTime  = BinaryDataReader->ReadDouble();
						 m_fDataReadRefX  = BinaryDataReader->ReadDouble();
						 m_fDataReadRefY  = BinaryDataReader->ReadDouble();
						 m_fDataReadRealX  = BinaryDataReader->ReadDouble();
						 m_fDataReadRealY  = BinaryDataReader->ReadDouble();
						 // 					 realPositionX  = BinaryDataReader->ReadDouble();
						 // 					 realPositionY  = BinaryDataReader->ReadDouble();

						 this->chart1->Series["Series1"]->Points->AddXY(m_fDataReadRefX,m_fDataReadRefY);
// 						 this->chartRealReferenceContour->Series["XYVoltage"]->Points->AddXY(m_fDataReadRefX,m_fDataReadRefY);
// 						 this->chartRealReferenceContour->Series["ewError"]->Points->AddXY(m_fDataReadRefX,m_fDataReadRefY);
// 						 this->chartRealReferenceContour->Series["XYRefVoltage"]->Points->AddXY(m_fDataReadRefX,m_fDataReadRefY);
						 StreamTextWriter->WriteLine(m_fDataReadTime.ToString("f3")
							 +"    "+m_fDataReadRefX.ToString("F4")+    "     "+m_fDataReadRefY.ToString("f4")
							 +"    "+m_fDataReadRealX.ToString("f4")+   "     "+m_fDataReadRealY.ToString("f4"));
						 //	 +"    "+m_fDataReadRealX.ToString("f4")+   "     "+m_fDataReadRealY.ToString("f4"));
					 }
					 //			Console::WriteLine(BinaryDataReader->ReadInt32().ToString());
					 StreamTextWriter->Close();
					 BinaryDataFileStream->Close( );
				 }
				 catch (Exception^ e)
				 {
					 //		if (dynamic_cast<FileNotFoundException^>(e))
					 //			Console::WriteLine("File '{0}' not found", fileName);
					 //		else
					 //			Console::WriteLine("Exception: ({0})", e);
					 //		return -1;
				 }


			 }
	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {

// 				this->chart1->ChartAreas["ChartArea1"]->AxisX->ScaleView->Zoomable = false;
// 				this->chart1->ChartAreas["ChartArea1"]->AxisY->ScaleView->Zoomable = false;
//				 chart1->AutoSize = true

				 // Back previous zoom
//				 this->chart1->ChartAreas["ChartArea1"]->AxisX->ScaleView->ZoomReset();
//				 this->chart1->ChartAreas["ChartArea1"]->AxisY->ScaleView->ZoomReset();

				 // Zoom in scall
// 		this->chart1->ChartAreas["ChartArea1"]->AxisX->ScaleView-> Zoom(-10, 10);
// 		this->chart1->ChartAreas["ChartArea1"]->AxisY->ScaleView-> Zoom(-5, 5);	

//			 this->chart1->ChartAreas["ChartArea1"]->AxisX->MulZoomCenter(0.5);
//		    this->chart1->ChartAreas["ChartArea1"]->AxisY->MulZoomCenter(0.5);

//			 this->chart1
			 }
};
}

