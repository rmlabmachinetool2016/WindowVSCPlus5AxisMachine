//#pragma once
#include "StdAfx.h"
#include "FiveAxisCNCForm.h"
using namespace RmFiveAxisCNC;
using namespace RmLabCNC;
#pragma region User define fucntion

void FiveAxisCNCForm::ShowGraph()
{

	double m_fRealTime,m_fPreRealTime,m_fTimeCounter,
		m_fRefX,m_fRefY,m_fRefZ,m_fRefC,m_fRefA,m_fPreRefX,m_fPreRefY,m_fPreRefZ,m_fPreRefC,m_fPreRefA,
		m_fRefX_1,m_fRefY_1,m_fRefZ_1,m_fRefC_1,m_fRefA_1,
		m_fRealX,m_fRealY,m_fRealZ,m_fRealC,m_fRealA,m_fPreRealX,m_fPreRealY,m_fPreRealZ,m_fPreRealC,m_fPreRealA,
		m_fRealX_1,m_fRealY_1,m_fRealZ_1,m_fRealC_1,m_fRealA_1,
		m_fewX, m_fewY,m_fewZ,m_fewC,m_fewA,
		m_fOutputControlX,m_fOutputControlY,m_fOutputControlZ,m_fOutputControlC,m_fOutputControlA,
		m_fPredictedOutputX,m_fPredictedOutputY,m_fPredictedOutputZ,m_fPredictedOutputC,m_fPredictedOutputA;


	m_fRealTime = 0.0;
	m_fRefX = 0;m_fRefY = 0;m_fRefZ = 0;
	m_fRefX_1 = 0.0;m_fRefY_1 = 0.0;m_fRefZ_1 = 0.0;
	double  realPositionX,realPositionY,realPositionZ;
	try
	{
	//	(_T("c:\\test"), _T("c:\\test1"), true);
	//	File.Copy(FileBinaryData,FileBinaryDataMoveToMatlabDir,true);
	//	using namespace std; 
	//	CopyFile("temp","temp1",true);
		//File.Copy("temp","temp1",true);
		System::IO::File::Copy(FileBinaryData,FileBinaryDataMoveToMatlabDir,true);
	}
	catch (Exception^ e)
	{
		
	}

	try
	{
		
		FileStream^ BinaryDataFileStream = gcnew FileStream(FileBinaryData, FileMode::Open);
		BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);
		StreamWriter^ StreamTextWriter = gcnew StreamWriter(FileTextToWrite);
		//		Console::WriteLine("contents of {0}:", fileName);
//		StreamTextWriter->WriteLine("Time         TCounter     RefX       RefY      RefZ      RefC      RefA      RealX      RealY      RealZ      RealC      RealA    RefX_1       RefY_1        RefZ_1     eW0        eW1        eW2       Output0     Output1     Output2   PredictedOut0   PredictedOut1   PredictedOut2  ");
		StreamTextWriter->WriteLine("Time         TCounter     RefX       RefY      RefZ      RefC      RefA      RealX      RealY      RealZ      RealC      RealA      eWX        eWY        eWZ         eWC        eWA        OutX     OutY     OutZ     OutC     OutA    EstOutX    EstOutY    EstOutZ    EstOutC    EstOutA");
		while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
		{
			m_fPreRealTime = m_fRealTime;
			m_fRealTime  = BinaryDataReader->ReadDouble();
			m_fTimeCounter = BinaryDataReader->ReadDouble();
			m_fPreRefX = m_fRefX;m_fPreRefY = m_fRefY;
			m_fRefX  = BinaryDataReader->ReadDouble();
			m_fRefY  = BinaryDataReader->ReadDouble();
			m_fRefZ  = BinaryDataReader->ReadDouble();
			m_fRefC  = BinaryDataReader->ReadDouble();
			m_fRefA  = BinaryDataReader->ReadDouble();
			m_fRealX  = BinaryDataReader->ReadDouble();
			m_fRealY  = BinaryDataReader->ReadDouble();
			m_fRealZ  = BinaryDataReader->ReadDouble();
			m_fRealC  = BinaryDataReader->ReadDouble();
			m_fRealA  = BinaryDataReader->ReadDouble();
			if ((m_fRealTime-m_fPreRealTime)>DOUBLE_TOLERANCE)
			{
				m_fRefX_1 = (m_fRefX - m_fPreRefX)/(m_fRealTime-m_fPreRealTime);
				m_fRefY_1 = (m_fRefY - m_fPreRefY)/(m_fRealTime-m_fPreRealTime);
				m_fRefZ_1 = (m_fRefZ - m_fPreRefZ)/(m_fRealTime-m_fPreRealTime);
				m_fRealX_1 = (m_fRealX - m_fPreRealX)/(m_fRealTime-m_fPreRealTime);
				m_fRealY_1 = (m_fRealY - m_fPreRealY)/(m_fRealTime-m_fPreRealTime);
				m_fRealZ_1 = (m_fRealZ - m_fPreRealZ)/(m_fRealTime-m_fPreRealTime);
			}

			m_fewX  = 1000.0*BinaryDataReader->ReadDouble();
			m_fewY  = 1000.0*BinaryDataReader->ReadDouble();
			m_fewZ  = 1000.0*BinaryDataReader->ReadDouble();
			m_fewC  = 1000.0*BinaryDataReader->ReadDouble();
			m_fewA  = 1000.0*BinaryDataReader->ReadDouble();
			m_fOutputControlX  = BinaryDataReader->ReadDouble();
			m_fOutputControlY  = BinaryDataReader->ReadDouble();
			m_fOutputControlZ  = BinaryDataReader->ReadDouble();
			m_fOutputControlC  = BinaryDataReader->ReadDouble();
			m_fOutputControlA  = BinaryDataReader->ReadDouble();
			m_fPredictedOutputX  = BinaryDataReader->ReadDouble();
			m_fPredictedOutputY  = BinaryDataReader->ReadDouble();
			m_fPredictedOutputZ  = BinaryDataReader->ReadDouble();
			m_fPredictedOutputC  = BinaryDataReader->ReadDouble();
			m_fPredictedOutputA  = BinaryDataReader->ReadDouble();
			// 					 realPositionX  = BinaryDataReader->ReadDouble();
			// 					 realPositionY  = BinaryDataReader->ReadDouble();
			this->chartRealReferenceContour->Series["RefX_Time"]->Points->AddXY(m_fRealTime,m_fRefX);
			this->chartRealReferenceContour->Series["RefY_Time"]->Points->AddXY(m_fRealTime,m_fRefY);
			this->chartRealReferenceContour->Series["RefZ_Time"]->Points->AddXY(m_fRealTime,m_fRefZ);
			this->chartRealReferenceContour->Series["RealX_1"]->Points->AddXY(m_fRealTime,m_fRealX_1);
			this->chartRealReferenceContour->Series["RealY_1"]->Points->AddXY(m_fRealTime,m_fRealY_1);

			this->chartRealReferenceContour->Series["RefX_1_UX"]->Points->AddXY(m_fRefX_1,m_fOutputControlX);
			this->chartRealReferenceContour->Series["RefY_1_UY"]->Points->AddXY(m_fRefY_1,m_fOutputControlY);

			this->chartRealReferenceContour->Series["RealContour"]->Points->AddXY(m_fRealX,m_fRealY);
			this->chartRealReferenceContour->Series["ReferenceContour"]->Points->AddXY(m_fRefX,m_fRefY);
 			this->chartRealReferenceContour->Series["XControlVoltage"]->Points->AddXY(m_fRealTime,m_fOutputControlX);
			this->chartRealReferenceContour->Series["YControlVoltage"]->Points->AddXY(m_fRealTime,m_fOutputControlY);//m_fOutputControl1);
			this->chartRealReferenceContour->Series["ZControlVoltage"]->Points->AddXY(m_fRealTime,m_fOutputControlZ);//m_fOutputControl1);
			this->chartRealReferenceContour->Series["ew0Error"]->Points->AddXY(m_fRealTime,m_fewX);//m_fRefX_1);//m_few0);
			this->chartRealReferenceContour->Series["ew1Error"]->Points->AddXY(m_fRealTime,m_fewY);//m_fRefY_1);//m_few1);
			this->chartRealReferenceContour->Series["ew2Error"]->Points->AddXY(m_fRealTime,m_fewZ);//m_fRefY_1);//m_few1);

			this->chartRealReferenceContour->Series["XPredictedVoltage"]->Points->AddXY(m_fRealTime,m_fPredictedOutputX);//m_fRefX_1);//m_fPredictedOutput0);
			this->chartRealReferenceContour->Series["YPredictedVoltage"]->Points->AddXY(m_fRealTime,m_fPredictedOutputY);//m_fRefY_1);//m_fPredictedOutput1);
			this->chartRealReferenceContour->Series["ZPredictedVoltage"]->Points->AddXY(m_fRealTime,m_fPredictedOutputZ);//m_fRefY_1);//m_fPredictedOutput1);

			// Show basic graphs
			this->chartRealReferenceContour->Series["RefX_Time"]->Enabled = false; // true
			this->chartRealReferenceContour->Series["RefY_Time"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefZ_Time"]->Enabled = false;
			// Disable some other series
			this->chartRealReferenceContour->Series["RealX_1"]->Enabled = false;
			this->chartRealReferenceContour->Series["RealX_2"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefX_1"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefX_2"]->Enabled = false;
			this->chartRealReferenceContour->Series["RealY_1"]->Enabled = false;
			this->chartRealReferenceContour->Series["RealY_2"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefY_1"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefY_2"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefX_1_UX"]->Enabled = false;
			this->chartRealReferenceContour->Series["RefY_1_UY"]->Enabled = false;
// 			this->chartRealReferenceContour->Series["ew0Error"]->Enabled = false;
// 			this->chartRealReferenceContour->Series["ew1Error"]->Enabled = false;
			this->chartRealReferenceContour->Series["ec0"]->Enabled = false;
			this->chartRealReferenceContour->Series["ec1"]->Enabled = false;
			
//("Time         TCounter     RefX       RefY      RefZ      RefC      RefA      RealX      RealY      RealZ      RealC      eWX        eWY        eWZ         eWC        eWA        OutX     OutY     OutZ     OutC     OutA    EstOutX    EstOutY    EstOutZ    EstOutC    EstOutA");
// 3Axis Control
// 			StreamTextWriter->WriteLine(m_fRealTime.ToString("f6")
// 				+"    "+m_fTimeCounter.ToString("F6")
// 				+"    "+m_fRefX.ToString("F4") + "     "+m_fRefY.ToString("f4") + "     "+m_fRefZ.ToString("f4")
// 				+"    "+m_fRealX.ToString("f4")+ "     "+m_fRealY.ToString("f4")+ "     "+m_fRealZ.ToString("f4")
// 				+"    "+m_fRefX_1.ToString("f4")+ "     "+m_fRefY_1.ToString("f4")+ "     "+m_fRefZ_1.ToString("f4")
// 				+"    "+m_fewX.ToString("f4")+ "     "+m_fewY.ToString("f4")+ "     "+m_fewZ.ToString("f4")
// 				+"    "+m_fOutputControlX.ToString("f4")+ "     "+m_fOutputControlY.ToString("f4")+ "     "+m_fOutputControlZ.ToString("f4")
// 				+"    "+m_fPredictedOutputX.ToString("f4")+ "     "+m_fPredictedOutputY.ToString("f4")+ "     "+m_fPredictedOutputZ.ToString("f4")
// 				);
// 5 Axis control
			StreamTextWriter->WriteLine(m_fRealTime.ToString("f6")
				+"    "+m_fTimeCounter.ToString("F6")
				+"    "+m_fRefX.ToString("F4") + "     "+m_fRefY.ToString("f4") + "     "+m_fRefZ.ToString("f4")+ "     "+m_fRefC.ToString("f4")+ "     "+m_fRefA.ToString("f4")
				+"    "+m_fRealX.ToString("f4")+ "     "+m_fRealY.ToString("f4")+ "     "+m_fRealZ.ToString("f4")+ "     "+m_fRealC.ToString("f4")+ "     "+m_fRealA.ToString("f4")
				+"    "+m_fewX.ToString("f4")+ "     "+m_fewY.ToString("f4")+ "     "+m_fewZ.ToString("f4")+ "     "+m_fewC.ToString("f4")+ "     "+m_fewA.ToString("f4")
				+"    "+m_fOutputControlX.ToString("f4")+ "     "+m_fOutputControlY.ToString("f4")+ "     "+m_fOutputControlZ.ToString("f4")+ "     "+m_fOutputControlC.ToString("f4")+ "     "+m_fOutputControlA.ToString("f4")
				+"    "+m_fPredictedOutputX.ToString("f4")+ "     "+m_fPredictedOutputY.ToString("f4")+ "     "+m_fPredictedOutputZ.ToString("f4")+ "     "+m_fPredictedOutputC.ToString("f4")+ "     "+m_fPredictedOutputA.ToString("f4")
				);
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

	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea3"]->AxisX->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea3"]->AxisY->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisX->LabelStyle->Format = "F2";
	this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->LabelStyle->Format = "F2";
	// Change properties of ChartArea1
	//			 this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->Minimum = -6;
	//			 this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->Maximum = 6;
	//			 this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->Minimum = this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->Minimum *967/522;
	//			 this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->Maximum = this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->Maximum*967/522;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->Interval = 1;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->LabelStyle->Interval = 5;
	// MajorGrid
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->MajorGrid->IntervalOffset = 0;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->MajorGrid->Interval = 10;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->MajorTickMark->IntervalOffset = 0;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->MajorTickMark->Enabled = true;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisX->MajorTickMark->Interval = 1;
	// Y Axis
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->MajorGrid->IntervalOffset = 0;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->MajorGrid->Interval = 6;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->MajorTickMark->IntervalOffset = 0;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->MajorTickMark->Enabled = true;
	this->chartRealReferenceContour->ChartAreas["ChartArea1"]->AxisY->MajorTickMark->Interval = 1;

	//Range of Chart
//	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Maximum = 36;
	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->Maximum = 500;//36;
//	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->Minimum = -36;
	this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisY->Minimum = -500;//-36;
	this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->Maximum = 500;//36;
	this->chartRealReferenceContour->ChartAreas["ChartArea4"]->AxisY->Minimum = -500;//-36;
	
}
void FiveAxisCNCForm::ClearGraph() 
{
	//	 this->chartRealReferenceContour->Series["RealContour"]->Points
	this->chartRealReferenceContour->Series["RealContour"]->Points->Clear();
	this->chartRealReferenceContour->Series["ReferenceContour"]->Points->Clear();
	this->chartRealReferenceContour->Series["XControlVoltage"]->Points->Clear();
	this->chartRealReferenceContour->Series["YControlVoltage"]->Points->Clear();
	this->chartRealReferenceContour->Series["ZControlVoltage"]->Points->Clear();
	this->chartRealReferenceContour->Series["ew0Error"]->Points->Clear();
	this->chartRealReferenceContour->Series["ew1Error"]->Points->Clear();
	this->chartRealReferenceContour->Series["ew2Error"]->Points->Clear();
	this->chartRealReferenceContour->Series["XPredictedVoltage"]->Points->Clear();
	this->chartRealReferenceContour->Series["YPredictedVoltage"]->Points->Clear();
	this->chartRealReferenceContour->Series["ZPredictedVoltage"]->Points->Clear();
	this->chartRealReferenceContour->Series["RefX_Time"]->Points->Clear();
	this->chartRealReferenceContour->Series["RefY_Time"]->Points->Clear();
	this->chartRealReferenceContour->Series["RefZ_Time"]->Points->Clear();

	this->chartRealReferenceContour->Series["RefX_1_UX"]->Points->Clear();
	this->chartRealReferenceContour->Series["RefY_1_UY"]->Points->Clear();

	// 			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->ScaleView->Size = 
	// 				 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->ScaleView->Size* 900/600;
	//			 this->chartRealReferenceContour->ChartAreas["ChartArea2"]->AxisX->ScaleView->ViewMinimum;

	System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea5 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
	System::Windows::Forms::DataVisualization::Charting::Series^  serie5 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
//	serie5->Enabled
	//			 serie5->Points.Clear();
//				 chartArea5->AxisX.Maximum
}
void FiveAxisCNCForm::SaveCurrentConfig() 
{
	StreamWriter^ StreamTextWriter = gcnew StreamWriter(FileConfig);
	StreamTextWriter->WriteLine("SampleTime "+textBoxSampleTime->Text);
	StreamTextWriter->WriteLine("TimeRun "+textBoxTimeRun->Text);
	StreamTextWriter->Close();
}
void FiveAxisCNCForm::InitConfigLastTime() 
{

}
void FiveAxisCNCForm::UpdateRealPosition() 
{
	textBoxXPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.X);
	textBoxY1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Y1);
	textBoxY2Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Y2);
	textBoxZPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.Z);
	textBoxCPosition->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.C);
	textBoxA1Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.A1);
	textBoxA2Position->Text = System::Convert::ToString(RmLabFiveAxisCNC.m_CNCRealPos.A2);
}
void FiveAxisCNCForm::InitStaticVariable() 
{
	bbuttonXPlusMouseDown = false;bbuttonXMinusMouseDown = false;
	bbuttonY1PlusMouseDown = false;bbuttonY1MinusMouseDown = false;
	bbuttonZPlusMouseDown = false;bbuttonZMinusMouseDown = false;	

	RmLabFiveAxisCNC.m_CNCPosManualStep.X = 0.0;
	RmLabFiveAxisCNC.m_CNCPosManualStep.Y = 0.0;
	RmLabFiveAxisCNC.m_CNCPosManualStep.Z = 0.0;

	m_fManualSpeed = 0.01; // mm/s
// 		mt_Identity.resize(MTSIZE);
// 
// 		mt_M.resize(NUM_COUNTER,NUM_COUNTER);mt_Kc.resize(NUM_COUNTER,NUM_COUNTER);
// 		mt_Fc.resize(NUM_COUNTER,NUM_COUNTER);mt_Fbr.resize(NUM_COUNTER,NUM_COUNTER);
// 
// 		mt_KcFW.resize(NUM_COUNTER,NUM_DIRECT);mt_FcFW.resize(NUM_COUNTER,NUM_DIRECT);
// 		mt_FbrFW.resize(NUM_COUNTER,NUM_DIRECT);mt_FvsFW.resize(NUM_COUNTER,NUM_DIRECT);
// 		mt_KcBW.resize(NUM_COUNTER,NUM_DIRECT);mt_FcBW.resize(NUM_COUNTER,NUM_DIRECT);
// 		mt_FbrBW.resize(NUM_COUNTER,NUM_DIRECT);mt_FvsBW.resize(NUM_COUNTER,NUM_DIRECT);
// 
// 		mt_Translate.resize(MTSIZE,MTSIZE);
// 		mt_rotR.resize(MTSIZE,MTSIZE);mt_rotR_1.resize(MTSIZE,MTSIZE);
// 		mt_rotR_2.resize(MTSIZE,MTSIZE);
// 		mt_Kvw.resize(NUM_COUNTER,NUM_COUNTER);mt_Kpw.resize(NUM_COUNTER,NUM_COUNTER);mt_Kiw.resize(NUM_COUNTER,NUM_COUNTER);
// 		mt_Kvl.resize(NUM_COUNTER,NUM_COUNTER);mt_Kpl.resize(NUM_COUNTER,NUM_COUNTER);mt_Kil.resize(NUM_COUNTER,NUM_COUNTER);
// 
// 
// 		mt_MachineCordinate.resize(MTSIZE,MTSIZE);
// 		mt_ToolCordinate.resize(MTSIZE,MTSIZE);
// 		//	mt_ToolWorkPieceRelation.resize(MTSIZE,MTSIZE);
// 		mt_WorkPieceCordinate.resize(MTSIZE,MTSIZE);
// 		mt_ContourWorkPieceCordinate.resize(MTSIZE,MTSIZE);
// 		mt_ContourMachineCordinate.resize(MTSIZE,MTSIZE);
// 
// 		mt_M.clear();mt_Kc.clear();mt_Fc.clear();mt_Fbr.clear();
// 		mt_KcFW.clear();mt_FcFW.clear();mt_FbrFW.clear();mt_FvsFW.clear();
// 		mt_KcBW.clear();mt_FcFW.clear();mt_FbrBW.clear();mt_FvsBW.clear();
// 		mt_Translate.clear();
// 		mt_rotR.clear();mt_rotR_1.clear();mt_rotR_2.clear();
// 
// 		vec_el.resize(NUM_COUNTER);vec_el_1.resize(NUM_COUNTER);vec_el_2.resize(NUM_COUNTER);
// 		vec_ew.resize(NUM_COUNTER);vec_ew_1.resize(NUM_COUNTER);vec_ew_2.resize(NUM_COUNTER);
// 		vec_Pre_refr.resize(NUM_COUNTER);vec_Pre_refr_1.resize(NUM_COUNTER);
// 		vec_refr.resize(NUM_COUNTER);vec_refr_1.resize(NUM_COUNTER);vec_refr_2.resize(NUM_COUNTER);
// 		vec_Pre_realx.resize(NUM_COUNTER); vec_Pre_realx_1.resize(NUM_COUNTER);
// 		vec_realx.resize(NUM_COUNTER);vec_realx_1.resize(NUM_COUNTER);vec_realx_2.resize(NUM_COUNTER);
// 		vec_OutputControl.resize(NUM_COUNTER);vec_PredictedOutput.resize(NUM_COUNTER);
// 
// 		vec_AbsolutePosition.resize(NUM_COUNTER);
// 		vec_AbsolutePosition.clear();
// 
// 		vec_el.clear();vec_el_1.clear();vec_el_2.clear();vec_ew.clear();vec_ew_1.clear();vec_ew_2.clear();
// 		vec_Pre_refr.clear();vec_Pre_refr_1.clear();vec_refr.clear();vec_refr_1.clear();vec_refr_2.clear();
// 		vec_Pre_realx.clear();vec_Pre_realx_1.clear();vec_realx.clear();vec_realx_1.clear();vec_realx_2.clear();
// 		vec_OutputControl.clear();vec_PredictedOutput.clear();
}
void FiveAxisCNCForm::FiveAxisCNCFormSettingSave() 
{
			StreamWriter^ StreamTextWriter;
// 	Show my last settings
// 		Show default settings
// 		Show settings for experiment
// 		Show settings for X,Y,Z,C,A tuning
// 		Show settings for machining

// 	My last settings
// 		Default settings
// 		Settings for experiment
// 		Settings for X,Y,Z,C,A tuning
// 		Settings for machining
		StreamTextWriter = gcnew StreamWriter(InitialConfigFolder+"MyLastSettings.txt");
		StreamTextWriter->WriteLine("SettingSelected "+comboBoxProgramStartSetting->SelectedIndex.ToString());
		StreamTextWriter->WriteLine("textBoxSampleTime "+textBoxSampleTime->Text+" ms");
		StreamTextWriter->WriteLine("textBoxConfigFilename "+textBoxConfigFilename->Text);
		StreamTextWriter->WriteLine("textBoxGcodeFilename "+textBoxGcodeFilename->Text);
		StreamTextWriter->WriteLine("textBoxDataFileName "+textBoxDataFileName ->Text);
		StreamTextWriter->WriteLine("checkBoxSaveData "+checkBoxSaveData->Checked);
		StreamTextWriter->WriteLine("checkBoxLimitTime "+checkBoxLimitTime->Checked);
		if (radioButtonManualSpeedX1->Checked)
		{
			StreamTextWriter->WriteLine("radioButtonManualSpeedX1");
		};
		if (radioButtonManualSpeedX10->Checked)
		{
			StreamTextWriter->WriteLine("radioButtonManualSpeedX10");
		};
		if (radioButtonManualSpeedX100->Checked)
		{
			StreamTextWriter->WriteLine("radioButtonManualSpeedX100");
		};
		if (radioButtonManualSpeedX1000->Checked)
		{
			StreamTextWriter->WriteLine("radioButtonManualSpeedX1000");
		};
		StreamTextWriter->WriteLine("comboBoxProgramStartSetting "+comboBoxProgramStartSetting->SelectedIndex);
		StreamTextWriter->WriteLine("comboBoxLoadSetting "+comboBoxLoadSetting->SelectedIndex);
		StreamTextWriter->WriteLine("comboBoxControllerType "+comboBoxControllerType->SelectedIndex);
		StreamTextWriter->WriteLine("comboBoxFrictionModel "+comboBoxFrictionModel->SelectedIndex);
		StreamTextWriter->WriteLine("comboBoxDisturbanceObserver "+comboBoxDisturbanceObserver->SelectedIndex);
		StreamTextWriter->Close();
}
void FiveAxisCNCForm::InitSettingStringScan(System::String^ scanString,System::String^ &strName,System::String^ &strValue)
{
	System::String^ stringResult;
	int i= 0;
	double temp = 0.0;
	scanString = " "+ scanString;

	stringResult = "";
	while ((i < scanString->Length) &&(scanString[i] ==' ')) i++;
	while ((i< scanString->Length ) &&(scanString[i] !=' '))
	{
		stringResult = stringResult+ scanString[i];
		i++;
	}
	strName = stringResult;
	//	stringResult = "Deo hieu"; // Can not understand why cannot reset stringResult to "";
	System::String^ stringValue;
	stringValue = "";
	while ((i<scanString->Length) &&(scanString[i] ==' ')) i++;
	while ((i<scanString->Length) &&(scanString[i] !=' '))
	{
		stringValue= stringValue+ scanString[i];
		i++;
	}
	strValue = stringValue;
	//	sscanf(strRead,"%f",&temp);
}// end InitSettingStringScan

void FiveAxisCNCForm::SettingFormVariable(System::String^ strName,System::String^ strValue)
{


	if (strName == "textBoxSampleTime") textBoxSampleTime->Text = strValue;
	if (strName == "textBoxConfigFilename") textBoxConfigFilename->Text = strValue;
	if (strName == "textBoxGcodeFilename") textBoxGcodeFilename->Text = strValue;
	if (strName == "textBoxDataFileName") textBoxDataFileName->Text = strValue;
	if (strName == "checkBoxSaveData")
	{
		if (strValue=="False")
		{
			checkBoxSaveData->Checked = false;
		} 
		else
		{
			checkBoxSaveData->Checked = true;
		};
	};
	if (strName == "checkBoxLimitTime")
	{
		if (strValue=="False")
		{
			checkBoxLimitTime->Checked = false;
		} 
		else
		{
			checkBoxLimitTime->Checked = true;
		};
	};
	if (strName == "radioButtonManualSpeedX1") radioButtonManualSpeedX1->Checked = true;
	if (strName == "radioButtonManualSpeedX10") radioButtonManualSpeedX10->Checked = true;
	if (strName == "radioButtonManualSpeedX100") radioButtonManualSpeedX100->Checked = true;
	if (strName == "radioButtonManualSpeedX1000") radioButtonManualSpeedX1000->Checked = true;

	if (strName == "comboBoxControllerType") comboBoxControllerType->SelectedIndex = System::Convert::ToByte(strValue);
	if (strName == "comboBoxFrictionModel") {comboBoxFrictionModel->SelectedIndex = System::Convert::ToByte(strValue); }
	if (strName == "comboBoxDisturbanceObserver") comboBoxDisturbanceObserver->SelectedIndex = System::Convert::ToByte(strValue);
//    if (strName == "comboBoxProgramStartSetting") comboBoxProgramStartSetting->SelectedIndex = System::Convert::ToByte(strValue);
 //   if (strName == "comboBoxLoadSetting") comboBoxLoadSetting->SelectedIndex = System::Convert::ToByte(strValue);
}
void FiveAxisCNCForm::FiveAxisCNCFormSettingLoad(short m_iIndex) 
{
	System::String^ strRead;
	System::String^ strName;
	System::String^ strValue;

	StreamReader^ SettingFileStream;
	switch (m_iIndex)
	{
	case 0:
		SettingFileStream = File::OpenText(InitialConfigFolder+"MyLastSettings.txt");
		break;
	case 1:
		SettingFileStream = File::OpenText(InitialConfigFolder+"DefaultSettings.txt");
		break;
	case 2:
		SettingFileStream = File::OpenText(InitialConfigFolder+"ExperimentalSettings.txt");
		break;
	case 3:
		SettingFileStream = File::OpenText(InitialConfigFolder+"TuningSettings.txt");
		break;
	case 4:
		SettingFileStream = File::OpenText(InitialConfigFolder+"MachiningSettings.txt");
		break;
	default:
		SettingFileStream = File::OpenText(InitialConfigFolder+"MyLastSettings.txt");
		break;
	}
	while ((strRead = SettingFileStream->ReadLine()) != nullptr)
	{
		InitSettingStringScan(strRead,strName,strValue);
		SettingFormVariable(strName,strValue);
	}
	SettingFileStream->Close();
}
void FiveAxisCNCForm::FiveAxisCNCFormSettingInit() 
{
	System::String^ strRead;
	System::String^ strName;
	System::String^ strValue;
	short m_iValue;

	StreamReader^ SettingFileStream;
	SettingFileStream = File::OpenText(InitialConfigFolder+"MyLastSettings.txt");
	strRead = SettingFileStream->ReadLine();
	InitSettingStringScan(strRead,strName,strValue);
	SettingFileStream->Close();

	try
	{
		m_iValue= System::Convert::ToByte(strValue);
	}
	catch (Exception^ e)
	{
		m_iValue = 0;
	}
	FiveAxisCNCFormSettingLoad(m_iValue); 
}
void FiveAxisCNCForm::UpdateSettingParameters() 
{
	NewGUIProcessing.SAMPLING_TIME = System::Convert::ToDouble(textBoxSampleTime->Text)/1000.0;//  change ms -> s;
	RmLabFiveAxisCNC.m_iSelectedControllerType = comboBoxControllerType->SelectedIndex;
	RmLabFiveAxisCNC.m_iSelectedFrictionModel = comboBoxFrictionModel->SelectedIndex;
	RmLabFiveAxisCNC.m_iSelectedDisturbanceObserver = comboBoxDisturbanceObserver->SelectedIndex;
	RmLabFiveAxisCNC.m_iSelectedCharacteristicCurveModel = comboBoxDisturbanceObserver->SelectedIndex;

}
#pragma endregion
