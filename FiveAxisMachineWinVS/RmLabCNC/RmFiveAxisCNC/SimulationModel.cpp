#include "StdAfx.h"
#include "SimulationModel.h"

using namespace RmLabCNC;
SimulationModel::SimulationModel(void)
{
	mt_Simul1_M.resize(NUMBERAXIS,NUMBERAXIS);
	mt_SimulPeakHeight.resize(NUMBERAXIS*2,PEAKLIMIT);
	mt_SimulPeakPos.resize(NUMBERAXIS*2,PEAKLIMIT);
	mt_SimulPeakWid.resize(NUMBERAXIS*2,PEAKLIMIT);

	vec_Simulx.resize(NUMBERAXIS);
	vec_SimulPrex.resize(NUMBERAXIS);
	vec_Simulx_1.resize(NUMBERAXIS);
	vec_SimulPrex_1.resize(NUMBERAXIS);
	vec_SimulKc.resize(NUMBERAXIS);
	vec_SimulFc.resize(NUMBERAXIS);
	vec_InertiaForce.resize(NUMBERAXIS);
	vec_SimulNominalFc.resize(NUMBERAXIS);
	vec_SimulNominalFv.resize(NUMBERAXIS);
	vec_SimulNumberPeak.resize(NUMBERAXIS*2);

	vec_Simulx.clear();
	vec_SimulPrex.clear();
	vec_Simulx_1.clear();
	vec_SimulPrex_1.clear();
	vec_SimulKc.clear();
	vec_SimulFc.clear();
	vec_InertiaForce.clear();	
	vec_SimulNominalFc.clear();
	vec_SimulNominalFv.clear();
	vec_SimulNumberPeak.clear();

	mt_Simul1_M.clear();
	mt_SimulPeakHeight.clear();
	mt_SimulPeakPos.clear();
	mt_SimulPeakWid.clear();
}
void SimulationModel::ResetSimulationModel()
{
	vec_Simulx.clear();
	vec_SimulPrex.clear();
	vec_Simulx_1.clear();
	vec_SimulPrex_1.clear();
}
vector<double> SimulationModel::GetSimulationModelPosition()
{
	return vec_Simulx;// vec_AbsolutePosition  vec_CNCPosition
}
void SimulationModel::SetControlInput(vector<double> vec_input)
{
	int i=0;
	CalculateInertiaForce(vec_input);
//	vec_Simulx_1 = vec_SimulPrex_1+ ((float)m_fSamplingTime)*prod(mt_Simul1_M,vec_SimulFriction);
	vec_Simulx_1 = vec_Simulx_1 + ((float)m_fSamplingTime) * prod(mt_Simul1_M,vec_InertiaForce);  
	vec_Simulx = vec_Simulx+ ((float)m_fSamplingTime) *vec_Simulx_1;

// 	vec_Simulx(1) = vec_Simulx_1(0);
// 	vec_Simulx(3) = vec_input(0);
// 	vec_Simulx(4) = vec_InertiaForce(0);
// 	vec_Simulx(5) = vec_InertiaForce(0);
	for (i=1; i<NUMBERAXIS; i++)
	{
		vec_Simulx(i) = 0.0;
	}

//	vec_SimulPrex_1 = vec_Simulx_1;
//	vec_Simulx = vec_SimulPrex+ ((float)m_fSamplingTime) *vec_Simulx_1;
}
double SimulationModel::gaussian(double x, double pos, double wid)
{
	//%  gaussian(x,pos,wid) = gaussian peak centered on pos, half-width=wid
	//%  x may be scalar, vector, or matrix, pos and wid both scalar
	//%  T. C. O'Haver, 1988
	//% Example: gaussian([1 2 3],1,2) gives result [0.5000    1.0000    0.5000]
	return exp(-((x-pos)/(0.6005612*wid))*((x-pos)/(0.6005612*wid)));
}
void SimulationModel::CalculateInertiaForce(vector<double> vec_SumForce)
{
	int i=0,j=0;
		for (i=0; i<NUMBERAXIS; i++)
		{
// 			if (vec_Simulx_1(i)>=0)
// 			{
// 				vec_InertiaForce(i) =  vec_SimulFc(i)+vec_SimulKc(i)*vec_Simulx_1(i);
// 
// 			} 
// 			else
// 			{
// 				vec_InertiaForce(i) =  -vec_SimulFc(i)+vec_SimulKc(i)*vec_Simulx_1(i);
// 
// 			}


				if (vec_Simulx_1(i)>=0)
				{
					vec_InertiaForce(i) = vec_SimulNominalFc(i)+vec_SimulNominalFv(i)*vec_Simulx_1(i);
					for (j=0;j<vec_SimulNumberPeak(i*2);j++)
					{
						vec_InertiaForce(i) = vec_InertiaForce(i)+ mt_SimulPeakHeight(i*2,j)*gaussian(vec_Simulx_1(i),mt_SimulPeakPos(i*2,j),mt_SimulPeakWid(i*2,j));
					}
				}
				else
				{
					vec_InertiaForce(i) = -vec_SimulNominalFc(i)+vec_SimulNominalFv(i)*vec_Simulx_1(i);
					for (j=0;j<vec_SimulNumberPeak(i*2+1);j++)
					{
						vec_InertiaForce(i) = vec_InertiaForce(i)+ mt_SimulPeakHeight(i*2+1,j)*gaussian(vec_Simulx_1(i),mt_SimulPeakPos(i*2+1,j),mt_SimulPeakWid(i*2+1,j));
					}
				}


			vec_InertiaForce(i) = vec_SumForce(i)-vec_InertiaForce(i);

			vec_InertiaForce(i) = vec_InertiaForce(i)+ ((double)rand() / (RAND_MAX + 1) * (1+1)-1);
// 			if (fabs(vec_InertiaForce(i))>fabs(vec_SumForce(i)))
// 			{
// 				vec_InertiaForce(i) = 0.0;
// 			} 
// 			else
// 			{
// 				vec_InertiaForce(i) = vec_SumForce(i)-vec_InertiaForce(i);
// 			}
		}
}
void SimulationModel::InitSimulNonlinearFrictionData(System::String^ fileNLF)
{
	int i=0,j=0;

	try
	{
		FileStream^ BinaryDataFileStream = gcnew FileStream(fileNLF, FileMode::Open);
		BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);

		//	while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
		{

			for (i=0;i<NUMBERAXIS;i++)
			{
				vec_SimulNominalFc(i) = BinaryDataReader->ReadDouble(); // Read nominal Couloumb Friction
				vec_SimulNominalFv(i) = BinaryDataReader->ReadDouble(); // Read nominal viscous coefficient

				vec_SimulNumberPeak(i*2)=BinaryDataReader->ReadByte();
				for (j=0;j<((int)vec_SimulNumberPeak(i*2));j++)
				{
					mt_SimulPeakHeight(i*2,j) = BinaryDataReader->ReadDouble();
					mt_SimulPeakPos(i*2,j) = BinaryDataReader->ReadDouble();
					mt_SimulPeakWid(i*2,j) = BinaryDataReader->ReadDouble();
				}
				vec_SimulNumberPeak(i*2+1)=BinaryDataReader->ReadByte();
				for (j=0;j<((int)vec_SimulNumberPeak(i*2+1));j++)
				{
					mt_SimulPeakHeight(i*2+1,j) = BinaryDataReader->ReadDouble();
					mt_SimulPeakPos(i*2+1,j) = BinaryDataReader->ReadDouble();
					mt_SimulPeakWid(i*2+1,j) = BinaryDataReader->ReadDouble();
				}
			}

		}
		//			Console::WriteLine(BinaryDataReader->ReadInt32().ToString());
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
void SimulationModel::rmsscanf(System::String^ scanString,System::String^ &strName,double &fValue)
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
	try
	{
		fValue= System::Convert::ToDouble(stringValue);
	}
	catch (Exception^ e)
	{
		fValue = 0.0;
	}

	//	sscanf(strRead,"%f",&temp);
}// end rmsscanf
void SimulationModel::InitVariableName(System::String^ strName,double fValue)
{
	mt_Simul1_M.resize(NUMBERAXIS,NUMBERAXIS);

	vec_Simulx.resize(NUMBERAXIS);
	vec_SimulPrex.resize(NUMBERAXIS);
	vec_Simulx_1.resize(NUMBERAXIS);
	vec_SimulPrex_1.resize(NUMBERAXIS);
	vec_SimulKc.resize(NUMBERAXIS);
	vec_SimulFc.resize(NUMBERAXIS);
	vec_InertiaForce.resize(NUMBERAXIS);


	// Weight 
	if (strName == "MX") mt_Simul1_M(0,0) = 1.0/fValue;
	if (strName == "MY1") mt_Simul1_M(1,1) = 1.0/fValue;
	if (strName == "MY2") mt_Simul1_M(2,2) = 1.0/fValue;
	if (strName == "MZ") mt_Simul1_M(3,3) = 1.0/fValue;
	if (strName == "MC") mt_Simul1_M(4,4) = 1.0/fValue;
	if (strName == "MA1") mt_Simul1_M(5,5) = 1.0/fValue;
	if (strName == "MA2") mt_Simul1_M(6,6) = 1.0/fValue;

	// Friction simulation model 
	// X Axis 
	if (strName == "Xao") vec_SimulFc(0) = fValue;
// 	if (strName == "Xa1") vec_SimulFc(0) = fValue;
// 	if (strName == "Xvs") vec_SimulFc(0) = fValue;
	if (strName == "Xa2") vec_SimulKc(0) = fValue;
	// Y1 Axis 
	if (strName == "Y1ao") vec_SimulFc(1) = fValue;
	if (strName == "Y1a2") vec_SimulKc(1) = fValue;
	// Y2 Axis 
	if (strName == "Y2ao") vec_SimulFc(2) = fValue;
	if (strName == "Y2a2") vec_SimulKc(2) = fValue;
	// Z Axis 
	if (strName == "Zao") vec_SimulFc(3) = fValue;
	if (strName == "Za2") vec_SimulKc(3) = fValue;
	// C Axis 
	if (strName == "Cao") vec_SimulFc(4) = fValue;
	if (strName == "Ca2") vec_SimulKc(4) = fValue;
	// A1 Axis 
	if (strName == "A1ao") vec_SimulFc(5) = fValue;
	if (strName == "A1a2") vec_SimulKc(5) = fValue;
	// A2 Axis 
	if (strName == "A2ao") vec_SimulFc(6) = fValue;
	if (strName == "A2a2") vec_SimulKc(6) = fValue;

}
void SimulationModel::LoadModelParameter(System::String^ fileSimulationModel)
{
	StreamReader^ GlobalVariableFileStream;
	// Initial Global variable 
	System::String^ strRead;
	System::String^ variableName;
	double variableValue;
	//	try 
	{
		//		Console::WriteLine("trying to open file {0}...", fileName);
		// 		GcodeFileStream = File::OpenText(FileGlobalVariable);
		// 		strRead = GcodeFileStream->ReadLine();

		GlobalVariableFileStream = File::OpenText(fileSimulationModel);
		while ((strRead = GlobalVariableFileStream->ReadLine()) != nullptr)
		{
			rmsscanf(strRead,variableName,variableValue);
			InitVariableName(variableName,variableValue);
		}

		// 		sscanf( tokenstring, "%80s", s ); // C4996 String
		// 		sscanf( tokenstring, "%c", &c );  // C4996 Character
		// 		sscanf( tokenstring, "%d", &i );  // C4996  Integer
		// 		sscanf( tokenstring, "%f", &fp ); // C4996 Real
		//		sscanf(strRead,"%30s%f",variableName,variableValue);

		GlobalVariableFileStream->Close();
	}
	//	catch (Exception^ e)
	{
		//		if (dynamic_cast<FileNotFoundException^>(e))
		//			Console::WriteLine("file '{0}' not found", fileName);
		//		else
		//			Console::WriteLine("problem reading file '{0}'", fileName);
	}


}