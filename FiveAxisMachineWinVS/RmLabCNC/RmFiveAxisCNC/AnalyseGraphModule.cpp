#include "StdAfx.h"
#include "AnalyseGraphModule.h"
using namespace System::IO;
using namespace RmLabCNC;
AnalyseGraphModule::AnalyseGraphModule(void)
{
}
void AnalyseGraphModule::ReadBinaryDataFile(System::String^ FileBinaryData)
{
	try
	{
		FileStream^ BinaryDataFileStream = gcnew FileStream(FileBinaryData, FileMode::Open);
		BinaryReader^ BinaryDataReader = gcnew BinaryReader(BinaryDataFileStream);

//		Console::WriteLine("contents of {0}:", fileName);
		while (BinaryDataReader->BaseStream->Position < BinaryDataReader->BaseStream->Length)
			BinaryDataReader->ReadInt32();
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

