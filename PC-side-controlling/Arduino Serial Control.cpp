// Arduino Serial Control.cpp: 主项目文件。

#include <stdafx.h>
#include <windows.h>
#include <conio.h>

using namespace System;
using namespace System::IO::Ports;

bool loopCheck = true;

int main(array<System::String ^> ^args){
	system("title Project 2 - Quadricopter - Controling Platform");
	String^ portName;
	int baudRate = 9600;
	Console::WriteLine("PC to Arduino Control.");
	Console::WriteLine("Serial port baud rate: 9600.");
	Console::Write("Type in the port name: ");
	portName = Console::ReadLine();
	//arduino settings
	SerialPort^ arduino;
	arduino = gcnew SerialPort(portName, baudRate);
	//open port
	String^ command;
	String^ message;
	char button;
	try{
		arduino -> Open();
		Console::WriteLine("Initializing completed...");
		Sleep(1000);
		//Console::Clear();
		Console::WriteLine(">> Waiting for command...");
		while(loopCheck == true){
			button = getch();
			command = "" + button;
			//command = Console::ReadLine();

			if(String::Compare(command, "119")==0){
				arduino -> Write("w");
			}
			else if(String::Compare(command, "115")==0){
				arduino -> Write("s");
			}
			else if(String::Compare(command, "97")==0){
				arduino -> Write("a");
			}
			else if(String::Compare(command, "100")==0){
				arduino -> Write("d");
			}
			else if(String::Compare(command, "113")==0){
				arduino -> Write("q");
			}
			else if(String::Compare(command, "101")==0){
				arduino -> Write("e");
			}
			else if(String::Compare(command, "114")==0){
				arduino -> Write("r");
			}
			else if(String::Compare(command, "102")==0){
				arduino -> Write("f");
			}

			else if(String::Compare(command, "105")==0){
				arduino -> Write("i");
			}
			else if(String::Compare(command, "107")==0){
				arduino -> Write("k");
			}
			else if(String::Compare(command, "106")==0){
				arduino -> Write("j");
			}
			else if(String::Compare(command, "108")==0){
				arduino -> Write("l");
			}
			else if(String::Compare(command, "117")==0){
				arduino -> Write("u");
			}
			else if(String::Compare(command, "111")==0){
				arduino -> Write("o");
			}

			else if(String::Compare(command, "121")==0){
				arduino -> Write("y");
			}
			else if(String::Compare(command, "104")==0){
				arduino -> Write("h");
			}

			else if(String::Compare(command, "103")==0){
				arduino -> Write("g");
				Console::WriteLine(">> ########");
				Console::WriteLine(">> Waiting to get data...");
				Sleep(1000);
				message = arduino -> ReadLine();
				Console::WriteLine(">> GPS Information:");
				Console::WriteLine(">> " + message);
			}
			else if(String::Compare(command, "116")==0){
				arduino -> Write("t");
			}

			else if(String::Compare(command, "120")==0){
				//terminate program
				arduino -> Write("x");
				loopCheck = false;
			}

			else{
				Console::WriteLine(">> WARNING. Unknown command.");
			}
			//Console::Clear();
			Sleep(50);
		}
		arduino -> Close();
	}
	catch (IO::IOException^ e){
//		Console::Write("\n");
		Console::WriteLine(e -> GetType() -> Name + ": Port is not ready");
	}
	catch (ArgumentException ^ e){
//		Console::Write("\n");
		Console::WriteLine(e -> GetType() -> Name + ": incorrect port name syntax, must start with COM/com.");
	}
	Console::Write(">> Press enter to terminate the program...");
	Console::Read();
	return 0;
}

/*
ascii chart

91  [   92  \   93  ]   96  '   97  a
98  b   99  c   100 d   101 e   102 f
103 g   104 h   105 i   106 j   107 k
108 l   109 m   110 n   111 o   112 p
113 q   114 r   115 s   116 t   117 u
108 v   119 w   120 x   121 y   122 z
*/