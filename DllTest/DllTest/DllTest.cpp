// DllTest.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "C:\rov\1718-control-station\GUI_3_29\main.cpp"
#include "C:\rov\1718-control-station\GUI_3_29\SerialPort.cpp"
#include "C:\rov\1718-control-station\GUI_3_29\SerialPort.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

#define DllExport __declspec(dllexport)

extern class DllExport SerialPort {
private:
	HANDLE handler;
	bool connected;
	COMSTAT status;
	DWORD errors;
public:
	char name;
	SerialPort();
	~SerialPort();
}
extern DllExport SerialPort* makeSerialPort(char *portname) {
	return new SerialPort(portname);
}
extern DllExport int readSerialPort(char *buffer, unsigned int buf_size) {
	DWORD bytesRead;
	unsigned int toRead;

	ClearCommError(this->handler, &this->errors, &this->status);

	if (this->status.cbInQue > 0) {
		if (this->status.cbInQue > buf_size) {
			toRead = buf_size;
		}
		else toRead = this->status.cbInQue;
	}

	if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL)) return bytesRead;

	return 0;
}
extern DllExport bool writeSerialPort() {
	DWORD bytesSend;

	if (!WriteFile(this->handler, (void*)buffer, buf_size, &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		return false;
	}
	else return true;
}
extern DllExport bool isConnected() {
	return this->connected;
}

