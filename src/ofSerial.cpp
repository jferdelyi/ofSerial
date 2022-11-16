// This class is a standalone version of openFrameworks communication/serial
// Distributerd under the MIT License.
// Copyright (c) 2022 - Jean-François Erdelyi

// openFrameworks is distributed under the MIT License. This gives everyone the freedoms 
// to use openFrameworks in any context: commercial or non-commercial, public or private, open or closed source.
// Copyright (c) 2004 - openFrameworks Community
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
// and associated documentation files (the "Software"), to deal in the Software without restriction, 
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
// subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "ofSerial.h"

#if defined( TARGET_OSX )
	#include <getopt.h>
#endif

#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
	#include <sys/ioctl.h>
	#include <dirent.h>
	#include <fcntl.h>
#endif

#if defined( TARGET_LINUX )
	#include <linux/serial.h>
	#include <unistd.h>
	#include <cstring>
	// Some things for serial compilation:
	#define B14400	14400
	#define B28800	28800
#endif

#include <iostream>

using std::vector;
using std::string;

#ifdef TARGET_WIN32

// needed for serial bus enumeration:
// 4d36e978-e325-11ce-bfc1-08002be10318}
DEFINE_GUID (GUID_SERENUM_BUS_ENUMERATOR, 0x4D36E978, 0xE325,
0x11CE, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18);

void ofSerial::enumerateWin32Ports(){
	if(bPortsEnumerated == true){
		return;
	}

	HDEVINFO hDevInfo = nullptr;
	SP_DEVINFO_DATA DeviceInterfaceData;
	DWORD dataType, actualSize = 0;

	// Reset Port List
	nPorts = 0;
	// Search device set
	hDevInfo = SetupDiGetClassDevs((struct _GUID *)&GUID_SERENUM_BUS_ENUMERATOR, 0, 0, DIGCF_PRESENT);
	if(hDevInfo){
		int i = 0;
		unsigned char dataBuf[MAX_PATH + 1];
		while(TRUE){
			ZeroMemory(&DeviceInterfaceData, sizeof(DeviceInterfaceData));
			DeviceInterfaceData.cbSize = sizeof(DeviceInterfaceData);
			if(!SetupDiEnumDeviceInfo(hDevInfo, i, &DeviceInterfaceData)){
				// SetupDiEnumDeviceInfo failed
				break;
			}

		if(SetupDiGetDeviceRegistryPropertyA(hDevInfo,
											 &DeviceInterfaceData,
											 SPDRP_FRIENDLYNAME,
											 &dataType,
											 dataBuf,
											 sizeof(dataBuf),
											 &actualSize)){

			 sprintf(portNamesFriendly[nPorts], "%s", dataBuf);
			 portNamesShort[nPorts][0] = 0;

			 // turn blahblahblah(COM4) into COM4

			 char * begin = nullptr;
			 char * end = nullptr;
			 begin = strstr((char *)dataBuf, "(COM");

			 if(begin){
				 begin++;	// get rid of the (
				 end = strstr(begin, ")");
				 if(end){
					 *end = 0;   // get rid of the )...
					 strcpy(portNamesShort[nPorts], begin);
				 }
				 if(nPorts++ > MAX_SERIAL_PORTS)
					 break;
				}
			}
			i++;
		}
	}
	SetupDiDestroyDeviceInfoList(hDevInfo);

	bPortsEnumerated = false;
}

#endif  // TARGET_WIN32



//----------------------------------------------------------------
ofSerial::ofSerial(){

	#ifdef TARGET_WIN32

		nPorts = 0;
		bPortsEnumerated = false;

		portNamesShort = new char * [MAX_SERIAL_PORTS];
		portNamesFriendly = new char * [MAX_SERIAL_PORTS];
		for(int i = 0; i < MAX_SERIAL_PORTS; i++){
			portNamesShort[i] = new char[10];
			portNamesFriendly[i] = new char[MAX_PATH];
		}

	#else

	fd = -1;

	#endif

	bHaveEnumeratedDevices = false;
	bInited = false;
}


//----------------------------------------------------------------
ofSerial::~ofSerial(){
	close();

	#ifdef TARGET_WIN32

		nPorts = 0;
		bPortsEnumerated = false;

		for(int i = 0; i < MAX_SERIAL_PORTS; i++){
			delete [] portNamesShort[i];
			delete [] portNamesFriendly[i];
		}
		delete [] portNamesShort;
		delete [] portNamesFriendly;

	#endif

	bInited = false;
}

//----------------------------------------------------------------
#if defined( TARGET_OSX )
static bool isDeviceArduino( ofSerialDeviceInfo & A ){
	return (strstr(A.getDeviceName().c_str(), "usbserial") != nullptr
			|| strstr(A.getDeviceName().c_str(), "usbmodem") != nullptr);
}
#endif

//----------------------------------------------------------------
void ofSerial::buildDeviceList() {
	deviceType = "serial";
	devices.clear();
	vector <string> prefixMatch;

	#ifdef TARGET_OSX

		prefixMatch.push_back("cu.");
		prefixMatch.push_back("tty.");

	#endif

	#ifdef TARGET_LINUX

		prefixMatch.push_back("ttyACM");
		prefixMatch.push_back("ttyS");
		prefixMatch.push_back("ttyUSB");
		prefixMatch.push_back("rfc");

	#endif

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
		DIR *dir;
		struct dirent *entry;
		dir = opendir("/dev");

		std::string deviceName	= "";
		int deviceCount		= 0;

		if (dir == NULL){
			std::cerr << "buildDeviceList(): error listing devices in /dev" << "\r\n" << std::endl;
		} else {
			//for each device
			while((entry = readdir(dir)) != NULL){
				deviceName = (char *)entry->d_name;

				//we go through the prefixes
				for(auto & prefix: prefixMatch){
					//if the device name is longer than the prefix
					if(deviceName.size() > prefix.size()){
						//do they match ?
						if(deviceName.substr(0, prefix.size()) == prefix.c_str()){
							devices.push_back(ofSerialDeviceInfo("/dev/"+deviceName, deviceName, deviceCount));
							deviceCount++;
							break;
						}
					}
				}
			}
			closedir(dir);
		}

	#endif

	#ifdef TARGET_WIN32

		enumerateWin32Ports();
		for(int i = 0; i < nPorts; i++) {
			//NOTE: we give the short port name for both as that is what the user should pass and the short name is more friendly
			devices.push_back(ofSerialDeviceInfo(string(portNamesShort[i]), string(portNamesFriendly[i]), i));
		}

	#endif

	#if defined( TARGET_OSX )
		//here we sort the device to have the aruino ones first.
		partition(devices.begin(), devices.end(), isDeviceArduino);
		//we are reordering the device ids. too!
		int k = 0;
		for(auto & device: devices){
			device.deviceID = k++;
		}
	#endif

	bHaveEnumeratedDevices = true;
}

//----------------------------------------------------------------
vector <ofSerialDeviceInfo> ofSerial::getDeviceList(){
	buildDeviceList();
	return devices;
}

//----------------------------------------------------------------
void ofSerial::close(){

	#ifdef TARGET_WIN32

		if(bInited){
			SetCommTimeouts(hComm, &oldTimeout);
			CloseHandle(hComm);
			hComm = INVALID_HANDLE_VALUE;
			bInited = false;
		}

	#else

		if(bInited){
			tcsetattr(fd, TCSANOW, &oldoptions);
			::close(fd);
			bInited = false;
		}
		// [CHECK] -- anything else need to be reset?

	#endif
}

//----------------------------------------------------------------
bool ofSerial::setup(int deviceNumber, int baud, int data, int parity, int stop) {
	buildDeviceList();
	if(deviceNumber < (int)devices.size()){
		return setup(devices[deviceNumber].devicePath, baud, data, parity, stop);
	} else {
		std::cerr << "Couldn't find device " << deviceNumber << ", only " << devices.size() << " devices found" << std::endl;
		return false;
	}

}

//----------------------------------------------------------------
bool ofSerial::setup(string portName, int baud, int data, int parity, int stop) {
	bInited = false;

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )

		//lets account for the name being passed in instead of the device path
		if(portName.size() > 5 && portName.substr(0, 5) != "/dev/"){
			portName = "/dev/" + portName;
		}

		fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		if(fd == -1){
			std::cerr << "Unable to open " << portName << std::endl << std::endl;
			return false;
		}

		struct termios options;
		if(tcgetattr(fd, &oldoptions) != 0) {
			std::cerr <<  "Error " << errno <<" from tcgetattr: " << strerror(errno) << std::endl;
			return false;
		}
		options = oldoptions;

		switch(baud){
			case 300:
				cfsetispeed(&options, B300);
				cfsetospeed(&options, B300);
				break;
			case 1200:
				cfsetispeed(&options, B1200);
				cfsetospeed(&options, B1200);
				break;
			case 2400:
				cfsetispeed(&options, B2400);
				cfsetospeed(&options, B2400);
				break;
			case 4800:
				cfsetispeed(&options, B4800);
				cfsetospeed(&options, B4800);
				break;
			case 9600:
				cfsetispeed(&options, B9600);
				cfsetospeed(&options, B9600);
				break;
			case 14400:
				cfsetispeed(&options, B14400);
				cfsetospeed(&options, B14400);
				break;
			case 19200:
				cfsetispeed(&options, B19200);
				cfsetospeed(&options, B19200);
				break;
			case 28800:
				cfsetispeed(&options, B28800);
				cfsetospeed(&options, B28800);
				break;
			case 38400:
				cfsetispeed(&options, B38400);
				cfsetospeed(&options, B38400);
				break;
			case 57600:
				cfsetispeed(&options, B57600);
				cfsetospeed(&options, B57600);
				break;
			case 115200:
				cfsetispeed(&options, B115200);
				cfsetospeed(&options, B115200);
				break;
			case 230400:
				cfsetispeed(&options, B230400);
				cfsetospeed(&options, B230400);
				break;
			case 12000000: 
				cfsetispeed(&options, 12000000);
				cfsetospeed(&options, 12000000);	
				break;
			default:
				cfsetispeed(&options, B9600);
				cfsetospeed(&options, B9600);
				std::cerr << "setup(): cannot set " << baud << " bps, setting to 9600" << std::endl;
				break;
		}
		switch (data) {
			case 5:
				options.c_cflag |= CS5; // 5 bits per byte
				break;
			case 6:
				options.c_cflag |= CS6; // 6 bits per byte
				break;
			case 7:
				options.c_cflag |= CS7; // 7 bits per byte
				break;
			case 8:
			default:
				options.c_cflag |= CS8; // 8 bits per byte (most common)
		}
		switch (parity) {
			case OF_SERIAL_PARITY_E:
				options.c_cflag |= PARENB; // Enable parity bit
				options.c_cflag &= ~PARODD; // Even parity (disable odd bit)
				break;
			case OF_SERIAL_PARITY_O:
				options.c_cflag |= PARENB; // Enable parity bit
				options.c_cflag |= PARODD; // Odd parity
				break;
			case OF_SERIAL_PARITY_N:
			default:
				options.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		}
		switch (stop) {
			case 2:
				options.c_cflag |= CSTOPB; // Enable stop field, two stop bit used in communication
				break;
			case 1:
			default:
				options.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		}

		options.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
		#if defined( TARGET_LINUX )
			options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		#endif

		options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		#if defined( TARGET_LINUX )
			options.c_lflag &= ~ICANON;
			options.c_lflag &= ~ECHO; // Disable echo
		#endif

		options.c_lflag &= ~ECHOE; // Disable erasure
		options.c_lflag &= ~ECHONL; // Disable new-line echo

		#if defined( TARGET_LINUX )
			options.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
			options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		#endif

		options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
		options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	
		#if defined( TARGET_OSX )
			options.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
			options.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
		#endif
		
		options.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		options.c_cc[VMIN] = 0;

		if (tcsetattr(fd, TCSANOW, &options) != 0) {
			std::cerr <<  "Error " << errno <<" from tcsetattr: " << strerror(errno) << std::endl;
			return 1;
		}
		
		#ifdef TARGET_LINUX
			struct serial_struct kernel_serial_settings;
			if (ioctl(fd, TIOCGSERIAL, &kernel_serial_settings) == 0) {
				kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
				ioctl(fd, TIOCSSERIAL, &kernel_serial_settings);
			}
		#endif

		bInited = true;
		return true;

	#elif defined( TARGET_WIN32 )

		char pn[sizeof(portName)];
		int num;
		if(sscanf(portName.c_str(), "COM%d", &num) == 1){
			// Microsoft KB115831 a.k.a if COM > COM9 you have to use a different
			// syntax
			sprintf(pn, "\\\\.\\COM%d", num);
		} else {
			strncpy(pn, (const char *)portName.c_str(), sizeof(portName)-1);
		}

		// open the serial port:
		// "COM4", etc...

		hComm = CreateFileA(pn, GENERIC_READ|GENERIC_WRITE, 0, 0,
							OPEN_EXISTING, 0, 0);

		if(hComm == INVALID_HANDLE_VALUE){
			std::cerr << "setup(): unable to open " << portName << std::endl;
			return false;
		}


		// now try the settings:
		COMMCONFIG cfg;
		DWORD cfgSize;
		WCHAR buf[80];

		cfgSize = sizeof(cfg);
		GetCommConfig(hComm, &cfg, &cfgSize);

		// Default parity=N, data=8, stop=1
		int l_baud = baud;
		char l_parity = 'N';
		int l_data = 8;
		int l_stop = 1;
		switch (data) {
			case 5:
				l_data = 5; // 5 bits per byte
				break;
			case 6:
				l_data = 6; // 6 bits per byte
				break;
			case 7:
				l_data = 7; // 7 bits per byte
				break;
		}
		switch (l_parity) {
			case OF_SERIAL_PARITY_E:
				parity = 'E'; // Even parity
				break;
			case OF_SERIAL_PARITY_O:
				parity = 'O'; // Odd parity
				break;
		}
		switch (stop) {
			case 2:
				l_stop = 2; // Two stop bit used in communication
				break;
		}

		swprintf(buf, L"baud=%d parity=%d data=%d stop=%d", l_baud, l_parity, l_data, l_stop);

		if(!BuildCommDCBW(buf, &cfg.dcb)){
			std::cerr << "setup(): unable to build comm dcb, (" << buf << ")" << std::endl;
		}

		// Set baudrate and bits etc.
		// Note that BuildCommDCB() clears XON/XOFF and hardware control by default

		if(!SetCommState(hComm, &cfg.dcb)){
			std::cerr << "setup(): couldn't set comm state: " << cfg.dcb.BaudRate << " bps, xio " << cfg.dcb.fInX << "/" << cfg.dcb.fOutX << std::endl;
		}

		// Set communication timeouts (NT)
		COMMTIMEOUTS tOut;
		GetCommTimeouts(hComm, &oldTimeout);
		tOut = oldTimeout;
		// Make timeout so that:
		// - return immediately with buffered characters
		tOut.ReadIntervalTimeout = MAXDWORD;
		tOut.ReadTotalTimeoutMultiplier = 0;
		tOut.ReadTotalTimeoutConstant = 0;
		SetCommTimeouts(hComm, &tOut);

		bInited = true;
		return true;

	#else

		std::cerr << "Not implemented in this platform" << std::endl;
		return false;

	#endif
}

//----------------------------------------------------------------
bool ofSerial::writeData(const char singleByte) {
	return writeData((const unsigned char)singleByte);
}

//----------------------------------------------------------------
bool ofSerial::writeData(const unsigned char singleByte) {
	return writeData(&singleByte, 1) > 0;
}

//----------------------------------------------------------------
long ofSerial::writeData(const char* buffer, size_t length) {
	return writeData((const unsigned char*)buffer, length);
}

//----------------------------------------------------------------
long ofSerial::writeData(const std::string& buffer) {
	return writeData((const unsigned char*)buffer.c_str(), buffer.size());
}

//----------------------------------------------------------------
long ofSerial::writeData(const unsigned char* buffer, size_t length) {
	if(!bInited){
		std::cerr << "writeData(): serial not inited" << std::endl;
		return OF_SERIAL_ERROR;
	}

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
		size_t written=0;
		fd_set wfds;
		struct timeval tv;

		while (written < length) {
			auto n = write(fd, buffer + written, length - written);
			if (n < 0 && (errno == EAGAIN || errno == EINTR)) n = 0;
			if (n < 0) return OF_SERIAL_ERROR;
			if (n > 0) {
				written += n;
			} else {
				tv.tv_sec = 10;
				tv.tv_usec = 0;
				FD_ZERO(&wfds);
				FD_SET(fd, &wfds);
				n = select(fd+1, NULL, &wfds, NULL, &tv);
				if (n < 0 && errno == EINTR) n = 1;
				if (n <= 0) return OF_SERIAL_ERROR;
			}
		}
		return written;
	#elif defined(TARGET_WIN32)

		DWORD written;
		if(!WriteFile(hComm, buffer, length, &written,0)){
			 std::cerr << "writeData(): couldn't write to port" << std::endl;
			 return OF_SERIAL_ERROR;
		}
		return (int)written;

	#else

		return 0;

	#endif
}

//----------------------------------------------------------------
long ofSerial::readData(unsigned char* buffer, size_t length){
	if (!bInited){
		std::cerr << "readData(): serial not inited" << std::endl;
		return OF_SERIAL_ERROR;
	}

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )

		auto nRead = read(fd, buffer, length);
		if(nRead < 0){
			if ( errno == EAGAIN )
				return OF_SERIAL_NO_DATA;
			std::cerr << "readData(): couldn't read from port: " << errno << " " << strerror(errno) << std::endl;
			return OF_SERIAL_ERROR;
		}
		return nRead;

	#elif defined( TARGET_WIN32 )

		DWORD nRead = 0;
		if (!ReadFile(hComm, buffer, length, &nRead, 0)){
			std::cerr << "readData(): couldn't read from port" << std::endl;
			return OF_SERIAL_ERROR;
		}
		return (int)nRead;

	#else

		std::cerr << "Not defined in this platform" << std::endl;
		return -1;

	#endif
}

//----------------------------------------------------------------
long ofSerial::readData(char* buffer, size_t length){
	return readData(reinterpret_cast<unsigned char*>(buffer), length);
}

//----------------------------------------------------------------
long ofSerial::readData(std::string& buffer, size_t length) {
	char* tmpBuffer = new char[length + 1];
	memset(tmpBuffer, 0, sizeof(char) * (length + 1));
	const auto& nBytes = readData(tmpBuffer, length);
	tmpBuffer[nBytes] = '\0';
	buffer = std::string(tmpBuffer);
	delete[] tmpBuffer;
	return nBytes;
}

//----------------------------------------------------------------
int ofSerial::readData(){
	if(!bInited){
		std::cerr << "readData(): serial not inited" << std::endl;
		return OF_SERIAL_ERROR;
	}

	unsigned char tmpByte = 0;

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )

		int nRead = read(fd, &tmpByte, 1);
		if(nRead < 0){
			if ( errno == EAGAIN ){
				return OF_SERIAL_NO_DATA;
			}
			std::cerr << "readData(): couldn't read from port: " << errno << " " << strerror(errno) << std::endl;
			return OF_SERIAL_ERROR;
		}

		if(nRead == 0){
			return OF_SERIAL_NO_DATA;
		}

	#elif defined( TARGET_WIN32 )

		DWORD nRead;
		if(!ReadFile(hComm, &tmpByte, 1, &nRead, 0)){
			std::cerr << "readData(): couldn't read from port" << std::endl;
			return OF_SERIAL_ERROR;
		}
	
		if(nRead == 0){
			return OF_SERIAL_NO_DATA;
		}

	#else

		std::cerr << "Not defined in this platform" << std::endl;
		return OF_SERIAL_ERROR;

	#endif

	return tmpByte;
}


//----------------------------------------------------------------
void ofSerial::flush(bool flushIn, bool flushOut){
	if(!bInited){
		std::cerr << "flush(): serial not inited" << std::endl;
		return;
	}

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )
		int flushType = 0;
		if(flushIn && flushOut) flushType = TCIOFLUSH;
		else if(flushIn) flushType = TCIFLUSH;
		else if(flushOut) flushType = TCOFLUSH;
		else return;

		tcflush(fd, flushType);
	#elif defined( TARGET_WIN32 )

		int flushType = 0;
		if(flushIn && flushOut) flushType = PURGE_TXCLEAR | PURGE_RXCLEAR;
		else if(flushIn) flushType = PURGE_RXCLEAR;
		else if(flushOut) flushType = PURGE_TXCLEAR;
		else return;

		PurgeComm(hComm, flushType);

	#endif
}

void ofSerial::drain(){
	if(!bInited){
		std::cerr << "drain(): serial not inited" << std::endl;
		return;
	}

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )

		tcdrain(fd);

	#endif
}

//-------------------------------------------------------------
int ofSerial::available(){
	if(!bInited){
		std::cerr << "available(): serial not inited" << std::endl;
		return OF_SERIAL_ERROR;
	}

	int numBytes = 0;

	#if defined( TARGET_OSX ) || defined( TARGET_LINUX )

		ioctl(fd, FIONREAD, &numBytes);

	#endif

	#ifdef TARGET_WIN32

		COMSTAT stat;
		DWORD err;
		if(hComm!=INVALID_HANDLE_VALUE){
			if(!ClearCommError(hComm, &err, &stat)){
				numBytes = 0;
			} else {
				numBytes = stat.cbInQue;
			}
		 } else {
			numBytes = 0;
		 }

	#endif

	return numBytes;
}

bool ofSerial::isInitialized() const{
	return bInited;
}
