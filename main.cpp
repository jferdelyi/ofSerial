// This is an example of the standalone version of openFrameworks communication/serial
// Distributerd under the MIT License.
// Copyright (c) 2022 - Jean-François Erdelyi

#include "ofSerial.h"

#include <iostream>
#include <signal.h>

static bool isRunning = true;

void stopHandler(int) {
	isRunning = false;
}

// ofSerial standalone example
int main(int argc, char* argv[]) {
	// Check and get params 
	if (argc < 3) {
		std::cout << "Usage: serial <port> <baudrate>" << std::endl;
		return EXIT_FAILURE;
	}
	char* port = argv[1];
	int baudrate = atoi(argv[2]);

	// Catch ctrl+c signal
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = stopHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// Create and connect serial
	ofSerial l_serial;
	bool l_connected = l_serial.setup(port, baudrate);

	// While is running
	std::string bytesToProcess;
	l_serial.writeBytes('0');
	while (isRunning) {
		// Check if there is data to read
		int bytesToRead = l_serial.available();
		if (bytesToRead > 0) {

			// Print number of bytes 
			std::cout << "RECEIVED " << std::dec << bytesToRead << " BYTES" << std::endl;
			l_serial.readBytes(bytesToProcess, bytesToRead);

			// Print hexa data
			std::cout << "0x" << std::hex << std::uppercase;
			for (int i = 0; i < bytesToRead; ++i) {
				std::cout << ((ushort*)bytesToProcess.c_str())[i];
			}

			// Print string data 
			std::cout << std::dec << " (" << bytesToProcess << ")" << std::endl;

			// Write TEST to serial
			l_serial.writeBytes("1");
		}
	}

	// Close and return
	std::cout <<  "FINISHED" << std::endl;
	l_serial.close();
	return EXIT_SUCCESS;
}
