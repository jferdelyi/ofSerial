// This is an example of the standalone version of openFrameworks communication/serial
// Distributerd under the MIT License.
// Copyright (c) 2022 - Jean-François Erdelyi

#include "ofSerial.h"

#include <iostream>
#include <signal.h>
#include <chrono>
#include <thread>

using namespace std;

static bool isRunning = true;

void stopHandler(int) {
	isRunning = false;
}

// ofSerial standalone example
int main(int argc, char* argv[]) {
	// Check and get params 
	if (argc < 3) {
		cout << "Usage: serial <port> <baudrate>" << endl;
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
	string bytesToProcess;
	l_serial.flush(true, true);
	cout << endl;
	while (isRunning) {
		// Get and send data
		string input;
		cout << "SEND DATA: ";
		getline(cin, input); 
		cout << endl;
		l_serial.writeBytes(input);

		// Wait the answer
		while(!l_serial.available() && isRunning) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}
		if (!isRunning) {
			break;
		}

		// Check if there is data to read
		int bytesToRead = l_serial.available();
	
		// Print number of bytes 
		cout << "RECEIVED " << dec << bytesToRead << " BYTES" << endl;
		l_serial.readBytes(bytesToProcess, bytesToRead);

		// Print hexa data
		cout << "0x" << hex << uppercase;
		for (int i = 0; i < bytesToRead; ++i) {
			cout << ((ushort*)bytesToProcess.c_str())[i];
		}

		// Print string data 
		cout << dec << " (" << bytesToProcess << ")" << endl << endl;
	}

	// Close and return
	cout <<  "FINISHED" << endl;
	l_serial.close();
	return EXIT_SUCCESS;
}
