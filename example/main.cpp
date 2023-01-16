// This is an example of the standalone version of openFrameworks communication/serial
// Distributerd under the MIT License.
// Copyright (c) 2022 - Jean-François Erdelyi

#include "ofSerial.h"

#include <iostream>
#include <thread>

using namespace std;

// ofSerial standalone example
int main(int argc, char* argv[]) {

	// Check and get params 
	ofSerial l_serial;
	if (argc < 3) {
		cout << "Usage: serial <port> <baudrate>" << endl << endl;
		cout << "List of serial ports detected:" << endl;
		for (auto& l_port : l_serial.getDeviceList()) {
			cout << "\t- " << l_port.getDeviceName() << endl;
		}
		return EXIT_FAILURE;
	}
	char* l_port = argv[1];
	int l_baudrate = atoi(argv[2]);

	// Setup and connect serial
	bool l_connected = l_serial.setup(l_port, l_baudrate);
	if (l_connected) {
		cout << "CONNECTED" << endl;
	} else {
		cerr << "NOT CONNECTED" << endl;
		return EXIT_FAILURE;
	}

	// While is running
	string l_bytes_to_process;
	cout << endl;
	while (true) {

		// Get and send data
		string l_input;
		cout << "SEND DATA (EXIT to quit): ";
		getline(cin, l_input); 
		if (l_input == "") {
			continue;
		} else if (l_input == "EXIT") {
			break;
		}
		l_serial.flush(true, true);
		l_serial.writeData(l_input);

		// Wait the answer
		while(!l_serial.available()) {
			this_thread::sleep_for(chrono::milliseconds(100));
		}

		// Check if there is data to read
		int l_bytes_to_read = l_serial.available();
	
		// Print number of bytes 
		cout << endl << "RECEIVED " << dec << l_bytes_to_read << " BYTES" << endl;
		int l_bytes_read = l_serial.readData(l_bytes_to_process, l_bytes_to_read);

		// Print hexa data
		cout << hex << uppercase;
		const char* l_c_str = l_bytes_to_process.c_str();
		for (int i = 0; i < l_bytes_read; ++i) {
			cout << static_cast<unsigned short>(l_c_str[i] & 0x00FF) << " ";
		}

		// Print string data 
		cout << dec << "-> " << l_bytes_to_process << endl << endl;
	}

	// Close and return
	l_serial.close();
	cout << "CLOSED" << endl;
	return EXIT_SUCCESS;
}
