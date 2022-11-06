#include <Arduino.h>

void setup() {
	Serial.begin(115200);
}

void loop() {
	if (Serial.available()) {
		String data = Serial.readString();
		Serial.print(data);
	}
}
