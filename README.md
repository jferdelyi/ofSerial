# ofSerialLib

A standalone version of openFrameworks communication/serial (https://github.com/openframeworks/openFrameworks).

The library can be compiled for Windows, Linux and macOS as a static or shared library (note that under Windows the static version is required for the shared version, I don't know exactly why. I think it is better to use the static library).

```shell
mkdir build
cd build
cmake .. -DSTATIC=ON -DDEMO=ON
cmake --build .
#./serial <PORT> <BAUD>
./serial cu.usbserial-0001 115200 # macOS
./serial.exe COM3 115200 # Windows
./serial ttyUSB0 115200 # Linux
```

<img width="997" alt="Capture d’écran 2022-11-07 à 22 28 10" src="https://user-images.githubusercontent.com/4105962/200419345-3a8e426f-6787-4173-b977-aebf30730ec6.png">


Parameters
 - -DSTATIC=ON for static, OFF for shared.
 - -DDEMO=ON to build the demo, OFF not
 
 An Arduino source file can be found on the 'example' folder, works on EPS32 using platformio, should work on Arduino with Arduino IDE (remove #include <Arduino.h> to use it on Arduino IDE).
