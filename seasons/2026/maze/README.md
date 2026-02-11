# Introduction
This document will walk you through the installation of the recommended software for the competition and then give you all the necessary tools to utilize the provided robot to its maximal potential. This is not meant as a supplement to reading the documentation but rather as a quick glossary view of the involved parts and a compilation of short code snippets to test if everything is working. Note that most of the wiring is already done on the robot you've recieved but is still included in the instructions for completeness.
# Required Software
The recommended software to be installed is the Arduino IDE (integrated development environment), and the specific package that allows _compiling for_ and _uploading to_ the ESP32 SoC that the Romeo Mini is based on. Instead of wasting your time by writing instructions that won't work for everyone, we will treat this as an exercise in finding the correct information online. As pointers, you can find the Arduino IDE at [Arduino Software](https://www.arduino.cc/en/software/) and the specific instructions for uploading to the Romeo Mini can be found at [The Romeo Mini Product Wiki](https://wiki.dfrobot.com/SKU_DFR1063_Romeo_mini_Control_Board_ESP32_C3).
# Programming
This section will walk you through everything from checking that the board works to having working sensors and spinning motors. If you think you can handle this on your own, there is no necessary information in the following parts of this document but you might want to skim it anyway to get ideas.
## Hello, World!
To get started with any programming, it's useful to perform a so-called _Hello, World!_ program. To do this, we will utilize the USB communication capabilities of the Romeo Mini using the following code.
```c
// The setup function will run once every time after your board gets power or is reset.
void setup() {
  // Here we say that we want to initialize serial communication with a baudrate of 9600 Bd.
  // The baudrate speciefies how fast the communication is in pulses per second.
  Serial.begin(9600);
}

// The loop function will be run over and over until the board loses power.
void loop() {
  // Send the string "Hello, World!" over serial.
  Serial.println("Hello, World!");

  // Because it will run as fast as possible, it is useful to slow down the execution as to not flood the
  // serial monitor. We can do this using the delay function which takes in the number of milliseconds it
  // should wait.
  delay(1000); // 1 s
}
```
Uploading this code, using the right-arrow in the top left of the IDE, you should see it successfully upload the code but then, to your dismay, nothing. The fix is simple, we need to open the serial monitor provided by the Arduino IDE by going to the top bar and clicking `Tools > Serial Monitor` or alternatively `Ctrl + Shift + M`. If the upload was successful and the board is connected, you should now see a new line with the text `Hello, World!` approximately once per second. If this does not work, try to find a solution online but in case you get stuck and can't find what's wrong, you may also contact us in the Robot Group at Umeå University through the Robot Competition Discord server.
## Hello, Sensor!
The next component to check is the line sensors whose specification can be found on [The Line Sensor Product Wiki](https://wiki.dfrobot.com/Line_Tracking_Sensor_for_Arduino_V4_SKU_SEN0017). Note that this is the wiki for V4 while you've been provided with V6. They are extremely similar but it might be worth to mention in case you find discrepancies. To test a sensor, we just upload the following code with the sensor connected to pin 3, which should be the leftmost sensor in the original configuration as provided by us. Also note that analog pins can be used as digital pins but not always the other way around, and the sensor is rated for both 3.3V and 5V so it doesn't matter which pins you choose.
```c
// The sensor is connected to pin 3.
int sensorPin = 3;

void setup() {
  Serial.begin(9600);
  // You don't need to do this since it's the default but to be explicit, we'll write it anyway.
  pinMode(sensorPin, INPUT);
}

void loop() {
  // Now we just read the value returned by the sensor and print it to the serial monitor.
  int line = digitalRead();
  Serial.println(line);
  delay(10); // 10 ms
}
```
You should now see that the serial monitor displays zeros and ones, and that the LED on the sensor turns on or off when it's aimed at black or white. To handle five sensors efficiently, I recommend storing the pins in an array and using for-loops and indexing to access each one. The pins which the sensors are connected to on the provided robot should be 3, 4, 5, 6, and 7, in order from left to right if the sensors are considered to be the front of the robot.
## Hello, Motor!
The specification for the motors can be found on [The Motor Product Wiki](https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ01_SKU__FIT0450). Running the motors is fairly straight forward since the motor drivers are built into the microcontroller. All you need to do is connect the positive and negative of the motors to the motor outputs on the board and run the following code. The pins 0, 1, 2, and 10 are wired on the microcontroller so no connection is necessary.
```c
// Motor 1 pins
int PWM1 = 0;
int DIR1 = 1;
// Motor 2 pins
int PWM2 = 2;
int DIR2 = 10;

void setup() {
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
}

int pwm = 64; // [0-255]
bool rev = false; // Reverse direction

void loop() {
  rev = !rev;
  analogWrite(PWM1, pwm);
  digitalWrite(DIR1, rev);
  analogWrite(PWM2, pwm);
  digitalWrite(DIR2, rev);
  delay(1000);
}
```

## Hello, Encoder!
The motors are equipped with a so-called encoder, which allows us to track exactly how much it has rotated. This is useful for estimating how much the robot has moved and/or turned. This is going to be the most involved code example so far and might be hard to understand at first glance since it relies on so-called interrupts and atomic operations. If you want to know how it works, you are more than welcome to do some research but you can also treat this as a black box if you prefer and focus on the line-following and maze-solving part.
```c
#include <atomic>

// Note that on your robot, the left and right motors may be swapped.
// If so, just change leftA to 21 and rightA to 20.
int leftA = 20;
int rightA = 21;
std::atomic<int> leftAtomicPulses{0};
std::atomic<int> rightAtomicPulses{0};

// Function that calls on left motor encoder pin A rise
void IRAM_ATTR leftEncoderISR() {
  leftAtomicPulses.fetch_add(1, std::memory_order_relaxed);
}

// Function that calls on right motor encoder pin A rise
void IRAM_ATTR rightEncoderISR() {
  rightAtomicPulses.fetch_add(1, std::memory_order_relaxed);
}

void setup() {
  Serial.begin(9600);
  pinMode(leftA, INPUT_PULLUP);
  pinMode(rightA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightA), rightEncoderISR, RISING);
}

void loop() {
  // Get the left and right counts, swaping them with zero to reset them
  int l = leftAtomicPulses.exchange(0, std::memory_order_relaxed);
  int r = rightAtomicPulses.exchange(0, std::memory_order_relaxed);
  Serial.print(l);
  Serial.print(", ");
  Serial.println(r);
  delay(10);
}
```
Running this on its own will not spin the motor but if you very gently turn one of the wheels, you should see it respond with encoder counts in the serial monitor. Note that we have not connected pin B which would allow us to sense which direction it's spinning so we only get absolute pulse counts. The reason for this is that if we include the B pins we will run out of connections on the board and won't have space for the next component (the ToF sensor). Therefore we will assume that you can guess which way the wheel is turning based on what you set the direction to. Note that the specification says that the motor provides 960 pulses per rotation. This example should be reliable enough for our use-case but to be robust at even higher speeds, one might want to utilize a hardware pulse counter (PCMT) instead of programmed interrupts.

## Hello, ToF!
For the final component, the time-of-flight (ToF) sensor, whose specification you can find at [The ToF Sensor Product Wiki](https://wiki.dfrobot.com/Fermion_TMF8701_ToF_Distance_Ranging_Sensor_10_600mm_SKU_SEN0429), you will need to download the library `TMF8x01` from the Arduino IDE package manager. When that is installed, it should be as simple as connecting SDA to pin 8, SCL to pin 9, and power, before running the following code. Note that we never specify 8 or 9 in the code because those are the default pins for I2C communication on the Romeo Mini.
```c
#include "DFRobot_TMF8x01.h"

DFRobot_TMF8701 tof(-1, -1);

void setup() {
  Serial.begin(9600);
  
  while (tof.begin() != 0) {
    Serial.println("Retrying...");
    delay(1000);
  }

  // Note that we're running without calibration.
  // If you wish to calibrate, you may do so using the instructions on the product wiki.
  tof.startMeasurement(tof.eModeNoCalib, tof.eCOMBINE);
}

void loop() {
  if (tof.isDataReady()) {
    Serial.println(tof.getDistance_mm());
  }
}
```
# What now?
At this point you should have all the necessary tools to know where the line is using the line sensors, know where the robot is using the motor encoders, know where the finish is using the ToF sensor, and last but not least be able to move the robot using the motors themselves. These are all very simple examples that you'll of course need to build on top of and combine to make anything useful. Now it's up to you to program the robot to solve a the maze described in the competition-specific document. If you feel lost and don't know where to start, check the [strategy directory](strategy/). Good luck!
