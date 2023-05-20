// Include the PID and Float64 libraries
#include <PID_v1.h>
// https://github.com/br3ttb/Arduino-PID-Library 
#include <Float64.h>
// https://github.com/mmoller2k/Float64
// Define thermistor pins
#define THERMISTOR_HOT A0
#define THERMISTOR_COLD A1

// Define Peltier pin
#define PELTIER 3

// Define thermistor parameters
#define THERMISTOR_NOMINAL 10000 // Resistance at 25 degrees C
#define TEMPERATURE_NOMINAL 25 // Temperature for nominal resistance (almost always 25 C)
#define NUM_SAMPLES 5 // How many samples to take for averaging
#define BCOEFFICIENT 3950 // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000 // The value of the series resistor

// Define PID parameters
#define Kp 2 // Proportional gain
#define Ki 5 // Integral gain
#define Kd 1 // Derivative gain
#define DT 10 // Desired temperature difference

// Define PID variables
double InputHot; // Measured temperature of hot side
double InputCold; // Measured temperature of cold side
double Setpoint; // Desired temperature difference
double Output; // PWM duty cycle for Peltier cell

// Create PID object
PID myPID(&InputCold, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Initialize variables
void setup() {
  // Set Peltier pin as output
  pinMode(PELTIER, OUTPUT);

  // Set desired temperature difference
  Setpoint = DT;

  // Initialize PID object
  myPID.SetMode(AUTOMATIC); // Turn on PID
  myPID.SetOutputLimits(0, 255); // Limit output to PWM range

  // Initialize Float64 library
  Float64.begin();
}

// Main loop
void loop() {
  // Read thermistor values and convert to Celsius
  InputHot = readThermistor(THERMISTOR_HOT);
  InputCold = readThermistor(THERMISTOR_COLD);

  // Compute PID output
  myPID.Compute();

  // Write PWM value to Peltier pin
  analogWrite(PELTIER, Output);
}

// Function to read thermistor value and convert to Celsius
double readThermistor(int pin) {
  double average = 0;

  // Take NUM_SAMPLES samples and average them
  for (int i = 0; i < NUM_SAMPLES; i++) {
    average += analogRead(pin);
    delay(10);
  }
  average /= NUM_SAMPLES;

  // Convert to resistance
  average = Float64(1023) / average - Float64(1);
  average = Float64(SERIESRESISTOR) / average;

  // Convert to temperature using Steinhart-Hart equation
  double temp;
  temp = average / Float64(THERMISTOR_NOMINAL);     // (R/Ro)
  temp = Float64::log(temp);                  // ln(R/Ro)
  temp /= Float64(BCOEFFICIENT);           // 1/B * ln(R/Ro)
  temp += Float64(1.0) / (Float64(TEMPERATURE_NOMINAL) + Float64(273.15)); // + (1/To)
  temp = Float64::pow(temp, -1);                 // Invert
  temp -= Float64(273.15);                 // Convert to Celsius

  return temp;
}
