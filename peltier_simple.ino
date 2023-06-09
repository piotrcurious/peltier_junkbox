// Include the PID library
#include <PID_v1.h>

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
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  // Convert to temperature using Steinhart-Hart equation
  double temp;
  temp = average / THERMISTOR_NOMINAL;     // (R/Ro)
  temp = log(temp);                  // ln(R/Ro)
  temp /= BCOEFFICIENT;           // 1/B * ln(R/Ro)
  temp += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  temp = 1.0 / temp;                 // Invert
  temp -= 273.15;                 // Convert to Celsius

  return temp;
}

//Source: Conversation with Bing, 5/20/2023
//(1) How to Set Up a Peltier Module. : 6 Steps - Instructables. https://www.instructables.com/How-to-Set-Up-a-Peltier-Module/.
//(2) PID temperature controller with Arduino using a NTC thermistor and a .... https://electronics.stackexchange.com/questions/355664/pid-temperature-controller-with-arduino-using-a-ntc-thermistor-and-a-peltier-cel.
//(3) Learn how to control a Peltier unit with Arduino | Freetronics. https://www.freetronics.com.au/blogs/news/17776585-learn-how-to-control-a-peltier-unit-with-arduino.
//(4) Peltier Element Efficiency - Meerstetter. https://www.meerstetter.ch/customer-center/compendium/71-peltier-element-efficiency.
//(5) Efficiency in thermoelectric generators based on Peltier cells. https://www.sciencedirect.com/science/article/pii/S2352484721007022.
//(6) Analysis of the Appropriateness of the Use of Peltier Cells as Energy .... https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934186/.
//(7) Highly efficient electricity generation with Peltier Module. https://www.researchgate.net/publication/305676982_Highly_efficient_electricity_generation_with_Peltier_Module.
