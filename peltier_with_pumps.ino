// Define pins for thermistors, pumps and peltier module
#define THERMISTOR_HOT A0
#define THERMISTOR_COLD A1
#define PUMP_HOT 3
#define PUMP_COLD 5
#define PELTIER 6

// Define constants for thermistor calibration and conversion
#define R0 10000 // nominal resistance at 25 degrees C
#define B 3950 // B parameter of the thermistor
#define T0 298.15 // reference temperature in Kelvin
#define R_SERIES 10000 // value of the series resistor

// Define constants for PID control
#define KP 1 // proportional gain
#define KI 0.1 // integral gain
#define KD 0.01 // derivative gain
#define DT 0.1 // sampling time in seconds

// Define constants for Lyapunov stability
#define EPSILON 0.01 // small positive constant for Lyapunov function
#define ALPHA 0.5 // positive constant for Lyapunov derivative

// Define variables for temperature measurement and control
float temp_hot; // temperature of the hot side in Celsius
float temp_cold; // temperature of the cold side in Celsius
float temp_diff; // temperature difference across the junction in Celsius
float temp_diff_setpoint; // desired temperature difference in Celsius
float error; // error between temp_diff and temp_diff_setpoint
float error_sum; // cumulative error for integral term
float error_prev; // previous error for derivative term
float output; // output of the PID controller

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pins as outputs or inputs
  pinMode(THERMISTOR_HOT, INPUT);
  pinMode(THERMISTOR_COLD, INPUT);
  pinMode(PUMP_HOT, OUTPUT);
  pinMode(PUMP_COLD, OUTPUT);
  pinMode(PELTIER, OUTPUT);

  // Set initial values for variables
  temp_hot = 0;
  temp_cold = 0;
  temp_diff = 0;
  temp_diff_setpoint = 10; // you can change this value as you wish
  error = 0;
  error_sum = 0;
  error_prev = 0;
  output = 0;
}

void loop() {
  
  // Read thermistor values and convert them to Celsius
  temp_hot = thermistorToCelsius(analogRead(THERMISTOR_HOT));
  temp_cold = thermistorToCelsius(analogRead(THERMISTOR_COLD));

  // Calculate temperature difference across the junction
  temp_diff = temp_hot - temp_cold;

  // Calculate error between temp_diff and temp_diff_setpoint
  error = temp_diff_setpoint - temp_diff;

  // Update error_sum for integral term
  error_sum += error * DT;

  // Calculate output of the PID controller using Lyapunov stability criterion
  output = KP * error + KI * error_sum + KD * (error - error_prev) / DT;
  
   if (output > EPSILON) {
    output = sqrt(2 * ALPHA * output); // ensure positive Lyapunov derivative for positive output
   } else if (output < -EPSILON) {
    output = -sqrt(2 * ALPHA * (-output)); // ensure negative Lyapunov derivative for negative output
   } else {
    output = 0; // ensure zero Lyapunov derivative for zero output
   }

   // Update error_prev for derivative term
   error_prev = error;

   // Map output to PWM duty cycle between 0 and 255
   output = map(output, -255, 255, -255, 255);

   // Control pumps and peltier module according to output sign and magnitude
   
   if (output > EPSILON) {
    analogWrite(PUMP_HOT, output); // turn on hot pump with output duty cycle 
    analogWrite(PUMP_COLD, LOW); // turn off cold pump 
    digitalWrite(PELTIER, HIGH); // turn on peltier module in one direction 
   } else if (output < -EPSILON) {
    analogWrite(PUMP_HOT, LOW); // turn off hot pump 
    analogWrite(PUMP_COLD, -output); // turn on cold pump with -output duty cycle 
    digitalWrite(PELTIER, LOW); // turn on peltier module in opposite direction 
   } else {
    analogWrite(PUMP_HOT, LOW); // turn off hot pump 
    analogWrite(PUMP_COLD, LOW); // turn off cold pump 
    digitalWrite(PELTIER, LOW); // turn off peltier module 
   }

   // Print values for debugging purposes
   
   Serial.print("Temp hot: ");
   Serial.print(temp_hot);
   Serial.print(" C\t");
   
   Serial.print("Temp cold: ");
   Serial.print(temp_cold);
   Serial.print(" C\t");
   
   Serial.print("Temp diff: ");
   Serial.print(temp_diff);
   Serial.print(" C\t");
   
   Serial.print("Error: ");
   Serial.print(error);
   Serial.print(" C\t");
   
   Serial.print("Output: ");
   Serial.println(output);

   delay(DT *1000); // wait for sampling time
  
}

// Function to convert thermistor resistance to Celsius using Steinhart-Hart equation

float thermistorToCelsius(int adc_value) {
  
 float resistance; // resistance of the thermistor in ohms
 float temperature; // temperature of the thermistor in Kelvin
 
 resistance = R_SERIES / ((1023.0 / adc_value) -1); // calculate resistance from ADC value
 
 temperature = log(resistance / R0) / B + (1 / T0); // calculate inverse temperature from resistance
 
 temperature = (1 / temperature) -273.15; // convert Kelvin to Celsius
 
 return temperature;
}

//Source: Conversation with Bing, 5/20/2023
//(1) How to Set Up a Peltier Module. : 6 Steps - Instructables. https://www.instructables.com/How-to-Set-Up-a-Peltier-Module/.
//(2) PWM control of Peltier with Arduino+Thermistor [duplicate]. https://electronics.stackexchange.com/questions/124141/pwm-control-of-peltier-with-arduinothermistor.
//(3) Peltier cell controller - General Electronics - Arduino Forum. https://forum.arduino.cc/t/peltier-cell-controller/639850.
//(4) Lyapunov stability - Wikipedia. https://en.wikipedia.org/wiki/Lyapunov_stability.
//(5) 4 Lyapunov Stability Theory - Caltech Computing. https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/mls93-lyap.pdf.
//(6) Lyapunov’s Stability Theory | SpringerLink. https://link.springer.com/referenceworkentry/10.1007/978-1-4471-5102-9_77-1.
