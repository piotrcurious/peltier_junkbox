// Define pins for thermistors, pumps and LED
#define THERMISTOR_HOT A0
#define THERMISTOR_COLD A1
#define PUMP_HOT 9
#define PUMP_COLD 10
#define LED 13

// Define constants for thermistor calculations
#define R0 10000 // nominal resistance at 25 degrees C
#define B 3950 // B parameter of the thermistor
#define T0 298.15 // reference temperature in Kelvin

// Define constants for PID controller
#define KP 1 // proportional gain
#define KI 0.1 // integral gain
#define KD 0.01 // derivative gain
#define DT 0.1 // sampling time in seconds

// Define constants for Lyapunov stability
#define ALPHA 0.5 // positive constant for Lyapunov function
#define BETA 0.5 // positive constant for Lyapunov function

// Define variables for temperature measurements
float temp_hot; // temperature of the hot side in Kelvin
float temp_cold; // temperature of the cold side in Kelvin
float temp_diff; // temperature difference across the junction in Kelvin
float temp_diff_setpoint; // desired temperature difference in Kelvin

// Define variables for PID controller
float error; // error between temp_diff and temp_diff_setpoint
float error_sum; // sum of errors for integral term
float error_prev; // previous error for derivative term
float output; // output of the PID controller

// Define variables for Lyapunov stability
float V; // Lyapunov function value
float V_dot; // Lyapunov function derivative value

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pins as inputs or outputs
  pinMode(THERMISTOR_HOT, INPUT);
  pinMode(THERMISTOR_COLD, INPUT);
  pinMode(PUMP_HOT, OUTPUT);
  pinMode(PUMP_COLD, OUTPUT);
  pinMode(LED, OUTPUT);

  // Initialize variables
  temp_diff_setpoint = 50; // set the desired temperature difference to 50 K
  error_sum = 0;
  error_prev = 0;
}

void loop() {
  // Read the thermistor values and convert them to Kelvin using Steinhart-Hart equation
  float R_hot = R0 * (1023.0 / analogRead(THERMISTOR_HOT) - 1.0); // resistance of the hot thermistor
  float R_cold = R0 * (1023.0 / analogRead(THERMISTOR_COLD) - 1.0); // resistance of the cold thermistor
  temp_hot = B / log(R_hot / R0) + T0; // temperature of the hot side in Kelvin
  temp_cold = B / log(R_cold / R0) + T0; // temperature of the cold side in Kelvin

  // Calculate the temperature difference across the junction
  temp_diff = temp_hot - temp_cold;

  // Calculate the error between the actual and desired temperature difference
  error = temp_diff_setpoint - temp_diff;

  // Calculate the output of the PID controller using discrete approximation
  output = KP * error + KI * DT * error_sum + KD / DT * (error - error_prev);

  // Update the error sum and previous error for the next iteration
  error_sum += error;
  error_prev = error;

  // Calculate the Lyapunov function value and derivative value using quadratic forms
  V = ALPHA * pow(error,2) + BETA * pow(output,2);
  V_dot = -2 * ALPHA * pow(error,2) -2 * BETA * pow(output,2);

  // Print the values to the serial monitor for debugging purposes
  Serial.print("Temp hot: ");
  Serial.print(temp_hot);
  Serial.print(" K, Temp cold: ");
  Serial.print(temp_cold);
  Serial.print(" K, Temp diff: ");
  Serial.print(temp_diff);
  Serial.print(" K, Error: ");
  Serial.print(error);
  Serial.print(" K, Output: ");
  Serial.print(output);
  
```c  
Serial.print(" %, V: ");
Serial.print(V);
Serial.print(", V_dot: ");
Serial.println(V_dot);

// Check if the output is within the range of -100% to +100%
if (output > +100) {
output = +100;
}
else if (output < -100) {
output = -100;
}

// Check if the Lyapunov function derivative is negative or zero (meaning stable or asymptotically stable)
if (V_dot <=0) {
digitalWrite(LED, HIGH); // turn on LED to indicate stability
}
else {
digitalWrite(LED, LOW); // turn off LED to indicate instability
}

// Control the pumps according to the sign and magnitude of the output 
if (output >0) { 
// positive output means heating mode 
analogWrite(PUMP_HOT, output); // turn on hot pump with PWM value proportional to output 
analogWrite(PUMP_COLD, LOW); // turn off cold pump 
}
else if (output <0) { 
// negative output means cooling mode 
analogWrite(PUMP_HOT, LOW); // turn off hot pump 
analogWrite(PUMP_COLD, -output); // turn on cold pump with PWM value proportional to absolute value of output 
}
else { 
// zero output means no action 
analogWrite(PUMP_HOT, LOW); // turn off hot pump 
analogWrite(PUMP_COLD, LOW); // turn off cold pump 
}

// Wait for sampling time before next iteration 
delay(DT*1000); 
}

//Source: Conversation with Bing, 5/20/2023
//(1) How to Set Up a Peltier Module. : 6 Steps - Instructables. https://www.instructables.com/How-to-Set-Up-a-Peltier-Module/.
//(2) Peltier cell controller - General Electronics - Arduino Forum. https://forum.arduino.cc/t/peltier-cell-controller/639850.
//(3) Learn how to control a Peltier unit with Arduino | Freetronics. https://www.freetronics.com.au/blogs/news/17776585-learn-how-to-control-a-peltier-unit-with-arduino.
//(4) 4 Lyapunov Stability Theory - Caltech Computing. https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/mls93-lyap.pdf.
//(5) Lecture 12 Basic Lyapunov theory - Stanford University. https://www-leland.stanford.edu/class/ee363/lectures/lyap.pdf.
//(6) Lyapunov Stability - University of Washington. https://sites.math.washington.edu/~burke/crs/555/555_notes/lyapunov_stability.pdf.
