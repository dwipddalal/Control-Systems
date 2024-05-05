#include <PID_v1.h>
#include <time.h> 
// Motor control pins
const int stepPin = 5;
const int dirPin = 4; // Also used for the encoder
const int enPin = 8;

// Encoder pins - cannot be changed because they are interrupt pins
int encoderPin1 = 2;
int encoderPin2 = 3;
clock_t previousTime; // Time at the last PID calculation

volatile double encoderValue = 0;
int lastEncoded = 0;
long verticalPosition = 0; // Store the calibrated vertical position

// PID variables
double Setpoint, Input, Output ,curr_delay , prev_delay;
double Kp = 31, Ki = 0.05, Kd = 0.0001; // Initial PID parameters, adjust these based on your system

double integral = 0.0; // Integral sum
double previousError = 0.0; // Last error value

int total_steps = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int steps = 800;
void setup() {
  Serial.begin(9600);
  
  // Initialize motor control pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW); // Enable the motor driver

  // Initialize encoder pins
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH); // Turn pull-up resistor on
  digitalWrite(encoderPin2, HIGH); // Turn pull-up resistor on
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);

  // PID initialization
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255); // Adjust based on your motor's characteristics

  Setpoint = 0; // This will be dynamically adjusted during calibration
  prev_delay = 150; // default value
  curr_delay = 150; // default
  while (!Serial.available() || Serial.read() != 'c') {
  delay(100); // Small delay to avoid flooding; adjust as needed
  }
  calibrateVerticalPosition(); // Perform calibration
}

void loop() {
  InitializePID();

   if (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Read the incoming character
    if (receivedChar == 'c') {
      calibrateVerticalPosition(); // Calibrate vertical position when 'c' is received
    }
  }

  Input = encoderValue - verticalPosition; // Adjust input based on calibrated vertical position

  // myPID.Compute(); // Compute PID output
  Output = ComputePID(verticalPosition, encoderValue);
  int Output_mag = map(abs(Output), 0, 255, 0, 255);
  prev_delay = curr_delay;
  curr_delay = calculateStepDelay(abs(Output_mag));
  digitalWrite(dirPin, Output < 0 ? HIGH : LOW);  
Serial.println(curr_delay);
  if(abs(Input) > 0){
  if(curr_delay < 90){
    int speed = curr_delay * 4;
    steps = 3200;
    if(abs(curr_delay - prev_delay) > 10) {
    for(int j = 0 ; j < 2 ; j++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(speed / 2); // Half the delay for the high signal
        digitalWrite(stepPin, LOW);
        delayMicroseconds(speed / 2); // Half the delay for the low signal
        speed / 2;
  }
    }
  if(curr_delay < 60){
    steps = 5000;
  }
  else if(curr_delay < 80){
    steps = 2200;  
  }  
  }
  else{ steps = 200;
}
  //steps = map(curr_delay, 20 , 125, 6400, 800);
  //steps = 2000;
  for (int stepCount = 0; stepCount < steps; stepCount++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(curr_delay / 2); // Half the delay for the high signal
    digitalWrite(stepPin, LOW);
    delayMicroseconds(curr_delay / 2); // Half the delay for the low signal
//}
  }
  }
  Serial.print("Input: ");
  Serial.print(Input);
  Serial.print(" | Output: ");
  Serial.print(Output);
  Serial.print("  |  Current Delay: ");
  Serial.println(curr_delay);
}

void controlMotor(double output) {  
}

unsigned long calculateStepDelay(double outputMagnitude) {
  // We mapped the PID output magnitude to an appropriate step delay
  // For demonstration, assuming outputMagnitude is directly proportional to desired speed
 // Lower bound (slow speed) might be 1000 microseconds, upper bound (fast speed) 100 microseconds
  
  unsigned long maxDelay = 150; // Slowest speed (largest delay between steps)
  unsigned long minDelay = 50 ;  // Fastest speed (smallest delay between steps)
  
  // Ensuring outputMagnitude is within a reasonable range to avoid extremely fast or slow speeds
  outputMagnitude = constrain(outputMagnitude, 0, 255); // Assuming PID output is scaled to 0-255
  // Mapping the output magnitude to a delay value
  return map(outputMagnitude, 0, 255, maxDelay, minDelay);
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1); // MSB = most significant bit
  int LSB = digitalRead(encoderPin2); // LSB = least significant bit
  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value
 if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue += 0.225;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue -= 0.225;

  lastEncoded = encoded; // store this value for next time
}

void calibrateVerticalPosition() {
  verticalPosition = encoderValue; // Set current position as vertical
  Serial.println("Calibrated Vertical Position: " + String(verticalPosition));
}
double ComputePID(double setpoint, double actualPosition) {
    double error = setpoint - actualPosition;
    long currentTime = micros();
    double dt = (double)(currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;
    // Proportional term
    double Pout = Kp * error;
    
    // Derivative term
    double derivative = (error - previousError) / dt;
    double Dout = Kd * derivative;
    // Total output
    double output = Pout + Dout;
    previousError = error;
    return output;
}
void InitializePID() {
    previousTime = micros(); // Initialize previousTime with the current system time
}
