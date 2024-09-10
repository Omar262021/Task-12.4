// Define PID parameters
double Kp = 2, Ki = 5, Kd = 1;
double setpoint = 100; // Desired speed
double input;         // Current speed (to be measured)
double output;        // PWM output to the motor

// Define exponential smoothing variables
double alpha = 0.1; // Smoothing factor
double previousSmoothedOutput = 0;

// PID control function
double pidCompute(double input, double setpoint, double Kp, double Ki, double Kd) {
    static double integral = 0;
    static double prevError = 0;
    
    double error = setpoint - input;
    integral += error;
    double derivative = error - prevError;
    
    double output = Kp * error + Ki * integral + Kd * derivative;
    
    prevError = error;
    return output;
}

// Exponential smoothing function
double exponentialSmoothing(double currentValue, double previousSmoothedValue, double alpha) {
    return alpha * currentValue + (1 - alpha) * previousSmoothedValue;
}

void setup() {
    Serial.begin(9600);
    pinMode(9, OUTPUT); // Assume motor control is connected to PWM pin 9
}

void loop() {
    // Measure the current speed
    input = analogRead(A0); // Example: reading from an analog sensor
  
    // Compute PID output
    double pidOutput = pidCompute(input, setpoint, Kp, Ki, Kd);
  
    // Apply exponential smoothing
    double smoothedOutput = exponentialSmoothing(pidOutput, previousSmoothedOutput, alpha);
  
    // Apply the smoothed output to the motor
    analogWrite(9, smoothedOutput);
  
    // Update the previous smoothed value
    previousSmoothedOutput = smoothedOutput;
  
    // Print values for debugging
    Serial.print("Input: ");
    Serial.print(input);
    Serial.print(" Smoothed Output: ");
    Serial.println(smoothedOutput);
  
    delay(100); // Delay of 100 milliseconds
}
