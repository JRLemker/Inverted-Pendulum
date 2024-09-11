#include <Encoder.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// Encoder and stepper motor pins
Encoder myEncoder(2, 3);
AccelStepper stepper(AccelStepper::DRIVER, 5, 4);

// PID parameters
double setpoint = 0;      // Target position (e.g., upright pendulum at 0 degrees)
double input;             // Encoder reading (current pendulum position)
double output;            // Output from PID (used to move the stepper motor)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;  // PID gains

// PID controller
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder and stepper motor variables
long oldEncoderPosition = -999;

void setup() {
  Serial.begin(115200);

  // Setup stepper motor parameters
  stepper.setMaxSpeed(1000);     // Max speed in steps per second
  stepper.setAcceleration(500);  // Acceleration in steps per second squared

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);      // Set PID to automatic mode
  myPID.SetOutputLimits(-1000, 1000);  // Limit PID output to match stepper speed
}

void loop() {
  // Read encoder position (the "input" for PID)
  long newEncoderPosition = myEncoder.read();
  input = newEncoderPosition;   // Current pendulum position (from the encoder)

  // Compute the PID output (control signal)
  myPID.Compute();

  // Use the PID output to move the stepper motor
  stepper.moveTo(stepper.currentPosition() + output);  // Adjust stepper position
  stepper.run();
  
  // Optionally print values for debugging
  Serial.print("Encoder Position: ");
  Serial.print(newEncoderPosition);
  Serial.print(" | PID Output: ");
  Serial.println(output);
}
