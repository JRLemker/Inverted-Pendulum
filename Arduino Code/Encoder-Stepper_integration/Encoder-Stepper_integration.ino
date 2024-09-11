#include <Encoder.h>
#include <AccelStepper.h>

// Define the pins for the encoder
Encoder myEncoder(2, 3);  // Encoder connected to pins 2 and 3

// Create an instance of AccelStepper for the stepper driver
AccelStepper stepper(AccelStepper::DRIVER, 5, 4);  // Pin 5: Step, Pin 4: Direction

long oldEncoderPosition = -999;  // Store last encoder position

void setup() {
  Serial.begin(9600);

  // Set up the stepper motor parameters
  stepper.setMaxSpeed(1000);     // Max speed in steps per second
  stepper.setAcceleration(500);  // Acceleration in steps per second squared

  // Print initial encoder position
  Serial.print("Initial Encoder Position: ");
  Serial.println(myEncoder.read());
}

void loop() {
  // Poll the encoder for its current position
  long newEncoderPosition = myEncoder.read();

  // Only update the stepper motor if the encoder position has changed
  if (newEncoderPosition != oldEncoderPosition) {
    oldEncoderPosition = newEncoderPosition;
    
    Serial.print("New Encoder Position: ");
    Serial.println(newEncoderPosition);

    // Move the stepper motor to match the encoder position
    stepper.moveTo(newEncoderPosition);  // Command the stepper to move to the encoder position
  }

  // Non-blocking call to move the stepper
  stepper.run();
}
