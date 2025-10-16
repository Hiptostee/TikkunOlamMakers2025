// Define the pins connected to the TB6600 driver
const int stepPin = 2; // Connect to PUL+ (Step/Pulse)
const int dirPin = 3;  // Connect to DIR+ (Direction)
const int enPin = 4;   // Connect to ENA+ (Enable)

// Define the pin for the Joystick X-axis (A0)
const int joystickX = A0;

// Variables for motor control
int stepSpeed = 0;   // Controls the delay between steps (speed)
int joystickValue = 0; // Value read from the joystick (0 to 1023)
const int centerThreshold = 50; // Tolerance around the center (512)

void setup() {
  // Set the driver pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  // Set the Enable pin LOW to turn the driver ON and engage the motor
  digitalWrite(enPin, LOW);

  // Initialize Serial communication for debugging
  Serial.begin(9600);
  Serial.println("Stepper Motor Controller Ready.");
}

void loop() {
  // 1. Read the joystick position
  joystickValue = analogRead(joystickX);

  // 2. Determine Direction and Speed
  
  // Calculate the difference from the center (512)
  int deviation = joystickValue - 512;
  
  // Check if the joystick is near the center (Motor OFF/STOP)
  if (abs(deviation) < centerThreshold) {
    // Motor is stopped
    stepSpeed = 0;
    // You can optionally disable the motor here to save power, but it loses holding torque:
    // digitalWrite(enPin, HIGH); 
  } else {
    // Motor is moving
    digitalWrite(enPin, LOW);

    // a. Determine Direction
    if (deviation > 0) {
      // Joystick pushed right (Clockwise)
      digitalWrite(dirPin, HIGH);
    } else {
      // Joystick pushed left (Counter-Clockwise)
      digitalWrite(dirPin, LOW);
    }

    // b. Determine Speed
    // Map the absolute deviation (50 to 512) to a delay time (inverse of speed).
    // The delay range below makes the motor run from slow (2000 microsecond delay) to fast (200 microsecond delay).
    // Note: Smaller stepSpeed number means FASTER motor speed.
    int absDeviation = abs(deviation);
    stepSpeed = map(absDeviation, centerThreshold, 512, 2000, 100);

    // Keep stepSpeed from being too low (or it might stop working)
    if (stepSpeed < 200) {
      stepSpeed = 200; 
    }
  }

  // 3. Drive the Motor
  if (stepSpeed > 0) {
    // Send a pulse (HIGH/LOW) to the Step pin
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepSpeed); // Time HIGH
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepSpeed); // Time LOW (Total period is 2 * stepSpeed)
  }
}