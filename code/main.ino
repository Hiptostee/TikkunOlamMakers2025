/**
 * @file tb6600_stepper_control.ino
 * @brief Controls a stepper motor with a joystick and features an adjustable zero position.
 *
 * This code allows for precise control of a stepper motor using a joystick.
 * It has two modes of operation, toggled by the joystick's built-in button:
 *
 * 1. RUN MODE:
 * - The motor's movement is restricted between a defined lower (0) and upper (10,000) bound.
 * - The position is tracked and printed to the Serial Monitor.
 *
 * 2. ADJUST MODE:
 * - The motor can move freely without any software limits.
 * - The Arduino's built-in LED will blink periodically to indicate this mode is active.
 * - The position is not tracked. This mode is used to manually move the motor
 * to a physical starting point or "zero" position.
 *
 * Switching from ADJUST MODE back to RUN MODE sets the current physical position
 * of the motor as the new logical zero (0) for position tracking.
 *
 * Hardware:
 * - Arduino or compatible microcontroller
 * - TB6600 Stepper Motor Driver
 * - Stepper Motor
 * - Joystick with a push-button
 */

// --- Pin Definitions ---
// TB6600 Driver Pins
const int stepPin = 2; // PUL+ pin
const int dirPin = 3;  // DIR+ pin
const int enPin = 4;   // ENA+ pin
const int buzzer = 9;   // Buzzer Pin

// Joystick Pins
const int joystickX = A0;  // VRx pin for X-axis movement
const int joystickBtn = 7; // SW pin for the push-button

// Indicator Pin (uses the built-in LED on most Arduino boards)
const int indicatorPin = LED_BUILTIN; // Typically pin 13

// --- Control Variables ---
int stepSpeed = 0;           // Delay in microseconds between steps (lower is faster)
long currentPosition = 0;    // Tracks the motor's position in steps during RUN MODE
int joystickValue = 0;       // Raw analog value from the joystick

// --- Constants ---
const int centerThreshold = 50;  // Joystick deadzone to prevent drift
const long lowerBound = 0;       // The minimum position in RUN MODE
const long upperBound = 15000;   // The maximum position in RUN MODE (135 degrees)

// --- Mode & Button Debounce Control ---
bool adjustMode = false; // Start in RUN MODE by default
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 300; // Prevents multiple button reads from one press

// --- LED Indicator Variables ---
unsigned long lastIndicatorOnTime = 0;
bool indicatorIsOn = false;
const unsigned long indicatorInterval = 750; // Blink every 750ms in adjust mode
const int indicatorOnDuration = 100;         // LED stays on for 100ms

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Stepper motor controller starting up...");

  // Configure pin modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(joystickBtn, INPUT_PULLUP); // Use internal pull-up resistor for the button
  pinMode(indicatorPin, OUTPUT);      // Set indicator LED pin as an output

  // Enable the driver by default (LOW signal enables it on many common drivers)
  digitalWrite(enPin, LOW);

  Serial.println("✅ RUN MODE active. Position is bounded.");
}

/**
 * @brief Executes a single step of the motor.
 * @param speed The delay in microseconds for the step pulse. Determines motor speed.
 * @param clockwise The direction of rotation (true for clockwise, false for counter-clockwise).
 */
void takeStep(int speed, bool clockwise) {
  digitalWrite(dirPin, clockwise ? HIGH : LOW);

  // Create a PULSE to trigger one step
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(speed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(speed);
}

void loop() {
  // --- 1. Handle Mode Change on Button Press ---
  if (digitalRead(joystickBtn) == LOW && millis() - lastButtonPress > debounceDelay) {
    lastButtonPress = millis();
    adjustMode = !adjustMode; // Toggle the mode

    if (adjustMode) {
      Serial.println("\n⚙️ ADJUST MODE ON. Motor is now free. Move to your desired zero position and press the button.\n");
    } else {
      // When exiting adjust mode, reset the position to establish the new zero.
      currentPosition = 0;
      Serial.println("\n✅ RUN MODE ON. Current position is now 0. Movement is bounded.\n");
      digitalWrite(indicatorPin, LOW); // Ensure the indicator LED is turned off
      indicatorIsOn = false;
    }
  }

  // --- 2. Visual Indicator for Adjust Mode (Non-blocking blink) ---
  if (adjustMode) {
    // Time to turn the indicator ON
    if (!indicatorIsOn && (millis() - lastIndicatorOnTime > indicatorInterval)) {
      lastIndicatorOnTime = millis();
      digitalWrite(indicatorPin, HIGH);
      tone(buzzer, 1000); // Send 1KHz sound signal...
      indicatorIsOn = true;
    }
    // Time to turn the indicator OFF
    if (indicatorIsOn && (millis() - lastIndicatorOnTime > indicatorOnDuration)) {
      digitalWrite(indicatorPin, LOW);
      noTone(buzzer); // Turn off buzzer
      indicatorIsOn = false;
    }
  }

  // --- 3. Read Joystick and Determine Speed/Direction ---
  joystickValue = analogRead(joystickX);
  int deviation = joystickValue - 512; // Calculate how far the joystick is pushed from its center

  // Only move if the joystick is outside the center deadzone
  if (abs(deviation) > centerThreshold) {
    digitalWrite(enPin, LOW); // Ensure motor is enabled

    bool clockwise = (deviation > 0);
    int absDeviation = abs(deviation);

    // Map the joystick's position to a step speed.
    // Further from center = smaller delay = faster speed.
    stepSpeed = map(absDeviation, centerThreshold, 512, 500, 100);
    stepSpeed = constrain(stepSpeed, 100, 500); // Clamp the speed to a safe/usable range

    // --- 4. Execute Movement Based on Current Mode ---
    if (adjustMode) {
      // In adjust mode, move freely without tracking position or checking bounds
      takeStep(stepSpeed, clockwise);
    } else {
      // In run mode, check bounds before moving
      if ((clockwise && currentPosition < upperBound) || (!clockwise && currentPosition > lowerBound)) {
        takeStep(stepSpeed, clockwise);
        // Update the position only when in run mode
        currentPosition += (clockwise ? 1 : -1);
      }
    }
  }

  // --- 5. Display Status and Position Periodically ---
  // Only print when the motor is stationary to prevent Serial commands
  // from interrupting the step pulses, which causes a "pulsing" sound.
  static unsigned long lastPrint = 0;
  if (abs(deviation) <= centerThreshold) {
    if (millis() - lastPrint > 250) {
      lastPrint = millis();

      // Print status and position, then move to the next line
      Serial.print(adjustMode ? "[ADJUST MODE] " : "[RUN MODE]    ");
      Serial.print("Position: ");
      if (adjustMode) {
        Serial.println("UNTRACKED");
      } else {
        Serial.println(currentPosition);
      }
    }
  }
}

