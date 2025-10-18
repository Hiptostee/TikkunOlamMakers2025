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
const int STEP_PIN = 2;   // PUL+ pin
const int DIR_PIN = 3;    // DIR+ pin
const int EN_PIN = 4;     // ENA+ pin
const int BUZZER_PIN = 9; // buzzer Pin

// Joystick Pins
const int JOYSTICK_X_PIN = A0;  // VRx pin for X-axis movement
const int JOYSTICK_BTN_PIN = 7; // SW pin for the push-button

// Indicator Pin (uses the built-in LED on most Arduino boards)
const int INDICATOR_PIN = LED_BUILTIN; // Typically pin 13

// --- Control Variables ---
int _stepSpeed = 0;                 // Delay in microseconds between steps (lower is faster)
long _currentPosition = 0;          // Tracks the motor's position in steps during RUN MODE
int _joystickValue = 0;             // Raw analog value from the joystick
bool _adjustMode = false;           // Start in RUN MODE by default
unsigned long _lastButtonPress = 0; // Time of last button press in milliseconds

// --- Constants ---
const int centerThreshold = 50;       // Joystick deadzone to prevent drift
const long lowerBound = 0;            // The minimum position in RUN MODE
const long upperBound = 15000;        // The maximum position in RUN MODE (135 degrees)
const long buttonDebounceDelay = 300; // Prevents multiple button reads from one press

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Stepper motor controller starting up...");

  // Configure pin modes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP); // Use internal pull-up resistor for the button

  // Enable the driver by default (LOW signal enables it on many common drivers)
  digitalWrite(EN_PIN, LOW);

  Serial.println("✅ RUN MODE active. Position is bounded.");
}

/**
 * @brief Executes a single step of the motor.
 * @param speed The delay in microseconds for the step pulse. Determines motor speed.
 * @param clockwise The direction of rotation (true for clockwise, false for counter-clockwise).
 */
void takeStep(int speed, bool clockwise) {
  //Serial.println(clockwise);
  //Serial.println(speed);
  //digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

  // Create a PULSE to trigger one step
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(speed);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(speed);
}

// Helper pre-declarations
void Detect_AdjustMode();
void Handle_AdjustMode();
void HandleJoystick();
void DisplayPositionInformation();

void loop() {
  Detect_AdjustMode();
  Handle_AdjustMode();
  HandleJoystick();
  DisplayPositionInformation();
  //takeStep(500,true);
}

void Detect_AdjustMode()
{
  // --- 1. Handle Mode Change on Button Press ---
  // TODO: use state machine to remove need for buttonDebounceDelay and fix infinite mode toggel issue with button press
  if (digitalRead(JOYSTICK_BTN_PIN) == LOW && millis() - _lastButtonPress > buttonDebounceDelay) 
  {
    _lastButtonPress = millis();
    _adjustMode = !_adjustMode; // Toggle the mode

    if (_adjustMode) 
    {
      Serial.println("\n⚙️ ADJUST MODE ON. Motor is now free. Move to your desired zero position and press the button.\n");
    } 
    else 
    {
      // When exiting adjust mode, reset the position to establish the new zero.
      _currentPosition = 0;
      //Serial.println("\n✅ RUN MODE ON. Current position is now 0. Movement is bounded.\n");
      digitalWrite(INDICATOR_PIN, LOW); // Ensure the indicator LED is turned off
      indicatorIsOn = false;
    }
  }
}

void Handle_AdjustMode()
{
  // --- 2. Visual Indicator for Adjust Mode (Non-blocking blink) ---
  if (_adjustMode) {
    // Time to turn the buzzer ON
    if (!indicatorIsOn && (millis() - lastIndicatorOnTime > indicatorInterval)) {
      lastIndicatorOnTime = millis();
      tone(BUZZER_PIN, 2000); // Send 1KHz sound signal...
      indicatorIsOn = true;
    }
    // Time to turn the indicator OFF
    if (indicatorIsOn && (millis() - lastIndicatorOnTime > indicatorOnDuration)) {
      digitalWrite(INDICATOR_PIN, LOW);
      noTone(BUZZER_PIN); // Turn off BUZZER_PIN
      indicatorIsOn = false;
    }
  }
}

void HandleJoystick()
{
  // --- 3. Read Joystick and Determine Speed/Direction ---
  _joystickValue = analogRead(JOYSTICK_X_PIN);
  _joystickValue = _joystickValue - 512; // Calculate how far the joystick is pushed from its center
  //Serial.println("HANDLE JOYSTICK");
  //Serial.println(_joystickValue);
  // Only move if the joystick is outside the center deadzone
  if (abs(_joystickValue) > centerThreshold) {
    digitalWrite(EN_PIN, LOW); // Ensure motor is enabled

    bool clockwise = (_joystickValue > 0);
    int abs_joystickValue = abs(_joystickValue);

    // Map the joystick's position to a step speed.
    // Further from center = smaller delay = faster speed.
    _stepSpeed = map(abs_joystickValue, centerThreshold, 512, 500, 100);
    _stepSpeed = constrain(_stepSpeed, 100, 500); // Clamp the speed to a safe/usable range

    // --- 4. Execute Movement Based on Current Mode ---
    if (_adjustMode) {
      // In adjust mode, move freely without tracking position or checking bounds
      takeStep(_stepSpeed, clockwise);
    } else {
      // In run mode, check bounds before moving
      if ((clockwise && _currentPosition < upperBound) || (!clockwise && _currentPosition > lowerBound)) {
        //Serial.println("STEP SPEED");
        //Serial.println(_stepSpeed);
        takeStep(_stepSpeed, clockwise);
        // Update the position only when in run mode
        _currentPosition += (clockwise ? 1 : -1);
      }
    }
  }
}

void DisplayPositionInformation()
{
  // --- 5. Display Status and Position Periodically ---
  // Only print when the motor is stationary to prevent Serial commands
  // from interrupting the step pulses, which causes a "pulsing" sound.
  static unsigned long lastPrint = 0;
  if (false)
  {
  if (abs(_joystickValue) <= centerThreshold) {
    if (millis() - lastPrint > 250) {
      lastPrint = millis();

      // Print status and position, then move to the next line
      Serial.print(_adjustMode ? "[ADJUST MODE] " : "[RUN MODE]    ");
      Serial.print("Position: ");
      if (_adjustMode) {
        Serial.println("UNTRACKED");
      } else {
        Serial.println(_currentPosition);
      }
    }
  }
  }
}
