// Enhanced Stepper Motor Control with TB6600 and Heartbeat
// Receives step numbers as strings via Serial and runs the motor accordingly
// Handles 'STOP' command to halt the motor

#define PIN_ENA   8
#define PIN_DIR   9
#define PIN_PUL   10

const unsigned long STEP_INTERVAL = 1500; // Microseconds between steps (1.5ms)
unsigned long previousStepTime = 0;

long stepsRemaining = 0;        // Number of steps remaining
bool isRunning = false;          // Motor running state
bool directionForward = true;    // Direction of motor

String inputString = "";         // Serial input buffer
bool stringComplete = false;     // Flag for complete command

// Heartbeat parameters
const unsigned long HEARTBEAT_INTERVAL = 1000000; // Microseconds between heartbeats (1 second)
unsigned long previousHeartbeatTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Enhanced Stepper Motor Control Ready");
  Serial.println("Enter step counts as numbers. Positive for forward, negative for backward.");
  Serial.println("Example: 1000 to move forward 1000 steps");
  Serial.println("         -500 to move backward 500 steps");
  Serial.println("         'STOP' to halt the motor");

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);

  // Enable the driver
  digitalWrite(PIN_ENA, LOW); // LOW typically enables the driver; verify with your TB6600

  // Set initial direction (forward)
  digitalWrite(PIN_DIR, directionForward ? LOW : HIGH);

  // Initialize timing variables
  previousStepTime = micros();
  previousHeartbeatTime = micros();
}

void loop() {
  unsigned long currentTime = micros();

  // Handle serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') { // Command terminated by newline
      stringComplete = true;
    } else if (inChar != '\r') { // Ignore carriage return
      inputString += inChar;
    }
  }

  // Process complete command
  if (stringComplete) {
    inputString.trim(); // Remove leading/trailing whitespace
    Serial.print("Received command: ");
    Serial.println(inputString);

    if (inputString.equalsIgnoreCase("STOP")) {
      stepsRemaining = 0;
      isRunning = false;
      Serial.println("Belt stopped.");
    }
    else {
      // Parse step count
      long stepCount = inputString.toInt(); // Convert to integer

      if (stepCount == 0 && inputString.charAt(0) != '0') { // Check for invalid input
        Serial.println("Invalid step count.");
      }
      else {
        if (stepCount > 0) {
          directionForward = true;
        }
        else if (stepCount < 0) {
          directionForward = false;
          stepCount = -stepCount; // Make step count positive
        }

        if (stepCount >= 0) {
          stepsRemaining = stepCount;
          isRunning = true;
          digitalWrite(PIN_DIR, directionForward ? LOW : HIGH);
          Serial.print("Belt moving ");
          Serial.print(directionForward ? "forward" : "backward");
          Serial.print(" for ");
          Serial.print(stepsRemaining);
          Serial.println(" steps.");
        }
      }
    }

    // Reset input buffer
    inputString = "";
    stringComplete = false;
  }

  // Handle motor stepping
  if (isRunning && stepsRemaining > 0) {
    if (currentTime - previousStepTime >= STEP_INTERVAL) {
      previousStepTime = currentTime;
      stepForward();
      stepsRemaining--;

      // Optionally, provide feedback every 1000 steps
      if (stepsRemaining % 1000 == 0) {
        Serial.print("Steps remaining: ");
        Serial.println(stepsRemaining);
      }

      // Check if stepping is complete
      if (stepsRemaining == 0) {
        isRunning = false;
        Serial.println("Belt run complete.");
      }
    }
  }

  // Handle heartbeat
  if (currentTime - previousHeartbeatTime >= HEARTBEAT_INTERVAL) {
    previousHeartbeatTime = currentTime;
    sendHeartbeat();
  }
}

// Function to perform a single step forward or backward
void stepForward() {
  digitalWrite(PIN_PUL, HIGH);
  delayMicroseconds(100); // Pulse width; adjust as necessary
  digitalWrite(PIN_PUL, LOW);
  // No additional delay needed; timing controlled by STEP_INTERVAL
}

// Function to send a heartbeat message
void sendHeartbeat() {
  unsigned long millisSinceStart = millis();
  Serial.print("Heartbeat: ");
  Serial.print(millisSinceStart);
  Serial.println(" ms since start");
}
