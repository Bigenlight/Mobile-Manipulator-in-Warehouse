// Enhanced Stepper Motor Control with TB6600
// Responds to 'on' and 'off' commands via Serial

#define PIN_ENA   8
#define PIN_DIR   9
#define PIN_PUL   10

const unsigned long STEP_INTERVAL = 1500; // Microseconds between steps (1ms)
unsigned long previousStepTime = 0;

bool running = false;          // Motor running state
String inputString = "";       // Serial input buffer
bool stringComplete = false;   // Flag for complete command

void setup() {
  Serial.begin(115200);
  Serial.println("Enhanced Stepper Motor Control Ready");

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);

  // Enable the driver
  digitalWrite(PIN_ENA, LOW); // LOW typically enables the driver; verify with your TB6600

  // Set initial direction (forward)
  digitalWrite(PIN_DIR, LOW); // Change as needed for your motor's direction

  // Initialize previousStepTime
  previousStepTime = micros();
}

void loop() {
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

    if (inputString.equalsIgnoreCase("on")) {
      running = true;
      Serial.println("Belt moving forward.");
    }
    else if (inputString.equalsIgnoreCase("off")) {
      running = false;
      Serial.println("Belt stopped.");
    }
    else {
      Serial.println("Unknown command.");
    }

    // Reset input buffer
    inputString = "";
    stringComplete = false;
  }

  // Handle motor stepping
  if (running) {
    unsigned long currentTime = micros();
    if (currentTime - previousStepTime >= STEP_INTERVAL) {
      previousStepTime = currentTime;
      stepForward();
    }
  }
}

void stepForward() {
  digitalWrite(PIN_PUL, HIGH);
  delayMicroseconds(100); // Minimum pulse width (10µs)
  digitalWrite(PIN_PUL, LOW);
  // No additional delay needed; timing controlled by STEP_INTERVAL
}
