// =====================================================================
// Solenoid Controller — MOSFET board version
// Receives fret commands over Serial from Python FSM.
//
// Protocol (1 byte):
//   0   → all solenoids off (open string)
//   1-8 → activate that fret number
//
// Pin-to-fret mapping:
//   Pin 7 → fret 1
//   Pin 6 → fret 2
//   Pin 5 → fret 3
//   Pin 4 → fret 5
//   Pin 3 → fret 6
//   Pin 2 → fret 8
// =====================================================================

// Fret-to-pin lookup (index = fret number, value = pin, -1 = not wired)
//                     fret: 0   1   2   3   4   5   6   7   8
int fretToPin[9]         = {-1,  7,  6,  5, -1,  4,  3, -1,  2};

int allPins[] = {2, 3, 4, 5, 6, 7};
int numPins = 6;

int activeFret = 0;
unsigned long lastCmdMs = 0;
const unsigned long WATCHDOG_MS = 2000;  // release all if no command within 2s

void turnAllOff() {
  for (int i = 0; i < numPins; i++) {
    digitalWrite(allPins[i], LOW);
  }
  activeFret = 0;
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numPins; i++) {
    pinMode(allPins[i], OUTPUT);
  }
  turnAllOff();
}

void loop() {
  unsigned long now = millis();

  if (Serial.available() > 0) {
    uint8_t cmd = (uint8_t)Serial.read();
    lastCmdMs = now;

    turnAllOff();

    if (cmd >= 1 && cmd <= 8) {
      int pin = fretToPin[cmd];
      if (pin >= 0) {
        digitalWrite(pin, HIGH);
        activeFret = cmd;
      }
    }
  }

  // Watchdog: release all if no command received recently
  if (activeFret > 0 && (now - lastCmdMs >= WATCHDOG_MS)) {
    turnAllOff();
  }
}
