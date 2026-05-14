#include <Arduino.h>

// --- Motor Pin Definitions ---
typedef enum { M_DATA=8, M_CLK=4, M_LATCH=12, M_ENABLE=7 } MotorPins;
typedef enum { M1_PWM=11, M2_PWM=3, M3_PWM=6, M4_PWM=5 } MotorPWM;
typedef enum { M4_A, M2_A, M1_A, M1_B, M2_B, M3_A, M4_B, M3_B, NONE=-1 } MotorBus;

MotorBus fret_to_bus[9] = {
    NONE, M1_A, M1_B, M2_A, M2_B, M3_A, M3_B, M4_A, M4_B
};

// --- Tunable constants ---
const unsigned int  STRIKE_MS    = 50;   // ms at full power to engage solenoid
const uint8_t       HOLD_PWM     = 100;  // hold power (0-255, ~40%) — tune to avoid overheating
const unsigned long WATCHDOG_MS  = 500;  // release all if no command within this delay

// --- FSM ---
enum State { IDLE, STRIKE, HOLD };
State         fsm_state      = IDLE;
int           active_fret    = 0;
unsigned long state_start_ms = 0;
unsigned long last_cmd_ms    = 0;

// -----------------------------------------------------------

void motor_send(MotorBus m, uint8_t pwm) {
    for (int i = 7; i >= 0; i--) {
        digitalWrite(M_DATA, (i == (int)m) ? HIGH : LOW);
        digitalWrite(M_CLK, HIGH);
        digitalWrite(M_CLK, LOW);
    }
    digitalWrite(M_LATCH, HIGH);
    delayMicroseconds(5);
    digitalWrite(M_LATCH, LOW);

    analogWrite(M1_PWM, (m == M1_A || m == M1_B) ? pwm : 0);
    analogWrite(M2_PWM, (m == M2_A || m == M2_B) ? pwm : 0);
    analogWrite(M3_PWM, (m == M3_A || m == M3_B) ? pwm : 0);
    analogWrite(M4_PWM, (m == M4_A || m == M4_B) ? pwm : 0);
}

void release_all() {
    motor_send(NONE, 0);
    fsm_state   = IDLE;
    active_fret = 0;
}

void activate_fret(int fret, unsigned long now) {
    if (fret < 1 || fret > 8) { release_all(); return; }
    motor_send(fret_to_bus[fret], 255);  // full power — STRIKE phase
    active_fret    = fret;
    fsm_state      = STRIKE;
    state_start_ms = now;  // same timestamp as loop to avoid unsigned wrap
}

// -----------------------------------------------------------

void setup() {
    pinMode(M_DATA,   OUTPUT);
    pinMode(M_CLK,    OUTPUT);
    pinMode(M_LATCH,  OUTPUT);
    pinMode(M_ENABLE, OUTPUT);
    digitalWrite(M_ENABLE, LOW);
    release_all();
    Serial.begin(115200);
}

void loop() {
    unsigned long now = millis();

    // Read incoming command (1 byte: 0=release all, 1-8=activate solenoid)
    if (Serial.available() > 0) {
        uint8_t cmd = (uint8_t)Serial.read();
        last_cmd_ms = now;
        if (cmd == 0) release_all();
        else          activate_fret((int)cmd, now);
    }

    // FSM transitions
    switch (fsm_state) {
        case IDLE:
            break;

        case STRIKE:
            if (now - state_start_ms >= STRIKE_MS) {
                motor_send(fret_to_bus[active_fret], HOLD_PWM);
                fsm_state = HOLD;
            }
            break;

        case HOLD:
            // Watchdog: release if controller stops sending commands
            if (now - last_cmd_ms >= WATCHDOG_MS)
                release_all();
            break;
    }
}
