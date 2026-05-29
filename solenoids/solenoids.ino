// =====================================================================
// HARDWARE SETUP
// =====================================================================
// Define the digital pins connected to your MOSFET boards. 
// Adjust these if you plug them into different pins!
int solenoidPins[] = {2, 3, 4, 5, 6, 7};
int numSolenoids = 6;

// Helper function to easily shut everything down
void turnAllOff() {
for(int i = 0; i < numSolenoids; i++) {
digitalWrite(solenoidPins[i], LOW); // LOW turns the MOSFET/Solenoid OFF
}
}

void setup() {
// Start the Serial monitor for debugging and for Test 4
Serial.begin(9600);
// Set all solenoid pins to OUTPUT and ensure they start OFF
for(int i = 0; i < numSolenoids; i++) {
pinMode(solenoidPins[i], OUTPUT);
}
turnAllOff();
}

void loop() {
// =====================================================================
// TEST 0: Simple Blink of the FIRST Solenoid (Hardcoded)
// =====================================================================
// This test ignores the arrays and just targets the very first pin (Pin 2).
// It is the simplest way to verify your first solenoid and diode wiring.
// digitalWrite(5, HIGH); // Turn ON the solenoid wired to Pin 2
// Serial.println("Test 0: First Solenoid (Pin 2) is ON.");
// delay(2000); // Keep it on for 1 second
// digitalWrite(5, LOW); // Turn OFF the solenoid
// Serial.println("Test 0: First Solenoid (Pin 2) is OFF.");
// delay(3000); // Keep it off for 1 second before repeating


// =====================================================================
// TEST 1: Sending a command to keep the solenoids off
// =====================================================================
/*
turnAllOff();
Serial.println("Test 1: All solenoids commanded OFF.");
delay(2000); // Wait 2 seconds before looping again
*/


// =====================================================================
// TEST 2: Turn on ONE solenoid while keeping others off
// =====================================================================
// int targetIndex = 0; // Change this from 0 to 5 to test different solenoids

// turnAllOff(); // Safety first
// digitalWrite(solenoidPins[targetIndex], HIGH); // Turn the target ON
// Serial.print("Test 2: Solenoid at index ");
// Serial.print(targetIndex);
// Serial.println(" is ON.");
// delay(2000); // Leave it on for 2 seconds
// turnAllOff(); // Turn it off
// Serial.println("Solenoid is OFF.");
// delay(2000); // Pause before repeating


// =====================================================================
// TEST 3: Turn the solenoids on sequentially
// =====================================================================
for(int i = 0; i < numSolenoids; i++) {
turnAllOff(); // Ensure others are off
digitalWrite(solenoidPins[i], HIGH);
Serial.print("Test 3: Sequential - Solenoid ");
Serial.print(i + 1);
Serial.println(" ON.");
delay(1000); // Keep ON for 1 second
digitalWrite(solenoidPins[i], LOW); // Turn OFF
delay(500); // Brief half-second pause between clicks
}


// =====================================================================
// TEST 4: Listening to the Computer (for Redis integration)
// =====================================================================
/*
// Check if the computer has sent data over the USB cable
if (Serial.available() > 0) {
char incomingByte = Serial.read();

// If we receive a '0', turn everything off
if (incomingByte == '0') {
turnAllOff();
Serial.println("Command Received: ALL OFF");
} 
// If we receive a number between '1' and '6'
else if (incomingByte >= '1' && incomingByte <= '6') {
turnAllOff(); // Turn off previously active solenoids
// Convert the character to an array index (e.g., '1' becomes 0)
int index = incomingByte - '1'; 
digitalWrite(solenoidPins[index], HIGH);
Serial.print("Command Received: Solenoid ");
Serial.print(incomingByte);
Serial.println(" ON");
}
}
*/
}