/*
  Arduino Nano - 3 Pin Input/Output Mirror
  - Reads 3 digital input pins
  - Mirrors the state to 3 corresponding output pins
  - Simple and efficient design
*/

// ===== PIN CONFIGURATION =====
// Input Pins (reading from sensors/switches/buttons)
#define INPUT_PIN_1  2   // Digital pin 2
#define INPUT_PIN_2  3   // Digital pin 3
#define INPUT_PIN_3  4   // Digital pin 4

// Output Pins (controlling relays/LEDs/actuators)
#define OUTPUT_PIN_1 8   // Digital pin 8
#define OUTPUT_PIN_2 9   // Digital pin 9
#define OUTPUT_PIN_3 10  // Digital pin 10

// ===== STATE VARIABLES =====
bool input1State = LOW;
bool input2State = LOW;
bool input3State = LOW;

bool lastInput1State = LOW;
bool lastInput2State = LOW;
bool lastInput3State = LOW;

// Debounce timing
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce

// ===== SETUP =====
void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("=== Arduino Nano Input/Output Mirror ===");
  
  // Configure Input Pins with internal pull-up resistors
  pinMode(INPUT_PIN_1, INPUT_PULLUP);
  pinMode(INPUT_PIN_2, INPUT_PULLUP);
  pinMode(INPUT_PIN_3, INPUT_PULLUP);
  
  // Configure Output Pins
  pinMode(OUTPUT_PIN_1, OUTPUT);
  pinMode(OUTPUT_PIN_2, OUTPUT);
  pinMode(OUTPUT_PIN_3, OUTPUT);
  
  // Initialize outputs to LOW
  digitalWrite(OUTPUT_PIN_1, LOW);
  digitalWrite(OUTPUT_PIN_2, LOW);
  digitalWrite(OUTPUT_PIN_3, LOW);
  
  Serial.println("Input Pins: 2, 3, 4 (with pull-up)");
  Serial.println("Output Pins: 8, 9, 10");
  Serial.println("Ready!\n");
}

// ===== MAIN LOOP =====
void loop() {
  // Read all input pins
  bool reading1 = digitalRead(INPUT_PIN_1);
  bool reading2 = digitalRead(INPUT_PIN_2);
  bool reading3 = digitalRead(INPUT_PIN_3);
  
  // ===== INPUT 1 PROCESSING =====
  if (reading1 != lastInput1State) {
    lastDebounceTime1 = millis();
  }
  
  if ((millis() - lastDebounceTime1) > DEBOUNCE_DELAY) {
    if (reading1 != input1State) {
      input1State = reading1;
      
      // Mirror to output (invert if using pull-up)
      // If input is LOW (pressed/active), output HIGH
      digitalWrite(OUTPUT_PIN_1, !input1State);
      
      Serial.print("Input 1: ");
      Serial.print(input1State ? "HIGH" : "LOW");
      Serial.print(" -> Output 1: ");
      Serial.println(!input1State ? "HIGH" : "LOW");
    }
  }
  lastInput1State = reading1;
  
  // ===== INPUT 2 PROCESSING =====
  if (reading2 != lastInput2State) {
    lastDebounceTime2 = millis();
  }
  
  if ((millis() - lastDebounceTime2) > DEBOUNCE_DELAY) {
    if (reading2 != input2State) {
      input2State = reading2;
      
      digitalWrite(OUTPUT_PIN_2, !input2State);
      
      Serial.print("Input 2: ");
      Serial.print(input2State ? "HIGH" : "LOW");
      Serial.print(" -> Output 2: ");
      Serial.println(!input2State ? "HIGH" : "LOW");
    }
  }
  lastInput2State = reading2;
  
  // ===== INPUT 3 PROCESSING =====
  if (reading3 != lastInput3State) {
    lastDebounceTime3 = millis();
  }
  
  if ((millis() - lastDebounceTime3) > DEBOUNCE_DELAY) {
    if (reading3 != input3State) {
      input3State = reading3;
      
      digitalWrite(OUTPUT_PIN_3, !input3State);
      
      Serial.print("Input 3: ");
      Serial.print(input3State ? "HIGH" : "LOW");
      Serial.print(" -> Output 3: ");
      Serial.println(!input3State ? "HIGH" : "LOW");
    }
  }
  lastInput3State = reading3;
  
  // Small delay for stability
  delay(10);
}
