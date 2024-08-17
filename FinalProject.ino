// Define motor control pins
#define ENA 5    // Enable pin for motor
#define IN1 6    // Input 1 pin for motor
#define IN2 7    // Input 2 pin for motor
#define BUTTON_PIN_1 2 // Pin for the first button
#define BUTTON_PIN_2 3 // Pin for the second button

int pwmValues[] = {210, 230, 243, 255};
int durations[] = {700, 650, 600, 500}; // in milliseconds
int currentPwmIndex = 0;

void setup() {
  // Initialize motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Initialize button pins as inputs with internal pull-ups
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the state of the first button
  int buttonState1 = digitalRead(BUTTON_PIN_1);
  // Read the state of the second button
  int buttonState2 = digitalRead(BUTTON_PIN_2);

  // Display button states in the Serial Monitor
  if (buttonState1 == LOW) {
    Serial.println("Button 1 pressed");
  }
  if (buttonState2 == LOW) {
    Serial.println("Button 2 pressed");
  }

  // If button 1 is pressed, run the motor at the current PWM value
  if (buttonState1 == LOW) {
    analogWrite(ENA, pwmValues[currentPwmIndex]);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(durations[currentPwmIndex]); // Run for specified duration
    // Stop the motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    delay(1000); // Wait for 1 second before restarting
  }

  // If button 2 is pressed, increment the PWM value index
  if (buttonState2 == LOW) {
    currentPwmIndex = (currentPwmIndex + 1) % (sizeof(pwmValues) / sizeof(pwmValues[0]));
    Serial.print("PWM value changed to: ");
    Serial.println(pwmValues[currentPwmIndex]);
    delay(500); // Debouncing delay
  }
}
