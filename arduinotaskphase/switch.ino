const int ledPin = 11;        
const int potPin = A1;        
const int buttonPin = 2;      

int brightness = 0;          
int potValue = 0;             
int buttonState = LOW;          
int lastButtonState = LOW;    // Variable to store the previous button state
unsigned long lastDebounceTime = 0;  // Variable for debounce
unsigned long debounceDelay = 50;     // Debounce time

void setup() {
  Serial.begin(9600); 
 
  //Serial.println("hello"); used it just to see if serial monitor was working. 
  pinMode(ledPin, OUTPUT);     // Set the LED pin as an output
  pinMode(buttonPin, INPUT);   // Set the button pin as an input
}

void loop() {
  // Read the potentiometer value
  potValue = analogRead(potPin);

  // Map the potentiometer value from the 0-1023 range to the 0-100 range
  brightness = map(potValue, 0, 1023, 0, 100);

  // Check if the button is pressed and debounced
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); //registers if changes are made 
    //ideally should register and store the value and serial monitor should pop up. 
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // If the button is pressed, switch to User Input Mode
      if (buttonState == LOW) {
        switchToUserInputMode();
      }
    }
  }

  // Set the LED brightness in Potentiometer Mode
  analogWrite(ledPin, map(brightness, 0, 100, 0, 255));
}

void switchToUserInputMode() {
  Serial.println("Switching to User Input Mode...");
  Serial.print("Enter LED brightness (0-100%) and press Enter: ");

  // Read the user input and set LED brightness
  brightness = Serial.parseInt();
  brightness = constrain(brightness, 0, 100);

  // Map the brightness value from the 0-100 range to the 0-255 range
  int mappedBrightness = map(brightness, 0, 100, 0, 255);

  // Set the LED brightness
  analogWrite(ledPin, mappedBrightness);

  // Print the current brightness to the Serial Monitor
  Serial.print("LED brightness set to: ");
  Serial.print(brightness);
  Serial.println("%");

  // Wait for a moment before returning to Potentiometer Mode
  delay(6000);
}

