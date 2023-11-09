const int button = 4; 
int buttonState;
int lastButtonState = HIGH; 
unsigned long a = 1; 
unsigned long b = 1; 
unsigned long c; 

void setup()
{
  pinMode(button, INPUT_PULLUP); 
  Serial.begin(9600); //begins serial commmunication.
  delay(1000); 
}

void loop()
{
  buttonState = digitalRead(button);
  // if button is just pressed
  if (buttonState == LOW && buttonState != lastButtonState)
  {
    c = a + b; // calculate next Fib. number
    printNumbers(a, b, c);
    a = b; // The last number becomes the second-to-last
    b = c; // The new number becomes the last number in the list
  }
  lastButtonState = buttonState; // store last button state
}

void printNumbers(unsigned long d, unsigned long e, unsigned long f)
{
  
  Serial.print(d); 
  Serial.print("\t");
  Serial.print("+");
  Serial.print("\t");
  Serial.print(e);
  Serial.print("\t");
  Serial.print("=");
  Serial.print("\t");
  Serial.println(f); 
}