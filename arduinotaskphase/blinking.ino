//C++ code
//
int ledPin=12;
void setup()
{
  pinMode(12, OUTPUT);
}

void loop()
{
  digitalWrite(ledPin,HIGH);
  delay(1000);
  digitalWrite(ledPin,LOW);
  delay(1000);
}
