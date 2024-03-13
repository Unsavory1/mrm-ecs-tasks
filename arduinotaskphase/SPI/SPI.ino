#include <SPI.h>


int buttonpin = 2;
int buttonpin2 = 3;
int SS2 = 9;
volatile int Mastereceive = 255;
int Mastersend;
volatile int received;
int Mastereceive2;
volatile int Mastersend2 = 255;
volatile int  received2;
int ledpin = 4;
int ledpin2 = 5;
int reading;
bool send;
bool ledst;
int reading2;







void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  pinMode(SS2, OUTPUT);
  pinMode(ledpin, OUTPUT);
  pinMode(ledpin2, OUTPUT);
  pinMode(buttonpin, INPUT_PULLUP);
  pinMode(buttonpin2, INPUT_PULLUP);
}

void loop() {
  send = digitalRead(buttonpin);
  send2 = digitalRead(buttonpin2);


  digitalWrite(SS, LOW);
  if (send==HIGH)
    Mastersend = 255;
  else Mastersend = 0;
  Mastereceive = SPI.transfer(Mastersend);
  received = SPDR;
  digitalWrite(SS, HIGH);

  digitalWrite(SS2, LOW);
  if (send2==HIGH)
    Mastersend2 = 255;
  else Mastersend2 = 0;
  Mastereceive2 = SPI.transfer(Mastersend2);
  received2 = SPDR;
  digitalWrite(SS2, HIGH);

 /* Serial.print(send);
  Serial.print(",");
  Serial.println(send2);
*/
  ledst = received & 128;
  digitalWrite(ledpin, ledst);
  ledst2 = received2 & 128;
  digitalWrite(ledpin2, ledst2);


}
